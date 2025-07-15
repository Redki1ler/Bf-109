#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

/* 
  speed units is always in m/s
  distance units are always in meter
*/

// Constants
const byte MPU_ADDR = 0x68;
const byte INTERRUPT_PIN = 13;
const double stallSpeed = 30;
const double PI_OVER_180 = M_PI / 180.0;

// Pin definitions
const byte ElevatorPin = 2;
const byte RudderPin = 3;
const byte RightAileronPin = 4;
const byte LeftAileronPin = 5;
const byte MotorPin = 6;
const byte LeftLedPin = 7;
const byte RightLedPin = 8;

// Variables
volatile double Altitude;
volatile byte ChangeAltBy;
volatile double Yaw;
volatile int ChangeYawBy;
double DesiredYaw;
double DesiredAlt;
double AltError;
double YawError;
double Speed;
double pitch;
double Roll;
volatile byte RudderAngle;
volatile byte ElevatorAngleStabilizer;
volatile byte ElevatorAngleGotoAlt;
volatile byte RightAileronAngle;
volatile byte RightAileronStabilizer;
volatile byte LeftAileronAngle;
volatile byte LeftAileronStabilizer;

// Control flags
volatile bool GotoYaw;
volatile bool GotoAlt;
volatile bool Elevate; 
volatile bool MaxSpeed;
volatile bool Land;
volatile bool LedFlash[2];

// Servo objects
Servo Motor;
Servo Elevator;
Servo Rudder;
Servo RightAileron;
Servo LeftAileron;

// MPU6050 objects and variables
MPU6050 mpu;
bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];
Quaternion q;
VectorFloat gravity;
float yrp[3];

//BMP280
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
unsigned BMPstatus;

//temp
bool DoOnceYaw;
bool DoOncePitch;

// Interrupt detection routine
volatile bool MPUInterrupt = false;

//Timer variables
unsigned long timerStartMillis = 0; // Variable to store the start time when the timer is started
unsigned long timerStartPitch = 0; // Start time for Pitch control
volatile unsigned long ledOnStartMillis = 0;  // Start time for LED ON state
volatile unsigned long ledOffStartMillis = 0; // Start time for LED OFF state
volatile bool ledState = false;               // Current state of the LEDs (false = OFF, true = ON)

void setup() {
  Serial.begin(115200);

  pinMode(ElevatorPin, OUTPUT);
  pinMode(RudderPin, OUTPUT);
  pinMode(RightAileronPin, OUTPUT);
  pinMode(LeftAileronPin, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(MotorPin, OUTPUT);
  pinMode(RightLedPin, OUTPUT);
  pinMode(LeftLedPin, OUTPUT);
  Elevator.attach(ElevatorPin);
  Rudder.attach(RudderPin);
  RightAileron.attach(RightAileronPin);
  LeftAileron.attach(LeftAileronPin);
  Motor.attach(MotorPin);

  BMP280Startup();
  MPUStartup();

  timerStartMillis = millis(); // Record the Start time to compare later
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                         // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false);              // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, true);  // request a total of 7*2=14 registers

  ShowData();

  Altitude = bmp.readAltitude(1013.25);
  pitch = yrp[2] * 180 / M_PI;
  Roll = yrp[1] * 180 / M_PI;
  //Yaw = yrp[0] * 180 / M_PI;

  //Flight Path
  if(timerStartMillis > 30000){
    Land = true;
  }else if(timerStartMillis > 60000){
    ChangeYawBy = 90;
    GotoYaw = true;
  }else if(timerStartMillis > 90000){
    ChangeYawBy = 90;
    GotoYaw = true;
  }else if(timerStartMillis > 120000){
    ChangeAltBy = 5;
    GotoAlt = true;
  }

  MotorSpeed();
  LEDFlash();

  if (GotoYaw) {
    RollDegree();
    if (GotoAlt) {
      // GotoYaw and GotoAlt are both true
      ElevatorDegree();
      if (AltError > 0) {
        Elevator.write(135);  // Climb
      } else {
        Elevator.write(ElevatorAngleGotoAlt);  // Adjust for altitude change
      }
      // Set ailerons to the calculated angle
      if(!Land){
        RightAileron.write(RightAileronAngle);
        LeftAileron.write(LeftAileronAngle);
      }else{
        RightAileron.write(RightAileronStabilizer);
        LeftAileron.write(LeftAileronStabilizer);
      }
    } else {
      // GotoYaw is true, but GotoAlt is false
      // Actively adjust yaw using rudder and ailerons

      // Adjust elevator based on Roll
      if (abs(Roll) > 5 && abs(Roll) <= 80) {
        Elevator.write(135);  // Roll is moderate, adjust elevator
        RightAileron.write(RightAileronAngle);
        LeftAileron.write(LeftAileronAngle);
      } else if (abs(Roll) > 80 && abs(Roll) < 180) {
        Elevator.write(45);  // Roll is extreme, adjust elevator
        RightAileron.write(RightAileronStabilizer);
        LeftAileron.write(LeftAileronStabilizer);
      } else {
        Elevator.write(90);  // Roll is neutral, keep elevator level
        RightAileron.write(RightAileronAngle);
        LeftAileron.write(LeftAileronAngle);
      }
    }
  } else {
    if (GotoAlt) {
      // GotoYaw is false, but GotoAlt is true
      // Stabilize ailerons and adjust elevator for altitude
      ElevatorDegree();
      if(ChangeAltBy > -4){
        RightAileron.write(RightAileronStabilizer);
        LeftAileron.write(LeftAileronStabilizer);
      }else{
        RightAileron.write(RightAileronAngle);
        LeftAileron.write(LeftAileronAngle);
      }

      Elevator.write(ElevatorAngleGotoAlt);
    } else {
      // Both GotoYaw and GotoAlt are false
      Elevate = true;  // Enable elevation mode
    }
  }

  RudderDegree();
  Rudder.write(RudderAngle);

  if (Elevate == true) {
    GotoYaw = false;
    GotoAlt = false;
    Altstabilizer();
    RollStabilizer();
    Elevator.write(ElevatorAngleStabilizer);
    RightAileron.write(RightAileronStabilizer);
    LeftAileron.write(LeftAileronStabilizer);
  }
}

void MotorSpeed() {
  if(MaxSpeed){
    Motor.write(180); //100%
  }else{
    if(Land){
      Motor.write(54); //30%
    }else if(Elevate){
      Motor.write(126); //70%
    }else{
      Motor.write(162); //90%
    }
  }
}

void Altstabilizer() {
  if ((pitch < 5 && pitch > 0)) {
    ElevatorAngleStabilizer = 90.0;
  }else if ((pitch < 20.0 || pitch > -20.0) && (pitch > 5 || pitch < 0)) {
    ElevatorAngleStabilizer = 90 - pitch;
    ElevatorAngleStabilizer = constrain(ElevatorAngleStabilizer, 45, 135);
  }else if (pitch > 20.0) {
    ElevatorAngleStabilizer = 45;
  }else if (pitch < -20.0) {
    ElevatorAngleStabilizer = 135;
  }
}

void RollStabilizer() {
  if (abs(Roll) > 4 && abs(Roll) < 90) {
    RightAileronStabilizer = 90 - (Roll / 2);
    LeftAileronStabilizer = 90 + (Roll / 2);
  } else if (Roll > 90 && Roll < 180) {
    RightAileronStabilizer = 45;
    LeftAileronStabilizer = 135;
  } else if (Roll < -90 && Roll > -180) {
    RightAileronStabilizer = 135;
    LeftAileronStabilizer = 45;
  } else {
    RightAileronStabilizer = LeftAileronStabilizer = 90;
  }
}

void RudderDegree() {
  if (GotoYaw) {
    if (abs(Roll) < 20) {
      YawError < 0 ? RudderAngle = 45 : RudderAngle = 135;
    } else if (abs(Roll) <= 90) {
      RudderAngle = 90 + (Roll / 2);
    } else {
      Roll < 0 ? RudderAngle = 45 : RudderAngle = 135;
    }
  } else {
    if (abs(Roll) < 3.5) {
      RudderAngle = 90;
    } else if (Roll > 0 && Roll < 110) {
      RudderAngle = 90 + Roll;
    } else if (Roll < 0 && Roll > -110) {
      RudderAngle = 90 + Roll;
    } else {
      RudderAngle = 90;
    }
  }
}

void ElevatorDegree() {
  if (!DoOncePitch) { 
    DesiredAlt = Altitude + ChangeAltBy;
    DoOncePitch = true;                   // Set the flag to true
  }

  ChangeAltBy = round(ChangeAltBy);

  AltError = DesiredAlt - Altitude;

  if (abs(AltError) < 0.2) {
    if (pitch > 0 && pitch < 1.5) {
      ElevatorAngleGotoAlt = 90;
      GotoAlt = false;
      DoOncePitch = false;
    } else {
      ElevatorAngleGotoAlt = ElevatorAngleStabilizer; //thieft
    }
  } else {
    if (abs(pitch) > 40 && abs(pitch) < 50) {
      ElevatorAngleGotoAlt = 90;
      MaxSpeed = true;
    } else {
      MaxSpeed = true;
      if (AltError > 5) {
        ElevatorAngleGotoAlt = 135;
      } else if (AltError < -5) {
        if (Roll > -10 && Roll < 10) {
          ElevatorAngleGotoAlt = 45;
        }else if(Roll > 170 && Roll < -170){
          ElevatorAngleGotoAlt = 135;
        }else {
          ElevatorAngleGotoAlt = 90;
        }
      } else {
        switch (round(AltError)) {
          case 4:
            ElevatorAngleGotoAlt = 120;
            break;
          case 3:
            ElevatorAngleGotoAlt = 110;
            break;
          case 2:
            ElevatorAngleGotoAlt = 102;
            break;
          case 1:
            ElevatorAngleGotoAlt = 97;
            break;
          case -1:
            ElevatorAngleGotoAlt = 81;
            break;
          case -2:
            ElevatorAngleGotoAlt = 71;
            break;
          case -3:
            ElevatorAngleGotoAlt = 60;
            break;
          case -4:
            ElevatorAngleGotoAlt = 50;
            break;
          default:
            break;
        }
      }
    }
  }
}

void RollDegree() {
  if (!DoOnceYaw) { //Refresh every 10 seconds
    // Set the target yaw only once when starting the turn
    DesiredYaw = Yaw + ChangeYawBy;  // ChangeYawBy is the desired yaw change
    DoOnceYaw = true;                // Set the flag to true
  }

  YawError = DesiredYaw - Yaw;  // Calculate yaw error

  if (abs(YawError) < 5) {  // Dead zone for small yaw errors
    // Yaw is close to the target, stabilize the ailerons
    if (abs(Roll) < 3) {
      RightAileronAngle = 90;
      LeftAileronAngle = 90;
      DoOnceYaw = false;  // Reset the flag for the next yaw adjustment
    } else {              //stabilize the craft instead of leaving it tilted
      RightAileronAngle = RightAileronStabilizer;
      LeftAileronAngle = LeftAileronStabilizer;
    }

  } else if (ChangeAltBy < -5) {
    if (Roll > 170 && Roll < -170) {  //keep aircraft flipped For altitude Change
      RightAileronAngle = 90;
      LeftAileronAngle = 90;
    } else {
      if (Roll > 0) {
        RightAileronAngle = 45;
        LeftAileronAngle = 135;
      } else {
        RightAileronAngle = 135;
        LeftAileronAngle = 45;
      }
    }
  } else {
    // Use ailerons to roll the aircraft toward the desired yaw

    MaxSpeed = true;

    if (abs(Roll) > 40 && abs(Roll) < 50) {  //keep aircraft tilted to change Yaw
      RightAileronAngle = 90;
      LeftAileronAngle = 90;
    } else {
      RightAileronAngle = 90 - (YawError / 2);                    // Adjust right aileron
      LeftAileronAngle = 90 + (YawError / 2);                     // Adjust left aileron
      RightAileronAngle = constrain(RightAileronAngle, 45, 135);  // Limit angles
      LeftAileronAngle = constrain(LeftAileronAngle, 45, 135);    // Limit angles
    }
  }
}

void LEDFlash() { //right light is green & left light is red
  if (LedFlash[0]) {
    // Fast flash: 0.4s ON, 0.4s OFF
    if (ledState) {
      // LEDs are ON — check if time to turn OFF
      if (millis() - ledOnStartMillis >= 400) {
        digitalWrite(RightLedPin, LOW);
        digitalWrite(LeftLedPin, LOW);
        ledOffStartMillis = millis(); // Record OFF start time
        ledState = false;
      }
    } else {
      // LEDs are OFF — check if time to turn ON
      if (millis() - ledOffStartMillis >= 400) {
        digitalWrite(RightLedPin, HIGH);
        digitalWrite(LeftLedPin, HIGH);
        ledOnStartMillis = millis(); // Record ON start time
        ledState = true;
      }
    }
  } else if (LedFlash[1]) {
    // Slow flash: 1s ON, 1s OFF
    if (ledState) {
      if (millis() - ledOnStartMillis >= 1000) {
        digitalWrite(RightLedPin, LOW);
        digitalWrite(LeftLedPin, LOW);
        ledOffStartMillis = millis();
        ledState = false;
      }
    } else {
      if (millis() - ledOffStartMillis >= 1000) {
        digitalWrite(RightLedPin, HIGH);
        digitalWrite(LeftLedPin, HIGH);
        ledOnStartMillis = millis();
        ledState = true;
      }
    }
  } else if (LedFlash[2]) {
    // Alternating flash: 0.4s RIGHT ON, 0.4s LEFT ON
    if (ledState) {
      if (millis() - ledOnStartMillis >= 400) {
        digitalWrite(RightLedPin, LOW);
        digitalWrite(LeftLedPin, HIGH); // Switch to LEFT LED
        ledOffStartMillis = millis();
        ledState = false;
      }
    } else {
      if (millis() - ledOffStartMillis >= 400) {
        digitalWrite(RightLedPin, HIGH); // Switch to RIGHT LED
        digitalWrite(LeftLedPin, LOW);
        ledOnStartMillis = millis();
        ledState = true;
      }
    }
  }
}

void MPUStartup() {
    Wire.beginTransmission(MPU_ADDR);  // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(0x6B);                  // PWR_MGMT_1 register
    Wire.write(0);                     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    mpu.initialize();

    /*Verify connection*/
    Serial.println(F("Testing MPU6050 connection..."));
    if (mpu.testConnection() == false) {
      Serial.println("MPU6050 connection failed");
      while (1){
        delay(2500);
        Serial.println("MPU6050 connection failed!");
      }
    } else {
      Serial.println("MPU6050 connection successful");
    }

    /* Initializate and configure the DMP*/
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    /* Making sure the mpu6050 worked (returns 0 if so) */
    if (devStatus == 0) {
      mpu.CalibrateAccel(10);  // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateGyro(6);
      Serial.println("These are the Active offsets: ");
      mpu.PrintActiveOffsets();
      Serial.println(F("Enabling DMP..."));  //Turning ON DMP
      mpu.setDMPEnabled(true);

      /*Enable Arduino interrupt detection*/
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      MPUIntStatus = mpu.getIntStatus();

      /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      DMPReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();  //Get expected DMP packet size for later comparison
    } else {
      Serial.print(F("DMP Initialization failed (code "));  //Print the error code
      Serial.print(devStatus);
      Serial.println(F(")"));
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
    }
}

void BMP280Startup(){
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  BMPstatus = bmp.begin();
  if (!BMPstatus) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void DMPDataReady() {
  MPUInterrupt = true;
}

void ShowData() {
  // print out data
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {  // Get the Latest packet

    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
    Serial.print("yrp\t");
    Serial.print(yrp[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(yrp[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(yrp[2] * 180 / M_PI);
  }
  
  //BMP280
  /*Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");*/

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");
}