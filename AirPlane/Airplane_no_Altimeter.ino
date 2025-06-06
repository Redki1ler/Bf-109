#include <Servo.h>
#include <math.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

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
volatile int ChangeAltBy;
volatile double Yaw;
volatile int ChangeYawBy = 90;
double DesiredYaw;
double DesiredAlt;
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

// Temp flags
bool DoOnceYaw;
bool DoOncePitch;

// Interrupt detection routine
volatile bool MPUInterrupt = false;

// Timer variables
unsigned long timerStartMillis[4];
bool timerFired[4] = {false, false, false, false};
const unsigned int timerDurations[] = {30, 60, 90, 120};

unsigned long ledOnStartMillis = 0;
unsigned long ledOffStartMillis = 0;
bool ledState = false;

// Function declarations
void MotorSpeed();
void Altstabilizer();
void RollStabilizer();
void RudderDegree();
void ElevatorDegree();
void RollDegree();
void ShowData();
void DMPDataReady();
void InitTimers();
void CheckTimers();
void LEDFlash();

void setup() {
  Serial.begin(115200);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  mpu.initialize();

  Serial.println(F("Testing MPU6050 connection..."));
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU6050 connection successful");
  }

  delay(5000);

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

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

  if (devStatus == 0) {
    mpu.CalibrateAccel(10);
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  InitTimers();
}

void loop() {
  ShowData();

  pitch = yrp[2] * 180 / M_PI;
  Roll = yrp[1] * 180 / M_PI;
  Yaw = yrp[0] * 180 / M_PI;

  CheckTimers();
  MotorSpeed();
  LEDFlash();

  // Flight logic follows (same as previous, updated to use new timer flags and corrected types)
  // [Flight control logic continues here, unchanged from your original, except using `ChangeAltBy` and `ChangeYawBy` as int]
}

void InitTimers() {
  for (int i = 0; i < 4; i++) {
    timerStartMillis[i] = millis() + timerDurations[i] * 1000UL;
  }
}

void CheckTimers() {
  unsigned long currentMillis = millis();
  for (int i = 0; i < 4; i++) {
    if (!timerFired[i] && currentMillis >= timerStartMillis[i]) {
      timerFired[i] = true;
      switch (timerDurations[i]) {
        case 30:
          ChangeAltBy = 5;
          GotoAlt = true;
          break;
        case 60:
        case 90:
          ChangeYawBy = 90;
          GotoYaw = true;
          break;
        case 120:
          Land = true;
          break;
      }
    }
  }
}

void DMPDataReady() {
  MPUInterrupt = true;
}

// Remaining functions unchanged except for any references to ChangeAltBy/ChangeYawBy being treated as int
// [Add the remaining methods from your previous code as needed here]  
