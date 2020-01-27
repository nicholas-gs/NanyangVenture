/*
 * allTeensy Code Version 5.0.0
 * This code is specific to the teensy 3.5
 * Author: Shen Chen
 * 
 */

#include <Filters.h>
#include <AccelStepper.h>
#include <SPI.h>
#include <AS5048A.h>
//ROS Libraries
#include <ros.h>
#include <std_msgs/Float32.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
// CAN Bus
#include <TeensyCANSerializer.h>

// CONFIGURATION PARAMETERS
// Defines if ROS is being used or remote is being used
// #define ROSMODE
// Defines if control is via position (steps) or via velocity
// #define STEPMODE

// CAN Bus configuration
// #define CANTX 2
// #define CANRX 35

// Interrupts configuration
#define CHANNEL1 36
#define CHANNEL2 35
#define CHANNEL3 34
#define CHANNEL4 33

// Stepper pin configurations
#define STEERDPIN 24
#define STEERPPIN 25
#define ACCELDPIN 26
#define ACCELPPIN 27
#define BRAKEDPIN 28
#define BRAKEPPIN 29

// Removes Jitter in the steps, this is the range where the motor will ignore step commands
#define JITTERZONE 3

// Software limited steps for the steppers
#define MINSTEER -5000
#define MAXSTEER 5000
#define MINACCEL 0
#define MAXACCEL 1200
#define MINBRAKE -5000
#define MAXBRAKE 0

// Encoder pin definitions for CSn
#define ENCODER 15

// Final sensor values to be used by the program
long sensorValueS;
long sensorValueB;
long sensorValueA;

// Middle step for doing conversion from percentage to steps
long stepS;
long stepB;
long stepA;

// Variables for implementing a deadzone to avoid jitter
long sensorValueALast = 0;
long sensorValueBLast = 0;
long sensorValueSLast = 0;

long stepALast = 0;
long stepBLast = 0;
long stepSLast = 0;

#ifdef ROSMODE
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;

void driveMessage(const ackermann_msgs::AckermannDriveStamped &drive_msg)
{
  Drive = drive_msg.drive;
  sensorValueA = Drive.acceleration.data;
  sensorValueB = Drive.jerk.data;
  sensorValueS = Drive.steering_angle.data;
}

ackermann_msgs::AckermannDrive Drive;
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> sub("arduino_cmd_topic", &driveMessage);
#endif

#ifndef STEPMODE
#include <PID_v1.h>
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
#endif

// Setting up steppers
AccelStepper steering(1, STEERPPIN, STEERDPIN);
AccelStepper accelerator(1, ACCELPPIN, ACCELDPIN);
AccelStepper brakes(1, BRAKEPPIN, BRAKEDPIN);

// Filters out changes faster that 0.5 Hz.
float filterFrequency = 0.5;

// Create a one pole (RC) lowpass filter
FilterOnePole steerFilter(LOWPASS, filterFrequency, 0.0);
FilterOnePole accelFilter(LOWPASS, filterFrequency);
FilterOnePole brakeFilter(LOWPASS, filterFrequency);

#ifndef ROSMODE
// Remote control variable setups
volatile int throttle = 0;
volatile int prev_throttle = 0;
volatile int roll = 0;
volatile int prev_roll = 0;
volatile int pitch = 0;
volatile int prev_pitch = 0;
volatile int yaw = 0;
volatile int prev_yaw = 0;

int throttleTop = 0;
int throttleBtm = 0;
int rollTop = 0;
int rollBtm = 0;
int pitchTop = 0;
int pitchBtm = 0;
int yawTop = 0;
int yawBtm = 0;

int throttleOut = 0;
int rollOut = 0;
int pitchOut = 0;
int yawOut = 0;
#endif

// Encoder code
AS5048A angleSensor(ENCODER);
uint16_t zero_position = 15037;
uint16_t zero_position_map;
float steerAngle = 0;

/**
 * Basic angle procedure:
 *
 * - Obtain register readings (zero position and current position)
 * - Transform to angles with read2angle() 
 * - Calculate the difference between them
 * - normalize() the result
 *
 * NOTE:
 * To see the angles between 0° and 360° comment the following
 * #define, instead, to see the angles between -180° and 180°,
 * uncomment it.
 *
 */

#define ANGLE_MODE_1 // Between -180° and 180°

// Function forward declaration
void motorInit();
void remoteInit();
void encoderInit();
void encoderDebug();
#ifndef ROSMODE
void remoteCalc();
void remoteDebug();
void remoteCalibration();
void remoteVal();
void rising1();
void falling1();
void rising2();
void falling2();
void rising3();
void falling3();
void rising4();
void falling4();
#endif
float read2angle(uint16_t);
float normalise(float);

float encoderCalc();

// CAN Bus handler object
TeensyCANSerializer tCANSerializer;

// CAN bus data objects
NV11DataUltrasonic dataUltrasonic;
NV11DataSpeedo dataSpeedo;

void setup()
{

  delay(3000);
  motorInit();
  Serial.println("Start");
#ifndef ROSMODE
  Serial.begin(115200);
  remoteInit();
  yawOut = 0;
#endif
  SPI.setSCK(14);
  encoderInit();
  pinMode(STEERDPIN, OUTPUT);
  pinMode(STEERPPIN, OUTPUT);
  pinMode(BRAKEDPIN, OUTPUT);
  pinMode(BRAKEPPIN, OUTPUT);
  pinMode(ACCELDPIN, OUTPUT);
  pinMode(ACCELPPIN, OUTPUT);
  steering.setCurrentPosition(steering.currentPosition());
  // Init for CAN bus
  tCANSerializer.init(&dataUltrasonic, &dataSpeedo);

  // Don't know if need to do this
  // pinMode(CANTX, OUTPUT);
  // pinMode(CANRX, OUTPUT);
  // digitalWrite(CANTX, HIGH);
  // digitalWrite(CANRX, HIGH);

#ifdef ROSMODE
  nh.initNode();
  nh.subscribe(sub);
#endif

#ifndef STEPMODE
  myPID.SetMode(AUTOMATIC);
#endif
}

void loop()
{
#ifndef ROSMODE
  remoteCalc();
  // remoteDebug();
  sensorValueS = steerFilter.input(yawOut);
  sensorValueA = accelFilter.input(throttleOut);
  sensorValueB = brakeFilter.input(pitchOut);
#endif

  steerAngle = encoderCalc();

#ifdef ROSMODE
  nh.spinOnce();
#endif

  // Enter formula for conversion from float parameters to steps here
  stepS = sensorValueS;
  stepA = sensorValueA;
  stepB = sensorValueB;

  int percentageAccel = map(sensorValueA, MINACCEL, MAXACCEL, 0, 100);
  int percentageBrake = map(sensorValueB, MINBRAKE, MAXBRAKE, -100, 0);
#ifdef STEPMODE
  if (stepS > stepSLast + JITTERZONE || stepS < stepSLast - JITTERZONE)
  {
    steering.moveTo(stepS);
    stepSLast = stepS;
  }
#endif

  if (stepA > stepALast + JITTERZONE || stepA < stepALast - JITTERZONE)
  {
    accelerator.moveTo(stepA);
    stepALast = stepA;
  }
  if (stepB > stepBLast + JITTERZONE || stepB < stepBLast - JITTERZONE)
  {
    brakes.moveTo(stepB);
    stepBLast = stepB;
  }

#ifdef STEPMODE
  steering.run();
#endif

  accelerator.run();
  brakes.run();

#ifndef STEPMODE
  //PID CODE HERE
  Input = steerAngle;
  Setpoint = sensorValueS;
  myPID.Compute();

  steering.setSpeed(Output);

  steering.runSpeed();
#endif

  // encoderDebug();
  // Serial.print(sensorValueS);
  // Serial.print("\t");
  // Serial.print(steering.currentPosition());
  // Serial.print("\t");
  Serial.print(percentageAccel);
  Serial.print("\t");
  Serial.println(percentageBrake);
  // Serial.println(accelerator.currentPosition());
  // Serial.println(sensorValueB);
}

// Initialisation and setup for steppers
// Tune the parameters here
void motorInit()
{
  steering.setMaxSpeed(10000.0);
  steering.setAcceleration(20000.0);
  steering.setPinsInverted(true, false, false);
  steering.setMinPulseWidth(20);
  brakes.setMaxSpeed(100000.0);
  brakes.setAcceleration(200000.0);
  brakes.setMinPulseWidth(20);
  accelerator.setMaxSpeed(15000000.0);
  accelerator.setAcceleration(15000000.0);
  accelerator.setMinPulseWidth(20);
}

void encoderInit()
{
  angleSensor.init();
  // zero_position = angleSensor.getRawRotation();
  zero_position_map = read2angle(zero_position);
}

float inline read2angle(uint16_t angle)
{
  /*
   * 14 bits = 2^(14) - 1 = 16.383
   *
   * https://www.arduino.cc/en/Reference/Map
   *
   */
  return angle * ((float)360 / 16383);
}

float normalize(float angle)
{
// http://stackoverflow.com/a/11498248/3167294
#ifdef ANGLE_MODE_1
  angle += 180;
#endif
  angle = fmod(angle, 360);
  if (angle < 0)
  {
    angle += 360;
  }
#ifdef ANGLE_MODE_1
  angle -= 180;
#endif
  return angle;
}

float encoderCalc()
{
  uint16_t current_angle = angleSensor.getRawRotation();
  float current_angle_map = read2angle(current_angle);

  float angle = current_angle_map - zero_position_map;
  angle = normalize(angle);
  if (angleSensor.error())
  {
    Serial.println(angleSensor.getErrors());
    return 999;
  }
  else
  {
    return angle;
  }
}

void encoderDebug()
{
  Serial.println(steerAngle);
}

#ifndef ROSMODE

// Initialisation and setup of remote control
void remoteInit()
{
  pinMode(CHANNEL1, INPUT);
  pinMode(CHANNEL2, INPUT);
  pinMode(CHANNEL3, INPUT);
  pinMode(CHANNEL4, INPUT);
  attachInterrupt(digitalPinToInterrupt(CHANNEL1), rising1, RISING);
  attachInterrupt(digitalPinToInterrupt(CHANNEL2), rising2, RISING);
  attachInterrupt(digitalPinToInterrupt(CHANNEL3), rising3, RISING);
  attachInterrupt(digitalPinToInterrupt(CHANNEL4), rising4, RISING);
  // remoteCalibration();
  remoteVal();
}

// calibration program for remote control
void remoteCalibration()
{
  delay(500);
  throttleTop = throttleBtm = throttle;
  rollTop = rollBtm = roll;
  pitchTop = pitchBtm = pitch;
  yawTop = yawBtm = yaw;

  Serial.println("Calibrating... Move the sticks around...");
  Serial.println("Press any key to continue...");
  while (!Serial.available())
  {
    if (throttle > throttleTop)
    {
      throttleTop = throttle;
    }
    if (throttle < throttleBtm)
    {
      throttleBtm = throttle;
    }
    if (roll > rollTop)
    {
      rollTop = roll;
    }
    if (roll < rollBtm)
    {
      rollBtm = roll;
    }
    if (pitch > pitchTop)
    {
      pitchTop = pitch;
    }
    if (pitch < pitchBtm)
    {
      pitchBtm = pitch;
    }
    if (yaw > yawTop)
    {
      yawTop = yaw;
    }
    if (yaw < yawBtm)
    {
      yawBtm = yaw;
    }
    // Serial.println(yaw);
  }
  while (Serial.available())
  {
    Serial.read();
  }
}

void remoteVal()
{
  throttleBtm = 983;
  throttleTop = 2007;
  rollBtm = 983;
  rollTop = 2005;
  pitchBtm = 987;
  pitchTop = 2012;
  yawBtm = 985;
  yawTop = 2010;
}

// Remaps raw remote control values to useable ones
void remoteCalc()
{

  throttleOut = map(throttle, throttleBtm, throttleTop, MINACCEL, MAXACCEL);
  rollOut = map(roll, rollBtm, rollTop, -100, 100);
  pitchOut = map(pitch, pitchBtm, pitchTop, MINBRAKE, MAXBRAKE);
#ifdef STEPMODE
  yawOut = map(yaw, yawBtm, yawTop, MINSTEER, MAXSTEER);
#endif

#ifndef STEPMODE
  yawOut = map(yaw, yawBtm, yawTop, -180, 180);
#endif

  // Serial.print(throttleBtm);
  // Serial.print("\t");
  // Serial.print(throttleTop);
  // Serial.print("\t");
  // Serial.print(rollBtm);
  // Serial.print("\t");
  // Serial.print(rollTop);
  // Serial.print("\t");
  // Serial.print(pitchBtm);
  // Serial.print("\t");
  // Serial.print(pitchTop);
  // Serial.print("\t");
  // Serial.print(yawBtm);
  // Serial.print("\t");
  // Serial.println(yawTop);
}

// Debug program for printing all remote control values
void remoteDebug()
{
  Serial.print(throttleOut);
  Serial.print("\t | \t");
  Serial.print(rollOut);
  Serial.print("\t | \t");
  Serial.print(pitchOut);
  Serial.print("\t | \t");
  Serial.println(yawOut);
}

// Remote Interrupt Service Routines
void rising1()
{
  attachInterrupt(digitalPinToInterrupt(CHANNEL1), falling1, FALLING);
  prev_throttle = micros();
}

void falling1()
{
  attachInterrupt(digitalPinToInterrupt(CHANNEL1), rising1, RISING);
  throttle = micros() - prev_throttle;
}

void rising2()
{
  attachInterrupt(digitalPinToInterrupt(CHANNEL2), falling2, FALLING);
  prev_roll = micros();
}

void falling2()
{
  attachInterrupt(digitalPinToInterrupt(CHANNEL2), rising2, RISING);
  roll = micros() - prev_roll;
}

void rising3()
{
  attachInterrupt(digitalPinToInterrupt(CHANNEL3), falling3, FALLING);
  prev_pitch = micros();
}

void falling3()
{
  attachInterrupt(digitalPinToInterrupt(CHANNEL3), rising3, RISING);
  pitch = micros() - prev_pitch;
}

void rising4()
{
  attachInterrupt(digitalPinToInterrupt(CHANNEL4), falling4, FALLING);
  prev_yaw = micros();
}

void falling4()
{
  attachInterrupt(digitalPinToInterrupt(CHANNEL4), rising4, RISING);
  yaw = micros() - prev_yaw;
}

#endif
