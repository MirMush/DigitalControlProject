/***************************************************************************
*   Copyright (C) 2024 by DTU                             *
*   jcan@dtu.dk                                                    *
*
*   Base Teensy firmware
*   build for Teensy 4.1,
*   intended for digital control course
* 
* The MIT License (MIT)  https://mit-license.org/
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the “Software”), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
* is furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies 
* or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
* THE SOFTWARE. */

void printLog();

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <IntervalTimer.h>
// Support functions
#include "src/udisplay.h"
#include "src/urobot.h"
#include "src/uusb.h"
#include "src/umotor.h"
#include "src/uimu2.h"
#include "src/uencoder.h"

/**
 * Global variables */
// Sample time can not go lower than 300us
const uint32_t sampleTimeUs = 1000; // desired sample time in us
const uint32_t logIntervalUs = 10000; // send one log row every 10 ms
const int logSampleDivider = logIntervalUs / sampleTimeUs;
// const float ts = sampleTimeUs * 1e-6; // sample time in seconds
// Robot configuration
const float gear = 9.6;
const float wheelRadius = 0.0315; // (m)
const float wheelBase = 0.15;
// pose values
float distA = 0; // trip distance
float poseX = 0;
float poseY = 0;
float heading = 0;
float forwardVel = 0;
float turnRate = 0;
bool velSaturated = false;

// ////////////////////////////////////////
// define a structure for log elements
typedef struct
{
  float time;
  int state;
  float desiredValue;
  float desiredHeading;
  float motorVel[2];
  float motorVolt[2];
  int32_t encoder[2];
  float motorCurrent[2];
  float battery;
  float poseX;
  float poseY;
  float heading;
  float distA;
  float forwardVel;
  float turnRate;
  float inVelLeft;
  float inVelRight;
  float gyroX;
  float gyroY;
  float gyroZ;
  float headU;
  float accVel;
  float accTurn;
} LogData;
//
// allocate memory for data log in memory block 2 available only through malloc/new
// (specific for this micro processor, a Teensy 4)
const int logsMax = 400000 / sizeof(LogData); // allocates 400kBytes
LogData * logs = (LogData*)malloc(logsMax * sizeof(LogData));
int logsCnt = 0; // next log entry to use / i.e. number of used log entries
int logSampleCounter = 0;

void printLogHeader()
{
  Serial.print("% Basebot log for ");
  Serial.println(robot.getRobotName());
  // hard-coded settings
  Serial.print("% Sample time ");  Serial.print(sampleTimeUs);
  Serial.print(" us, log interval "); Serial.print(logIntervalUs);
  Serial.print(" us, Gear ");  Serial.print(gear);
  Serial.print(", Wheels ");  Serial.print(wheelRadius);
  Serial.print(" m, PPR ");  Serial.println(encoder.pulsPerRev);
  Serial.print("% maximum log samples ");  Serial.print(logsMax);
  Serial.print(" each sized ");  Serial.print(sizeof(LogData)); Serial.println(" bytes");
  // Columns
  Serial.println("% 1 time (sec)");
  Serial.println("% 2 state");
  Serial.println("% 3   Desired value");
  Serial.println("% 4-5 motor voltage (left,right) (V)");
  Serial.println("% 6-7 Encoder count (left, right)");
  Serial.println("% 8-9 motor vel (left, right) (rad/s)");
  Serial.println("% 10  Battery voltage (V)");
  Serial.println("% 11-13 gyro x,y,z (deg/s)");
  Serial.println("% 14-15 motor current (left, right) (A)");
  Serial.println("% 16-17 pose x,y (m)");
  Serial.println("% 18 heading (rad)");
  Serial.println("% 19 distance (m)");
  Serial.println("% 20 forward velocity (m/s)");
  Serial.println("% 21 turn rate (rad/s)");
}

void printLogRow(const LogData &d)
{
  Serial.print(d.time,4);    // 1
  Serial.print(" ");    Serial.print(d.state);          // 2
  Serial.print(" ");    Serial.print(d.desiredValue,2); // 3
  Serial.print(" ");    Serial.print(d.motorVolt[0],2); // 4 (V)
  Serial.print(" ");    Serial.print(d.motorVolt[1],2); // 5 (V)
  Serial.print(" ");    Serial.print(d.encoder[0]);     // 6
  Serial.print(" ");    Serial.print(d.encoder[1]);     // 7
  Serial.print(" ");    Serial.print(d.motorVel[0],2);  // 8 (rad/s)
  Serial.print(" ");    Serial.print(d.motorVel[1],2);  // 9 (rad/s)
  Serial.print(" ");    Serial.print(d.battery,2);      // 10 (V)
  Serial.print(" ");    Serial.print(d.gyroX,2);        // 11 (deg/s)
  Serial.print(" ");    Serial.print(d.gyroY,2);        // 12 (deg/s)
  Serial.print(" ");    Serial.print(d.gyroZ,2);        // 13 (deg/s)
  Serial.print(" ");    Serial.print(d.motorCurrent[0],2); // 14 (A)
  Serial.print(" ");    Serial.print(d.motorCurrent[1],2); // 15 (A)
  Serial.print(" ");    Serial.print(d.poseX,2);         // 16 (m)
  Serial.print(" ");    Serial.print(d.poseY,2);         // 17 (m)
  Serial.print(" ");    Serial.print(d.heading,2);       // 18 (rad)
  Serial.print(" ");    Serial.print(d.distA,2);          // 19 (m)
  Serial.print(" ");    Serial.print(d.forwardVel,2);    // 20 (m/s)
  Serial.print(" ");    Serial.print(d.turnRate,2);      // 21 (rad/s)
  Serial.println();
}

void printLog()
{
  printLogHeader();
  for (int i = 0; i < logsCnt; i++)
    printLogRow(logs[i]);

  Serial.print("% sent ");
  Serial.print(logsCnt);
  Serial.println(" log lines");
}

// ////////////////////////////////////////

/**
 * Global variables for timing */
bool isSampleTime = false;
float time_sec = 0;

// Timer that controls sample time (interrupt based)
IntervalTimer sampleTimer;

void timerInterrupt()
{
  isSampleTime = true;
  time_sec += sampleTimeUs / 1e6;
}
/**
 * declaration, that there is a reset function */
void reset();

/**
 * Global variables for state machine */
int state = 0;               // actual sequence state
float endTime = 0;           // for current state
float desiredValue = 0;      // desired (reference) value send to controller
float desiredHeading = 0;
float integralLeft = 0;
float integralRight = 0;

void start()
{ // Start timing
  // reset log and encoders
  logsCnt = 0;
  logSampleCounter = 0;
  printLogHeader();
  // change motor PWM frequency for sampling time test
  // should not be above 100000 (100kHz), default is 80kHz.
  motor.setPWMfrq(80000);
  time_sec = 0;
  encoder.encoder[0] = 0; // left motor encoder
  encoder.encoder[1] = 0; // right motor encoder
  // Update of display takes too long for fast sampling
  // so, disable during sequence.
  display.useDisplay = false;

  integralLeft = 0;
  integralRight = 0;

  poseX = 0;
  poseY = 0;
  heading = 0;
  distA = 0;
}

void stop(float after)
{ // stop, but allow continued logging for a while
  state = 2;
  desiredValue = 0;
  // end log after an additional short time
  endTime = time_sec + after;
}

void finished()
{ // prepare for a new start
  // stop motors
  motor.motorVoltage[0] = 0; // left motor
  motor.motorVoltage[1] = 0;  // right motor
  // set default PWM frequency
  motor.setPWMfrq(80000);
  // start updating display
  display.useDisplay = true;
  // wait for next button press
  state = 0;
  Serial.print("% sent ");
  Serial.print(logsCnt);
  Serial.println(" log lines");
}

/**
 * Simple 2-step sequence */
void sequenceTwoSteps()
{ // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.
  bool button;
  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence
  switch (state)
  { // run mission, initial value
    case 0: // State 0 is just inactive, waiting for the start signal.
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      { // starting by a short time to get zero velocity data to the log
        // reset for new run
        start();
        // Prepare next state
        desiredValue = 0; // reference value to the controller
        // to get start the log with no velocity
        endTime = time_sec + 0.020; // new state to end after 20ms
        state = 10;
      }
      break;
    case 10: // Waiting for first step (zero velocity to log) to finish.
      if (time_sec > endTime)
      { // change to next values - drive
        desiredValue = 0.25;  // should be in meters/sec, but is motor voltage for now,
        endTime = time_sec + 3.0 ; // ~ 0.5 second
        state = 11;
      }
      break;
    case 11: // First step
      // test if ready for next state
      if (time_sec > endTime)
      { // change to next values
        desiredValue = 0.25;
        endTime = time_sec + 3.0;
        state = 12;
      }
      break;
    case 12: // Second step
      // test if this state is finished
      if (time_sec > endTime)
      { // change to next values
        desiredValue = 0.25;
        endTime = time_sec + 3.0;
        state = 100;
      }
      break;
    case 100: // Maintain last step
      // stop after end time
      if (time_sec > endTime)
      { // stop, but continue logging for a while (0.3 sec)
        stop(0.3); // actually sets state to 999 (i.e. default:)
      }
      break;
    default: // Hold until finished
      // this state is needed to enable a new start
      if (time_sec > endTime)
        finished();
      break;
  }
}

void goStraigthSequence(float distance)
{
  bool button;

  static float targetDistance = 0;

  switch (state)
  {
    case 0: // Wait for start
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      {
        start(); // Resets distA and the other run values
        targetDistance = distance;
        desiredValue = 0;
        endTime = time_sec + 0.020f;
        state = 10;
      }
      break;

    case 10: // Brief pause before driving
      if (time_sec > endTime)
      {
        desiredValue = 0.20f; // Forward speed reference in m/s
        endTime = time_sec + targetDistance / 0.20f + 3.0f;
        state = 11;
      }
      break;

    case 11: // Drive until the requested distance is reached
      if (distA >= targetDistance || time_sec > endTime)
      {
        desiredValue = 0;
        endTime = time_sec + 0.3f; // Keep logging while the robot slows down
        state = 12;
      }
      break;

    case 12: // Finish
      if (time_sec > endTime)
        finished();
      break;
  }
}


void updatePose(float leftWheelVel, float rightWheelVel)
{
  const float dt = sampleTimeUs * 1e-6f;

  forwardVel = (leftWheelVel + rightWheelVel) / 2.0f;
  turnRate = (rightWheelVel - leftWheelVel) / wheelBase;

  const float middleHeading = heading + turnRate * dt / 2.0f;

  poseX += forwardVel * cosf(middleHeading) * dt;
  poseY += forwardVel * sinf(middleHeading) * dt;
  heading += turnRate * dt;
  distA += forwardVel * dt;
}

float integrateError(float error, float &integral, float dt)
{
  integral += error * dt;
  return integral;
}

/**
 * make the control */
void controlUpdate()
{ // do control during a mission only.
  // called at every tick

  float mvleft = -encoder.motorVelocity[0];
  float mvright = encoder.motorVelocity[1];

  float gearWrad = gear / wheelRadius;
  float PGain = 0.1202;

  float velRef = desiredValue;
  //
  // For a start, the desiredValue
  // is just used as motor voltage
  //

  float newValueLeft = PGain* (gearWrad * velRef - mvleft);
  float newValueRight = PGain* (gearWrad * velRef - mvright);

  if (newValueLeft<-9)
    newValueLeft = -9;
  else if (newValueLeft >9)
    newValueLeft = 9;

  if (newValueRight<-9)
    newValueRight = -9;
  else if (newValueRight >9)
    newValueRight = 9;
  // Left motor voltage (-9V to +9V)

  motor.motorVoltage[0] = -newValueLeft;
  // Right motor voltage (-9V to +9V)
  motor.motorVoltage[1] = newValueRight;
  //
  // in the main loop, there is a motor.tick() that actually send the
  // values to the motor.
}

void updateLog()
{ // step response mode
  if (state > 0 and logsCnt < logsMax)
  {
    if (++logSampleCounter < logSampleDivider)
      return;
    logSampleCounter = 0;

    // Save and stream one sample at the selected log interval.
    logs[logsCnt].time = time_sec;
    logs[logsCnt].state = state;
    logs[logsCnt].desiredValue = desiredValue;
    logs[logsCnt].motorVolt[0] = -motor.motorVoltage[0]; // The left motor runs backwards, therefore the sign.
    logs[logsCnt].motorVolt[1] = motor.motorVoltage[1];
    logs[logsCnt].encoder[0] = -encoder.encoder[0];
    logs[logsCnt].encoder[1] = encoder.encoder[1];
    logs[logsCnt].motorVel[0] = -encoder.motorVelocity[0];
    logs[logsCnt].motorVel[1] = encoder.motorVelocity[1];
    logs[logsCnt].battery = robot.batteryVoltage;
    logs[logsCnt].gyroX = imu2.gyro[0];
    logs[logsCnt].gyroY = imu2.gyro[1];
    logs[logsCnt].gyroZ = imu2.gyro[2];
    // the last part is offset of current measurement (in Amps)
    // take the initial current readings and add or subtract as needed
    // these values fits Tania: (+0.260A, -0.275A)
    logs[logsCnt].desiredHeading = desiredHeading;
    logs[logsCnt].motorCurrent[0] = -(analogRead(A1) - 2048) * 0.0045 + 0.260;
    logs[logsCnt].motorCurrent[1] =  (analogRead(A0) - 2048) * 0.0045 - 0.275;
    logs[logsCnt].forwardVel = forwardVel;
    logs[logsCnt].turnRate = turnRate;
    logs[logsCnt].poseX = poseX;
    logs[logsCnt].poseY = poseY;
    logs[logsCnt].heading = heading;
    logs[logsCnt].distA = distA;
    // add more items as needed, NB! also the the struct definition at top.
    printLogRow(logs[logsCnt]);
    logsCnt++;
  }
}


void setup()   // INITIALIZATION
{ // to be able to print to USB interface
  Serial.begin(12000000);
  sampleTimer.begin(timerInterrupt, sampleTimeUs);
  // initialize sensors, safety and display
  robot.setup(); // alive LED, display and battery
  motor.setup(); // motor voltage
  encoder.setup(); // motor encoders to velocity
  encoder.pulsPerRev = 48;
  imu2.setup(); // gyro and accelerometer
  motor.setPWMfrq(0x13000); // 0x13000 = 77824 Hz
  usb.setup();
  display.useDisplay = true;
}

/**
* Main loop
* */
void loop ( void )
{ // init sample time
  while ( true )
  { // main loop
    // loop until time for next sample
    if (isSampleTime) // start of new control cycle
    { // time for next tick
      isSampleTime = false;
      // read sensors
      imu2.tick();
      encoder.tick();
      // updatePose();
      // spikeSequence();
      //sequenceTwoSteps();
      goStraigthSequence(1); // drive 1 meter straight
      //
      if (state > 0)
      { // Only if started
        // Calculate new motor voltage
        updatePose(
            -encoder.motorVelocity[0] * wheelRadius / gear,
             encoder.motorVelocity[1] * wheelRadius / gear);

        controlUpdate();
      }
      // give value to actuators
      motor.tick();
      // save relevant values
      updateLog();
      // support functions
      robot.tick();   // measure battery voltage etc.
      display.tick(); // update O-LED display
    }
    usb.tick(); // listen to incoming from USB
  }
}

/////////////////////////////////////////////////////////////////
