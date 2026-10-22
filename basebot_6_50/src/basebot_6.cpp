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
#include "pose.h"
#include "speed_controller.h"
#include "logger.h"
#include "mission.h"
#include "robot_config.h"

/**
 * Global variables */
// pose values
PoseEstimator pose(RobotConfig::sampleTimeUs * 1e-6f, RobotConfig::wheelBase);
SpeedController speedController(
    RobotConfig::gearRatio,
    RobotConfig::wheelRadius,
    RobotConfig::speedProportionalGain);
Mission mission(RobotConfig::missionCruiseVelocity);
Logger logger(400000, RobotConfig::logSampleDivider);

void printLog()
{
  logger.printAll(
      robot.getRobotName(),
      RobotConfig::sampleTimeUs,
      RobotConfig::logIntervalUs,
      RobotConfig::gearRatio,
      RobotConfig::wheelRadius,
      encoder.pulsPerRev);
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
  time_sec += RobotConfig::sampleTimeUs / 1e6f;
}

void start()
{ // Start timing
  logger.startRun(
      robot.getRobotName(),
      RobotConfig::sampleTimeUs,
      RobotConfig::logIntervalUs,
      RobotConfig::gearRatio,
      RobotConfig::wheelRadius,
      encoder.pulsPerRev);
  // change motor PWM frequency for sampling time test
  // should not be above 100000 (100kHz), default is 80kHz.
  motor.setPWMfrq(80000);
  time_sec = 0;
  encoder.encoder[0] = 0; // left motor encoder
  encoder.encoder[1] = 0; // right motor encoder
  // Update of display takes too long for fast sampling
  // so, disable during sequence.
  display.useDisplay = false;
  pose.reset();
}

void stop(float after)
{
  mission.stop(time_sec, after);
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
  logger.finish();
}

void updateLog()
{
  const int state = mission.state();
  if (!logger.shouldLog(state > 0))
    return;

  LogData sample{};
  sample.time = time_sec;
  sample.state = state;
  sample.desiredValue = mission.speedReference();
  sample.motorVolt[0] = -motor.motorVoltage[0];
  sample.motorVolt[1] = motor.motorVoltage[1];
  sample.encoder[0] = -encoder.encoder[0];
  sample.encoder[1] = encoder.encoder[1];
  sample.motorVel[0] = -encoder.motorVelocity[0];
  sample.motorVel[1] = encoder.motorVelocity[1];
  sample.battery = robot.batteryVoltage;
  sample.gyroX = imu2.gyro[0];
  sample.gyroY = imu2.gyro[1];
  sample.gyroZ = imu2.gyro[2];
  sample.motorCurrent[0] = -(analogRead(A1) - 2048) * 0.0045f + 0.260f;
  sample.motorCurrent[1] = (analogRead(A0) - 2048) * 0.0045f - 0.275f;
  sample.forwardVel = pose.forwardVelocity();
  sample.turnRate = pose.turnRate();
  sample.poseX = pose.x();
  sample.poseY = pose.y();
  sample.heading = pose.heading();
  sample.distA = pose.distance();
  logger.record(sample);
}

void updateMission()
{
  const bool startSignal = digitalReadFast(PIN_START_BUTTON) or robot.missionStart;
  if (mission.waitingForStart() and startSignal)
  {
    start();
    mission.startStraight(RobotConfig::missionDistance, time_sec, pose.distance());
  }
  else if (mission.updateStraight(time_sec, pose.distance()))
  {
    finished();
  }
}

void updatePoseAndSpeedController()
{
  if (mission.state() <= 0)
    return;

  pose.update(
      -encoder.motorVelocity[0] * RobotConfig::wheelRadius / RobotConfig::gearRatio,
       encoder.motorVelocity[1] * RobotConfig::wheelRadius / RobotConfig::gearRatio);

  const MotorVoltage output = speedController.update(
      mission.speedReference(),
      -encoder.motorVelocity[0],
      encoder.motorVelocity[1]);
  motor.motorVoltage[0] = -output.left;
  motor.motorVoltage[1] = output.right;
}

void updateControlCycle()
{
  imu2.tick();
  encoder.tick();
  updateMission();
  updatePoseAndSpeedController();
  motor.tick();
  updateLog();
  robot.tick();
  display.tick();
}

void setup()   // INITIALIZATION
{ // to be able to print to USB interface
  Serial.begin(12000000);
  sampleTimer.begin(timerInterrupt, RobotConfig::sampleTimeUs);
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
  {
    if (isSampleTime)
    {
      isSampleTime = false;
      updateControlCycle();
    }
    usb.tick();
  }
}

/////////////////////////////////////////////////////////////////
