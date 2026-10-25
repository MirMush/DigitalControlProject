#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "integrator.h"

struct MotorVoltage
{
  float left;
  float right;
};

class SpeedController
{
public:
  SpeedController(
      float gearRatio,
      float wheelRadius,
      float Kp,
      float Ki,
      float sampleTime);

  MotorVoltage update(
      float desiredLeftSpeed,
      float desiredRightSpeed,
      float measuredLeftAngularSpeed,
      float measuredRightAngularSpeed);

  void reset();

private:
  float motorRadiansPerMeter;
  float Kp;
  float Ki;
  Integrator leftErrorIntegrator;
  Integrator rightErrorIntegrator;
};

#endif
