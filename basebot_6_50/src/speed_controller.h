#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "controller_base.h"
#include "integrator.h"
#include "motion_reference.h"

struct MotorVoltage
{
  float left;
  float right;
};

class SpeedController : public Controller
{
public:
  SpeedController(
      float gearRatio,
      float wheelRadius,
      float wheelBase,
      float proportionalGain,
      float integralTime,
      float sampleTime);

  MotorVoltage update(
      const MotionReference &reference,
      float measuredLeftAngularSpeed,
      float measuredRightAngularSpeed);
  void reset() override;

private:
  float motorRadiansPerMeter;
  float wheelBase;
  float proportionalGain;
  float integralGain;
  Integrator leftErrorIntegrator;
  Integrator rightErrorIntegrator;
};

#endif
