#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

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
      float wheelBase,
      float proportionalGain,
      float turnRateProportionalGain);

  MotorVoltage update(
      float desiredSpeed,
      float desiredTurnRate,
      float measuredLeftAngularSpeed,
      float measuredRightAngularSpeed,
      float measuredTurnRate);

private:
  float motorRadiansPerMeter;
  float halfWheelBase;
  float proportionalGain;
  float turnRateProportionalGain;
};

#endif
