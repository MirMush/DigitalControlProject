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
      float proportionalGain);

  MotorVoltage update(
      float desiredSpeed,
      float measuredLeftAngularSpeed,
      float measuredRightAngularSpeed);

private:
  float motorRadiansPerMeter;
  float proportionalGain;
};

#endif
