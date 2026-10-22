#include "speed_controller.h"

SpeedController::SpeedController(
    float gearRatio,
    float wheelRadius,
    float proportionalGain)
    : motorRadiansPerMeter(gearRatio / wheelRadius),
      proportionalGain(proportionalGain)
{
}

MotorVoltage SpeedController::update(
    float desiredSpeed,
    float measuredLeftAngularSpeed,
    float measuredRightAngularSpeed)
{
  const float referenceMotorSpeed = motorRadiansPerMeter * desiredSpeed;

  const float leftError = referenceMotorSpeed - measuredLeftAngularSpeed;
  const float rightError = referenceMotorSpeed - measuredRightAngularSpeed;

  MotorVoltage output = {
      proportionalGain * leftError,
      proportionalGain * rightError};

  if (output.left < -9.0f)
    output.left = -9.0f;
  else if (output.left > 9.0f)
    output.left = 9.0f;

  if (output.right < -9.0f)
    output.right = -9.0f;
  else if (output.right > 9.0f)
    output.right = 9.0f;

  return output;
}