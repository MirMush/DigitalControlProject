#include "speed_controller.h"

SpeedController::SpeedController(
    float gearRatio,
    float wheelRadius,
    float wheelBase,
    float proportionalGain,
    float turnRateProportionalGain)
    : motorRadiansPerMeter(gearRatio / wheelRadius),
      halfWheelBase(wheelBase / 2.0f),
      proportionalGain(proportionalGain),
      turnRateProportionalGain(turnRateProportionalGain)
{
}

MotorVoltage SpeedController::update(
    float desiredSpeed,
    float desiredTurnRate,
    float measuredLeftAngularSpeed,
    float measuredRightAngularSpeed,
    float measuredTurnRate)
{
  const float turnRateError = desiredTurnRate - measuredTurnRate;
  const float correctedTurnRate =
      desiredTurnRate + turnRateProportionalGain * turnRateError;
  const float leftWheelSpeed = desiredSpeed - halfWheelBase * correctedTurnRate;
  const float rightWheelSpeed = desiredSpeed + halfWheelBase * correctedTurnRate;

  const float leftReferenceMotorSpeed = motorRadiansPerMeter * leftWheelSpeed;
  const float rightReferenceMotorSpeed = motorRadiansPerMeter * rightWheelSpeed;

  const float leftError = leftReferenceMotorSpeed - measuredLeftAngularSpeed;
  const float rightError = rightReferenceMotorSpeed - measuredRightAngularSpeed;

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