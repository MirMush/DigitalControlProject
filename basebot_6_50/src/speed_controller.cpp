#include "speed_controller.h"

SpeedController::SpeedController(
    float gearRatio,
    float wheelRadius,
    float wheelBase,
    float proportionalGain,
    float integralTime,
    float sampleTime)
    : motorRadiansPerMeter(gearRatio / wheelRadius),
      wheelBase(wheelBase),
      proportionalGain(proportionalGain),
      integralGain(proportionalGain / integralTime),
      leftErrorIntegrator(sampleTime),
      rightErrorIntegrator(sampleTime)
{
}

MotorVoltage SpeedController::update(
    const MotionReference &reference,
    float measuredLeftAngularSpeed,
    float measuredRightAngularSpeed)
{
  const float halfTrackAngularSpeed = reference.angularVelocity * wheelBase / 2.0f;
  const float referenceLeftAngularSpeed =
      motorRadiansPerMeter * (reference.linearVelocity - halfTrackAngularSpeed);
  const float referenceRightAngularSpeed =
      motorRadiansPerMeter * (reference.linearVelocity + halfTrackAngularSpeed);

  const float leftError = referenceLeftAngularSpeed - measuredLeftAngularSpeed;
  const float rightError = referenceRightAngularSpeed - measuredRightAngularSpeed;
  const float candidateLeftIntegral = leftErrorIntegrator.preview(leftError);
  const float candidateRightIntegral = rightErrorIntegrator.preview(rightError);

  MotorVoltage output = {
      proportionalGain * leftError + integralGain * candidateLeftIntegral,
      proportionalGain * rightError + integralGain * candidateRightIntegral};

  const bool acceptLeftIntegral =
      !((output.left > 9.0f && leftError > 0.0f) ||
        (output.left < -9.0f && leftError < 0.0f));
  const bool acceptRightIntegral =
      !((output.right > 9.0f && rightError > 0.0f) ||
        (output.right < -9.0f && rightError < 0.0f));

  if (acceptLeftIntegral)
    leftErrorIntegrator.update(leftError);
  else
    output.left = proportionalGain * leftError + integralGain * leftErrorIntegrator.value();

  if (acceptRightIntegral)
    rightErrorIntegrator.update(rightError);
  else
    output.right = proportionalGain * rightError + integralGain * rightErrorIntegrator.value();

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

void SpeedController::reset()
{
  leftErrorIntegrator.reset();
  rightErrorIntegrator.reset();
}