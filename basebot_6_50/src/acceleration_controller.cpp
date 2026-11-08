#include "acceleration_controller.h"

AccelerationController::AccelerationController(
    float Kp,
    float maxAcceleration,
    float sampleTime)
    : Kp(Kp),
      maxAcceleration(maxAcceleration),
      sampleTime(sampleTime),
      limitedVelocity(0.0f)
{
}

float AccelerationController::update(float reference)
{
  const float error = reference - limitedVelocity;
  float acceleration = Kp * error;

  if (acceleration > maxAcceleration)
    acceleration = maxAcceleration;
  else if (acceleration < -maxAcceleration)
    acceleration = -maxAcceleration;

  const float nextVelocity = limitedVelocity + acceleration * sampleTime;
  if (error > 0.0f && nextVelocity > reference)
    limitedVelocity = reference;
  else if (error < 0.0f && nextVelocity < reference)
    limitedVelocity = reference;
  else
    limitedVelocity = nextVelocity;

  return limitedVelocity;
}

void AccelerationController::reset(float initialVelocity)
{
  limitedVelocity = initialVelocity;
}