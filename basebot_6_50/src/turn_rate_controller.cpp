#include "turn_rate_controller.h"

TurnRateController::TurnRateController(float Kp, float Ki, float sampleTime)
    : Kp(Kp),
      Ki(Ki),
      errorIntegrator(sampleTime)
{
}

float TurnRateController::update(float reference, float measuredTurnRate)
{
  const float error = reference - measuredTurnRate;
  return Kp * error + Ki * errorIntegrator.update(error);
}

void TurnRateController::reset()
{
  errorIntegrator.reset();
}