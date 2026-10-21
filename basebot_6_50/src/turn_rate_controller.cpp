#include "turn_rate_controller.h"

TurnRateController::TurnRateController(
    float proportionalGain,
    float integralTime,
    float sampleTime)
    : proportionalGain(proportionalGain),
      integralGain(proportionalGain / integralTime),
      errorIntegrator(sampleTime)
{
}

float TurnRateController::update(float desiredTurnRate, float measuredTurnRate)
{
  const float error = desiredTurnRate - measuredTurnRate;
  const float integral = errorIntegrator.update(error);
  return proportionalGain * error + integralGain * integral;
}

void TurnRateController::reset()
{
  errorIntegrator.reset();
}
