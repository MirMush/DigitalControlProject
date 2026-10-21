#include "turn_rate_controller.h"

TurnRateController::TurnRateController(
    float proportionalGain,
    float integralTime,
    float maximumTurnRate,
    float sampleTime)
    : proportionalGain(proportionalGain),
      integralGain(proportionalGain / integralTime),
      maximumTurnRate(maximumTurnRate),
      errorIntegrator(sampleTime)
{
}

float TurnRateController::update(float desiredTurnRate, float measuredTurnRate)
{
  const float error = desiredTurnRate - measuredTurnRate;
  const float candidateIntegral = errorIntegrator.preview(error);
  float output = proportionalGain * error + integralGain * candidateIntegral;

  const bool acceptIntegral =
      !((output > maximumTurnRate && error > 0.0f) ||
        (output < -maximumTurnRate && error < 0.0f));

  if (acceptIntegral)
    errorIntegrator.update(error);
  else
    output = proportionalGain * error + integralGain * errorIntegrator.value();

  if (output > maximumTurnRate)
    output = maximumTurnRate;
  else if (output < -maximumTurnRate)
    output = -maximumTurnRate;

  return output;
}

void TurnRateController::reset()
{
  errorIntegrator.reset();
}
