#ifndef TURN_RATE_CONTROLLER_H
#define TURN_RATE_CONTROLLER_H

#include "controller_base.h"
#include "integrator.h"

class TurnRateController : public Controller
{
public:
  TurnRateController(float proportionalGain, float integralTime, float sampleTime);

  float update(float desiredTurnRate, float measuredTurnRate);
  void reset() override;

private:
  float proportionalGain;
  float integralGain;
  Integrator errorIntegrator;
};

#endif
