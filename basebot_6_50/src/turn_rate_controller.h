#ifndef TURN_RATE_CONTROLLER_H
#define TURN_RATE_CONTROLLER_H

#include "integrator.h"

class TurnRateController
{
public:
  TurnRateController(float Kp, float Ki, float sampleTime);

  float update(float reference, float measuredTurnRate);
  void reset();

private:
  float Kp;
  float Ki;
  Integrator errorIntegrator;
};

#endif