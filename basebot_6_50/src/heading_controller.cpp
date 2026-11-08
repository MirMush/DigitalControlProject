#include "heading_controller.h"

HeadingController::HeadingController(float Kc, float tau_z, float tau_p, float sampleTime)
    : Kc(Kc),
      tau_z(tau_z),
      tau_p(tau_p),
      sampleTime(sampleTime)
{
}

float HeadingController::update(float reference, float measuredHeading)
{
  const float error = reference - measuredHeading;
  return (K1*previousOutput + K2*previousError + K3*error) / K4;
}

void HeadingController::reset()
{
  previousError = 0.0;
  previousOutput = 0.0;
  K1 = 2*tau_p - sampleTime;
  K2 = Kc*sampleTime - 2*tau_z;
  K3 = sampleTime*(2*tau_z + Kc);
  K4 = 2*tau_p + sampleTime;
}