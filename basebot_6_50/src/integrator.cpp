#include "integrator.h"

Integrator::Integrator(float sampleTime, IntegrationMethod method)
    : sampleTime(sampleTime),
      previousRate(0.0f),
      integral(0.0f),
      method(method)
{
}

float Integrator::preview(float rate) const
{
  switch (method)
  {
    case IntegrationMethod::ForwardEuler:
      return integral + rate * sampleTime;
    case IntegrationMethod::Tustin:
      return integral + (rate + previousRate) * sampleTime / 2.0f;
  }
  return integral;
}

float Integrator::update(float rate)
{
  integral = preview(rate);
  previousRate = rate;
  return integral;
}

float Integrator::value() const
{
  return integral;
}

void Integrator::reset()
{
  previousRate = 0.0f;
  integral = 0.0f;
}
