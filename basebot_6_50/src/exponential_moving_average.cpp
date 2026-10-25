#include "exponential_moving_average.h"

ExponentialMovingAverage::ExponentialMovingAverage(float alpha, bool enabled)
    : alpha(alpha),
      enabled(enabled)
{
}

float ExponentialMovingAverage::update(float rawValue)
{
  if (!enabled || !initialized)
    filteredValue = rawValue;
  else
    filteredValue += alpha * (rawValue - filteredValue);

  initialized = true;
  return filteredValue;
}

void ExponentialMovingAverage::reset(float value)
{
  filteredValue = value;
  initialized = true;
}

float ExponentialMovingAverage::value() const
{
  return filteredValue;
}