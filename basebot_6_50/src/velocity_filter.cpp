#include "velocity_filter.h"

VelocityFilter::VelocityFilter(float alpha, bool enabled)
    : alpha(alpha),
      enabled(enabled)
{
}

float VelocityFilter::update(float rawValue)
{
  if (!enabled || !initialized)
    filteredValue = rawValue;
  else
    filteredValue += alpha * (rawValue - filteredValue);

  initialized = true;
  return filteredValue;
}

void VelocityFilter::reset(float value)
{
  filteredValue = value;
  initialized = true;
}

float VelocityFilter::value() const
{
  return filteredValue;
}