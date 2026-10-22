#include "mission.h"

Mission::Mission(float cruiseVelocity)
    : currentState(0),
      endTime(0.0f),
      targetDistance(0.0f),
      startDistance(0.0f),
      cruiseVelocity(cruiseVelocity),
      reference(0.0f)
{
}

bool Mission::waitingForStart() const
{
  return currentState == 0;
}

void Mission::startStraight(float distance, float currentTime, float traveledDistance)
{
  targetDistance = distance;
  startDistance = traveledDistance;
  reference = 0.0f;
  endTime = currentTime + 0.020f;
  currentState = 10;
}

bool Mission::updateStraight(float currentTime, float traveledDistance)
{
  switch (currentState)
  {
    case 10:
      if (currentTime > endTime)
      {
        endTime = currentTime + targetDistance / cruiseVelocity + 3.0f;
        currentState = 11;
      }
      break;

    case 11:
    {
      const float measuredProgress = traveledDistance - startDistance;
      const float progress = measuredProgress > 0.0f ? measuredProgress : 0.0f;
      if (progress >= targetDistance || currentTime > endTime)
      {
        reference = 0.0f;
        endTime = currentTime + 0.3f;
        currentState = 12;
      }
      else
      {
        reference = cruiseVelocity;
      }
      break;
    }

    case 12:
      if (currentTime > endTime)
      {
        currentState = 0;
        return true;
      }
      break;

    case 2:
      if (currentTime > endTime)
      {
        currentState = 0;
        return true;
      }
      break;
  }

  return false;
}

void Mission::stop(float currentTime, float delay)
{
  reference = 0.0f;
  endTime = currentTime + delay;
  currentState = 2;
}

int Mission::state() const
{
  return currentState;
}

float Mission::speedReference() const
{
  return reference;
}