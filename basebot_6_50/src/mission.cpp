#include "mission.h"

#include <math.h>

Mission::Mission(float cruiseVelocity, float acceleration)
    : currentState(0),
      endTime(0.0f),
      profileStartTime(0.0f),
      targetDistance(0.0f),
      startDistance(0.0f),
      cruiseVelocity(cruiseVelocity),
      acceleration(acceleration),
      reference{0.0f, 0.0f}
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
  reference = {0.0f, 0.0f};
  endTime = currentTime + 0.020f;
  currentState = 10;
}

float Mission::profileVelocity(float elapsedTime, float remainingDistance) const
{
  float velocity = acceleration * elapsedTime;
  if (velocity > cruiseVelocity)
    velocity = cruiseVelocity;

  const float brakingDistance = remainingDistance > 0.0f ? remainingDistance : 0.0f;
  const float brakingLimit = sqrtf(2.0f * acceleration * brakingDistance);
  if (velocity > brakingLimit)
    velocity = brakingLimit;

  return velocity;
}

bool Mission::updateStraight(float currentTime, float traveledDistance)
{
  switch (currentState)
  {
    case 10:
      if (currentTime > endTime)
      {
        profileStartTime = currentTime;
        endTime = currentTime + targetDistance / cruiseVelocity + 3.0f;
        currentState = 11;
      }
      break;

    case 11:
    {
      const float measuredProgress = traveledDistance - startDistance;
      const float progress = measuredProgress > 0.0f ? measuredProgress : 0.0f;
      const float remainingDistance = targetDistance - progress;
      if (progress >= targetDistance || currentTime > endTime)
      {
        reference = {0.0f, 0.0f};
        endTime = currentTime + 0.3f;
        currentState = 12;
      }
      else
      {
        const float elapsedTime = currentTime - profileStartTime;
        reference.linearVelocity = profileVelocity(elapsedTime, remainingDistance);
        reference.angularVelocity = 0.0f;
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
  reference = {0.0f, 0.0f};
  endTime = currentTime + delay;
  currentState = 2;
}

int Mission::state() const
{
  return currentState;
}

MotionReference Mission::motionReference() const
{
  return reference;
}