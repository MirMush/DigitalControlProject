#include "mission.h"

Mission::Mission(float cruiseVelocity)
    : currentState(State::Waiting),
      endTime(0.0f),
      targetDistance(0.0f),
      startDistance(0.0f),
      cruiseVelocity(cruiseVelocity),
      currentReference{0.0f, 0.0f}
{
}

bool Mission::waitingForStart() const
{
  return currentState == State::Waiting;
}

bool Mission::isActive() const
{
  return currentState != State::Waiting;
}

void Mission::startStraight(float distance, float currentTime, float traveledDistance)
{
  targetDistance = distance;
  startDistance = traveledDistance;
  currentReference = {0.0f, 0.0f};
  endTime = currentTime + 0.020f;
  currentState = State::StartDelay;
}

bool Mission::updateStraight(float currentTime, float traveledDistance)
{
  switch (currentState)
  {
    case State::Waiting:
      break;

    case State::StartDelay:
      if (currentTime > endTime)
      {
        endTime = currentTime + targetDistance / cruiseVelocity + 3.0f;
        currentState = State::Driving;
      }
      break;

    case State::Driving:
    {
      const float measuredProgress = traveledDistance - startDistance;
      const float progress = measuredProgress > 0.0f ? measuredProgress : 0.0f;
      if (progress >= targetDistance || currentTime > endTime)
      {
        currentReference = {0.0f, 0.0f};
        endTime = currentTime + 0.3f;
        currentState = State::Finishing;
      }
      else
      {
        currentReference = {cruiseVelocity, 0.0f};
      }
      break;
    }

    case State::Finishing:
      if (currentTime > endTime)
      {
        currentState = State::Waiting;
        return true;
      }
      break;

    case State::Stopping:
      if (currentTime > endTime)
      {
        currentState = State::Waiting;
        return true;
      }
      break;
  }

  return false;
}

void Mission::stop(float currentTime, float delay)
{
  currentReference = {0.0f, 0.0f};
  endTime = currentTime + delay;
  currentState = State::Stopping;
}

Mission::State Mission::state() const
{
  return currentState;
}

MotionReference Mission::reference() const
{
  return currentReference;
}