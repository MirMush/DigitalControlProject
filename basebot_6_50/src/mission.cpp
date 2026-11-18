#include "mission.h"

#include "robot_config.h"

#include <math.h>

namespace
{
constexpr float pi = 3.14159265358979323846f;
// Integral of sin(pi * smoothstep(u)) over u in [0, 1].
constexpr float turnOffsetIntegral = 0.486076f;
constexpr float turnDistance =
    RobotConfig::missionLaneOffset / turnOffsetIntegral;
constexpr float endOfTurnDistance =
    RobotConfig::missionDistance + turnDistance;
constexpr float routeEndDistance =
    endOfTurnDistance + RobotConfig::missionFinalStraightDistance;
}

Mission::Mission(float cruiseVelocity)
    : currentState(State::Waiting),
      endTime(0.0f),
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

void Mission::begin(float currentTime)
{
  targetDistance = distance;
  startDistance = traveledDistance;
  reference = 0.0f;
  endTime = currentTime + 0.020f;
  startingDistance = traveledDistance;
  startingHeading = initialHeading;
  currentState = State::StartDelay;
}


bool Mission::update(float currentTime)
{
  switch (currentState)
  {
    case State::Waiting:
      break;

    case State::StartDelay:
      if (currentTime > endTime)
      {
        endTime = currentTime + 12.0f;
        currentState = State::Driving;
      }
      break;

    case State::Driving:
    {
      if (currentTime > 1.0f)
        currentReference = {0.5f, 0.0f};
      
      if (currentTime > 3.0f)
        currentReference.heading += 3.1415f / 2;

      if (currentTime > 5.0f)
        currentReference.heading += 3.1415f / 2;
      
      if (currentTime > 7.0f)
        currentReference.heading += 3.1415f / 2;
      
      if (currentTime > 9.0f)
        currentReference.heading += 3.1415f / 2;

      if (currentTime > endTime)
      {
        endTime = currentTime + 0.2f;
        currentState = State::Finishing;
      }
      else
      {
        reference = cruiseVelocity;
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
  reference = 0.0f;
  endTime = currentTime + delay;
  currentState = State::Stopping;
}

Mission::State Mission::state() const
{
  return currentState;
}

float Mission::speedReference() const
{
  return reference;
}