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
}

Mission::Mission(float cruiseVelocity)
    : currentState(State::Waiting),
      endTime(0.0f),
      cruiseVelocity(cruiseVelocity),
      startingDistance(0.0f),
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

void Mission::begin(
    float currentTime,
    float traveledDistance)
{
  currentReference = {0.0f, 0.0f};
  endTime = currentTime + 0.020f;
  startingDistance = traveledDistance;
  currentState = State::StartDelay;
}


bool Mission::update(float currentTime, float traveledDistance)
{
  switch (currentState)
  {
    case State::Waiting:
      break;

    case State::StartDelay:
      if (currentTime > endTime)
      {
        currentState = State::Driving;
      }
      break;

    case State::Driving:
    {
      const float pathDistance = traveledDistance - startingDistance;
      const float turnEndDistance =
          RobotConfig::missionDistance + turnDistance;
      const float routeEndDistance =
          turnEndDistance + RobotConfig::missionFinalStraightDistance;

      if (pathDistance < RobotConfig::missionDistance)
      {
        currentReference = {cruiseVelocity, 0.0f};
      }
      else if (pathDistance < turnEndDistance)
      {
        const float turnProgress =
            (pathDistance - RobotConfig::missionDistance) / turnDistance;
        const float smoothProgressRate =
            6.0f * turnProgress * (1.0f - turnProgress);
        currentReference = {
            RobotConfig::missionTurnVelocity,
            RobotConfig::missionTurnVelocity * pi * smoothProgressRate /
                turnDistance};
      }
      else if (pathDistance < routeEndDistance)
      {
        const float remainingDistance = routeEndDistance - pathDistance;
        const float stoppingSpeed = sqrtf(
            2.0f * RobotConfig::maximumLinearAcceleration * remainingDistance);
        currentReference = {
            fminf(cruiseVelocity, stoppingSpeed),
            0.0f};
      }
      else
      {
        currentReference = {0.0f, 0.0f};
        endTime = currentTime + 1.0f;
        currentState = State::Finishing;
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

float Mission::speedReference() const
{
  return currentReference.forwardSpeed;
}

float Mission::turnRateReference() const
{
  return currentReference.turnRate;
}
