#include "mission.h"

#include <math.h>

namespace
{
constexpr float pi = 3.14159265358979323846f;
}

Mission::Mission(float cruiseVelocity, float acceleration, float circleDiameter)
    : currentState(0),
      endTime(0.0f),
      profileStartTime(0.0f),
      straightDistance(0.0f),
      totalCourseDistance(0.0f),
      startDistance(0.0f),
      cruiseVelocity(cruiseVelocity),
      acceleration(acceleration),
      circleRadius(circleDiameter / 2.0f),
      reference{0.0f, 0.0f}
{
}

bool Mission::waitingForStart() const
{
  return currentState == 0;
}

void Mission::startCourse(float lineDistance, float currentTime, float traveledDistance)
{
  straightDistance = lineDistance;
  totalCourseDistance = straightDistance + 2.0f * pi * circleRadius;
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

void Mission::updateStraightSegment(float elapsedTime, float remainingDistance)
{
  currentState = 11;
  reference.linearVelocity = profileVelocity(elapsedTime, remainingDistance);
  reference.angularVelocity = 0.0f;
}

void Mission::updateCircleSegment(float elapsedTime, float remainingDistance)
{
  currentState = 13;
  reference.linearVelocity = profileVelocity(elapsedTime, remainingDistance);
  reference.angularVelocity = reference.linearVelocity / circleRadius;
}

bool Mission::updateCourse(float currentTime, float traveledDistance)
{
  switch (currentState)
  {
    case 10:
      if (currentTime > endTime)
      {
        profileStartTime = currentTime;
        endTime = currentTime + totalCourseDistance / cruiseVelocity + 3.0f;
        currentState = 11;
      }
      break;

    case 11:
    case 13:
    {
      const float measuredProgress = traveledDistance - startDistance;
      const float progress = measuredProgress > 0.0f ? measuredProgress : 0.0f;
      const float remainingDistance = totalCourseDistance - progress;
      if (progress >= totalCourseDistance || currentTime > endTime)
      {
        reference = {0.0f, 0.0f};
        endTime = currentTime + 0.3f;
        currentState = 12;
      }
      else
      {
        const float elapsedTime = currentTime - profileStartTime;
        if (progress >= straightDistance)
          updateCircleSegment(elapsedTime, remainingDistance);
        else
          updateStraightSegment(elapsedTime, remainingDistance);
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