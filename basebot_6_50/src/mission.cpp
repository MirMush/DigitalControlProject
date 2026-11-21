#include "mission.h"

#include "robot_config.h"

#include <math.h>

namespace
{
struct VoltageStep
{
  float time;
  float voltage;
};

constexpr VoltageStep stepSequence[] = {
  {0.0f, 0.0f},
  {0.020f, 1.5f},
  {0.500f, 3.0f},
  {1.000f, 6.0f},
  {1.500f, 0.0f},
};

constexpr VoltageStep pulseSequence[] = {
  {0.0f, 0.0f},
  {0.050f, 6.0f},
  {0.060f, 0.0f},
  {0.300f, 6.0f},
  {0.310f, 0.0f},
  {0.600f, 6.0f},
  {0.610f, 0.0f},
};

template <unsigned int N>
float voltageAt(const VoltageStep (&steps)[N], float elapsedTime)
{
  float voltage = 0.0f;
  for (const VoltageStep &step : steps)
  {
    if (elapsedTime + RobotConfig::sampleTimeSeconds * 0.5f < step.time)
      break;
    voltage = step.voltage;
  }
  return voltage;
}

float smoothHeading(
    float startHeading,
    float endHeading,
    float startSlope,
    float endSlope,
    float segmentLength,
    float progress)
{
  const float t = progress / segmentLength;
  const float t2 = t * t;
  const float t3 = t2 * t;
  const float startWeight = 2.0f * t3 - 3.0f * t2 + 1.0f;
  const float startSlopeWeight = t3 - 2.0f * t2 + t;
  const float endWeight = -2.0f * t3 + 3.0f * t2;
  const float endSlopeWeight = t3 - t2;
  return startWeight * startHeading +
         startSlopeWeight * segmentLength * startSlope +
         endWeight * endHeading +
         endSlopeWeight * segmentLength * endSlope;
}

struct ArcDisplacement
{
  float x;
  float y;
};

ArcDisplacement circleArcDisplacement(
    float entryHeading,
    float arcLength,
    float entryBlendDistance,
    float exitBlendDistance)
{
  constexpr int integrationSteps = 128;
  constexpr float pi = 3.14159265f;
  const float radius = RobotConfig::circleRadius;
  const float finalHeading =
      pi + RobotConfig::circleFinalHeadingOffsetDegrees * pi / 180.0f;
  const float stepLength = arcLength / integrationSteps;
  ArcDisplacement displacement{0.0f, 0.0f};

  for (int i = 0; i < integrationSteps; ++i)
  {
    const float progress = (i + 0.5f) * stepLength;
    float heading;
    if (progress < entryBlendDistance)
    {
      heading = smoothHeading(
          0.0f,
          entryHeading + entryBlendDistance / radius,
          0.0f,
          1.0f / radius,
          entryBlendDistance,
          progress);
    }
    else if (progress < arcLength - exitBlendDistance)
    {
      heading = entryHeading + progress / radius;
    }
    else
    {
      const float exitProgress =
          progress - (arcLength - exitBlendDistance);
      heading = smoothHeading(
          entryHeading + (arcLength - exitBlendDistance) / radius,
          finalHeading,
          1.0f / radius,
          0.0f,
          exitBlendDistance,
          exitProgress);
    }

    displacement.x += cosf(heading) * stepLength;
    displacement.y += sinf(heading) * stepLength;
  }
  return displacement;
}

}

Mission::Mission()
    : currentState(State::Waiting),
      endTime(0.0f),
      sequenceStartTime(0.0f),
      pathStartDistance(0.0f),
      circleArcEntryHeading(0.0f),
      circleArcLength(0.0f),
      currentReference{0.0f, 0.0f},
      currentMotorVoltage{0.0f, 0.0f}
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
  currentReference = {0.0f, 0.0f};
  currentMotorVoltage = {0.0f, 0.0f};
  sequenceStartTime = currentTime;
  if (RobotConfig::missionType == RobotConfig::MissionType::CircularReturn)
  {
    // Solve entry heading and arc length using the configured blends.
    constexpr float pi = 3.14159265f;
    const float sweepAngle =
        RobotConfig::circleSweepAngleDegrees * pi / 180.0f;
    const float halfChordAngle = (2.0f * pi - sweepAngle) / 2.0f;
    const float targetSeparation =
        2.0f * RobotConfig::circleRadius * sinf(halfChordAngle);
    float entryHeading = halfChordAngle - 3.14159265f / 2.0f;
    float arcLength = RobotConfig::circleRadius * sweepAngle;
    constexpr float finiteDifference = 0.001f;

    for (int i = 0; i < 5; ++i)
    {
      const ArcDisplacement current =
          circleArcDisplacement(
              entryHeading,
              arcLength,
              RobotConfig::circleJoinBlendDistance,
              RobotConfig::circleExitBlendDistance);
      const ArcDisplacement headingPerturbed =
          circleArcDisplacement(
              entryHeading + finiteDifference,
              arcLength,
              RobotConfig::circleJoinBlendDistance,
              RobotConfig::circleExitBlendDistance);
      const ArcDisplacement lengthPerturbed =
          circleArcDisplacement(
              entryHeading,
              arcLength + finiteDifference,
              RobotConfig::circleJoinBlendDistance,
              RobotConfig::circleExitBlendDistance);
      const float dx = current.x;
      const float dy = current.y - targetSeparation;
      const float dxdHeading = (headingPerturbed.x - current.x) / finiteDifference;
      const float dydHeading = (headingPerturbed.y - current.y) / finiteDifference;
      const float dxdLength = (lengthPerturbed.x - current.x) / finiteDifference;
      const float dydLength = (lengthPerturbed.y - current.y) / finiteDifference;
      const float determinant =
          dxdHeading * dydLength - dxdLength * dydHeading;

      if (fabsf(determinant) < 1e-6f)
        break;

      entryHeading += (-dx * dydLength + dxdLength * dy) / determinant;
      arcLength += (-dxdHeading * dy + dydHeading * dx) / determinant;
    }

    const float direction =
        RobotConfig::circleDirection == RobotConfig::TurnDirection::Left
            ? 1.0f
            : -1.0f;
    circleArcEntryHeading = direction * entryHeading;
    circleArcLength = arcLength;
  }

  if (usesDirectMotorVoltage())
  {
    currentState = State::Driving;
  }
  else
  {
    endTime = currentTime + 0.020f;
    currentState = State::StartDelay;
  }
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
        pathStartDistance = traveledDistance;
        currentReference = {
          RobotConfig::missionCruiseVelocity,
          0.0f
        };
        currentState = State::Driving;
      }
      break;

    case State::Driving:
    {
      if (usesDirectMotorVoltage())
      {
        const float elapsedTime = currentTime - sequenceStartTime;
        const bool stepTest =
            RobotConfig::missionType == RobotConfig::MissionType::MotorStepSequence;
        const float voltage = stepTest
            ? voltageAt(stepSequence, elapsedTime)
            : voltageAt(pulseSequence, elapsedTime);
        currentMotorVoltage = {voltage, voltage};

        const float duration = stepTest ? 2.0f : 0.75f;
        if (elapsedTime + RobotConfig::sampleTimeSeconds * 0.5f >= duration)
        {
          currentMotorVoltage = {0.0f, 0.0f};
          currentState = State::Waiting;
          return true;
        }
        break;
      }

      if (RobotConfig::missionType == RobotConfig::MissionType::CircularReturn)
      {
        const float progress = traveledDistance - pathStartDistance;
        const float arcStart = RobotConfig::firstStraightDistance -
                               RobotConfig::circleJoinBlendDistance;
        const float arcEnd = arcStart + circleArcLength;
        const float entryBlendDistance = RobotConfig::circleJoinBlendDistance;
        const float exitBlendDistance = RobotConfig::circleExitBlendDistance;
        const float entryBlendEnd = arcStart + entryBlendDistance;
        const float exitBlendStart = arcEnd - exitBlendDistance;
        const float finalStraightStart = arcEnd;
        const float pathEnd = finalStraightStart +
                              RobotConfig::secondStraightDistance;
        const float direction =
            RobotConfig::circleDirection == RobotConfig::TurnDirection::Left
                ? 1.0f
                : -1.0f;
        const float entryHeading = circleArcEntryHeading;
        const float finalHeading = direction *
            (3.14159265f +
             RobotConfig::circleFinalHeadingOffsetDegrees *
                 3.14159265f / 180.0f);
        const float arcHeadingAtEntryBlendEnd =
            entryHeading + direction * entryBlendDistance / RobotConfig::circleRadius;
        const float arcHeadingAtExitBlendStart =
            entryHeading + direction * (circleArcLength - exitBlendDistance) /
                               RobotConfig::circleRadius;
        float heading;

        if (progress < arcStart)
        {
          heading = 0.0f;
        }
        else if (progress < entryBlendEnd)
        {
          heading = smoothHeading(
              0.0f,
              arcHeadingAtEntryBlendEnd,
              0.0f,
              direction / RobotConfig::circleRadius,
              entryBlendDistance,
              progress - arcStart);
        }
        else if (progress < exitBlendStart)
        {
          heading = entryHeading + direction * (progress - arcStart) /
                                      RobotConfig::circleRadius;
        }
        else if (progress < arcEnd)
        {
          const float exitProgress = progress - exitBlendStart;
          heading = smoothHeading(
              arcHeadingAtExitBlendStart,
              finalHeading,
              direction / RobotConfig::circleRadius,
              0.0f,
              exitBlendDistance,
              exitProgress);
        }
        else
        {
          heading = finalHeading;
        }

        if (progress >= pathEnd)
        {
          currentReference = {0.0f, finalHeading};
          endTime = currentTime + 0.2f;
          currentState = State::Finishing;
        }
        else
        {
          float forwardSpeed = RobotConfig::missionCruiseVelocity;
          if (progress >= arcStart && progress < arcEnd)
          {
            forwardSpeed = RobotConfig::missionCurveVelocity;
          }
          else if (progress >= arcEnd)
          {
            const float remainingDistance = pathEnd - progress;
            const float brakingAcceleration =
                RobotConfig::maximumLinearAcceleration * 0.8f;
            const float brakingSpeed = sqrtf(
                2.0f * brakingAcceleration * remainingDistance);
            if (brakingSpeed < forwardSpeed)
              forwardSpeed = brakingSpeed;
          }
          currentReference = {forwardSpeed, heading};
        }
        break;
      }

      if (traveledDistance >= RobotConfig::missionDistance)
      {
        currentReference = {0.0f, 0.0f};
        endTime = currentTime + 0.2f;
        currentState = State::Finishing;
      }
      break;
    }

    case State::Finishing:
      currentReference.forwardSpeed = 0.0f;
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

bool Mission::usesDirectMotorVoltage() const
{
  return RobotConfig::missionType == RobotConfig::MissionType::MotorStepSequence ||
         RobotConfig::missionType == RobotConfig::MissionType::MotorPulseSequence;
}

void Mission::stop(float currentTime, float delay)
{
  currentReference = {0.0f, 0.0f};
  currentMotorVoltage = {0.0f, 0.0f};
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

MotorVoltageReference Mission::motorVoltage() const
{
  return currentMotorVoltage;
}