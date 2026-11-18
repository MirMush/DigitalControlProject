#include "mission.h"

#include "robot_config.h"

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
}

Mission::Mission()
    : currentState(State::Waiting),
      endTime(0.0f),
      sequenceStartTime(0.0f),
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

      if (traveledDistance >= RobotConfig::missionDistance)
      {
        currentReference = {0.0f, 0.0f};
        endTime = currentTime + 0.2f;
        currentState = State::Finishing;
      }
      break;
    }

    case State::Finishing:
      currentReference = {0.0f, 0.0f};
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