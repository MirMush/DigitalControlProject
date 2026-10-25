#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

namespace RobotConfig
{
enum class ControlMode
{
  OpenLoop,
  ClosedLoop
};

constexpr ControlMode controlMode = ControlMode::ClosedLoop;

constexpr uint32_t sampleTimeUs = 1000;
constexpr uint32_t logIntervalUs = 10000;
constexpr int logSampleDivider = logIntervalUs / sampleTimeUs;

constexpr float gearRatio = 9.6f;
constexpr float wheelRadius = 0.0315f;
constexpr float wheelBase = 0.15f;

constexpr float speedProportionalGain = 0.1202f;
constexpr float velocityFilterAlpha = 0.02f;
constexpr bool velocityFilterEnabled = true;

constexpr float missionDistance = 1.0f;
constexpr float missionCruiseVelocity = 0.40f;
constexpr float openLoopMotorVoltage = 1.0f;
}

#endif
