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

enum class MissionType
{
  Straight,
  ConstantTurn
};

constexpr ControlMode controlMode = ControlMode::ClosedLoop;
constexpr MissionType missionType = MissionType::ConstantTurn;

constexpr uint32_t sampleTimeUs = 1000;
constexpr uint32_t logIntervalUs = 1000;
constexpr float sampleTimeSeconds = sampleTimeUs * 1e-6f;
constexpr float logIntervalSeconds = logIntervalUs * 1e-6f;
constexpr int logSampleDivider = logIntervalUs / sampleTimeUs;

constexpr float gearRatio = 9.6f;
constexpr float wheelRadius = 0.0315f;
constexpr float wheelBase = 0.15f;

constexpr float speedProportionalGain = 0.04;
constexpr float speedIntegralTimeConstant = 0.1;
constexpr float speedIntegralGain =
    speedProportionalGain / speedIntegralTimeConstant;
    
constexpr bool turnRateControlEnabled = true;
constexpr float turnRateProportionalGain = 1.0f;
constexpr float turnRateIntegralTimeConstant = 0.2f;
constexpr float turnRateIntegralGain =
    turnRateProportionalGain / turnRateIntegralTimeConstant;

constexpr bool headingControlEnabled = true;
constexpr float headingKcGain = 1.0;
constexpr float headingTauZ = 1.0;
constexpr float headingTauP = 0.5;

constexpr float turnRateFilterAlpha = 0.005f;
constexpr bool turnRateFilterEnabled = false;

constexpr float velocityFilterAlpha = 0.01f;
constexpr bool velocityFilterEnabled = false;

constexpr float missionDistance = 1.0f;
constexpr float missionCruiseVelocity = 0.40f;
constexpr float missionTurnRate = 1.0f;
constexpr float openLoopMotorVoltage = 7.0f;
}

#endif
