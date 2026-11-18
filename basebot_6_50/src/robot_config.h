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
// Log at 5 ms intervals instead of every sample. The Teensy logger buffer is
// limited and fills up quickly when logging at 1 kHz for a multi-second run.
constexpr uint32_t logIntervalUs = 5000;
constexpr int logSampleDivider = logIntervalUs / sampleTimeUs;

constexpr float gearRatio = 9.6f;
constexpr float wheelRadius = 0.0315f;
constexpr float wheelBase = 0.15f;

// Control Mode
constexpr ControlMode controlMode = ControlMode::ClosedLoop;

// Speed Controller (PI-controller)
constexpr float speedProportionalGain = 0.04;
constexpr float speedIntegralTimeConstant = 0.5;
constexpr float speedIntegralGain =
    speedProportionalGain / speedIntegralTimeConstant;

// Acceleration-limited forward-speed reference
constexpr float accelerationProportionalGain = 25.0f;
constexpr float maximumLinearAcceleration = 0.5f; // Tune for the robot and load

// Turn Rate Controller (PI-controller)
constexpr bool turnRateControlEnabled = false;
constexpr float turnRateProportionalGain = 1.0f;
constexpr float turnRateIntegralTimeConstant = 0.2f;
constexpr float turnRateIntegralGain =
    turnRateProportionalGain / turnRateIntegralTimeConstant;

// Heading Controller (Lead Controller)
constexpr bool headingControlEnabled = true;
constexpr float headingKcGain = 0.5;
constexpr float headingTauZ = 1000.0;
constexpr float headingTauP = 0.1;

// Mission
constexpr MissionType missionType = MissionType::Square;
constexpr float missionDistance = 1.0f; 
constexpr float missionCruiseVelocity = 3.0f; // max 3 m/s
constexpr float missionTurnRate = 1.0f;
constexpr float openLoopMotorVoltage = 7.0f; // max 9V
}

#endif
