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
  Square,
  MotorStepSequence,
  MotorPulseSequence,
  CircularReturn,
};

enum class TurnDirection
{
  Left,
  Right,
};

constexpr uint32_t sampleTimeUs = 1000;
constexpr uint32_t logIntervalUs = 10000;
constexpr float sampleTimeSeconds = sampleTimeUs * 1e-6f;
constexpr float logIntervalSeconds = logIntervalUs * 1e-6f;
constexpr int logSampleDivider = logIntervalUs / sampleTimeUs;

// Model Parameters
constexpr float gearRatio = 9.6f;
constexpr float wheelRadius = 0.0315f;
constexpr float wheelBase = 0.15f;

// MPU9250 yaw-rate axis and sign for the robot's forward-left-up frame.
constexpr int imuYawRateAxis = 2;
constexpr float imuYawRateSign = 1.0f;
constexpr float radiansPerDegree = 0.017453292519943295f;

// Control Mode
constexpr ControlMode controlMode = ControlMode::ClosedLoop;

// Speed Controller (PI-controller)
constexpr float speedProportionalGain = 0.04;
constexpr float speedIntegralTimeConstant = 0.5;
constexpr float speedIntegralGain =
    speedProportionalGain / speedIntegralTimeConstant;

// Acceleration-limited forward-speed reference
constexpr float accelerationProportionalGain = 25.0f;
constexpr float maximumLinearAcceleration = 1.0f; // Tune for the robot and load

// Turn Rate Controller (PI-controller)
constexpr bool turnRateControlEnabled = false;
constexpr float turnRateProportionalGain = 1.0f;
constexpr float turnRateIntegralTimeConstant = 0.2f;
constexpr float turnRateIntegralGain =
    turnRateProportionalGain / turnRateIntegralTimeConstant;

// Heading Controller (Lead Controller)
constexpr bool headingControlEnabled = true;
constexpr float headingKcGain = 9.85;
constexpr float headingTauZ = 0.15;
constexpr float headingTauP = 0.09;

// Mission
constexpr MissionType missionType = MissionType::CircularReturn;
constexpr float missionDistance = 1.0f;
constexpr float missionCruiseVelocity = 1.0f; // max 3 m/s
constexpr float missionCurveVelocity = 0.5f;
constexpr float missionTurnRate = 1.0f;
constexpr float openLoopMotorVoltage = 7.0f; // max 9V
constexpr float firstStraightDistance = 1.0f;
constexpr float circleRadius = 0.5f;
constexpr float circleSweepAngleDegrees = 325.1f;
constexpr float circleFinalHeadingOffsetDegrees = 0.0f;
constexpr float secondStraightDistance = 1.5f;
constexpr float circleJoinBlendDistance = 0.2f;
constexpr float circleExitBlendDistance = 0.2f;
constexpr TurnDirection circleDirection = TurnDirection::Left;
}

#endif
