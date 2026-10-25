#include "speed_controller.h"

namespace
{
constexpr float maxMotorVoltage = 9.0f;

float clampMotorVoltage(float voltage)
{
  if (voltage < -maxMotorVoltage)
    return -maxMotorVoltage;
  if (voltage > maxMotorVoltage)
    return maxMotorVoltage;
  return voltage;
}

float updatePIOutput(
    float error,
    float Kp,
    float Ki,
    Integrator &errorIntegrator)
{
  const float candidateIntegral = errorIntegrator.preview(error);
  const float candidateOutput = Kp * error + Ki * candidateIntegral;

  const bool withinVoltageLimit =
      candidateOutput >= -maxMotorVoltage && candidateOutput <= maxMotorVoltage;
  const bool unwindingSaturation =
      (candidateOutput > maxMotorVoltage && error < 0.0f) ||
      (candidateOutput < -maxMotorVoltage && error > 0.0f);

  if (withinVoltageLimit || unwindingSaturation)
  {
    errorIntegrator.update(error);
    return candidateOutput;
  }

  return Kp * error + Ki * errorIntegrator.value();
}
}

SpeedController::SpeedController(
    float gearRatio,
    float wheelRadius,
    float Kp,
    float Ki,
    float sampleTime)
    : motorRadiansPerMeter(gearRatio / wheelRadius),
      Kp(Kp),
      Ki(Ki),
      leftErrorIntegrator(sampleTime),
      rightErrorIntegrator(sampleTime)
{
}

MotorVoltage SpeedController::update(
    float desiredLeftSpeed,
    float desiredRightSpeed,
    float measuredLeftAngularSpeed,
    float measuredRightAngularSpeed)
{
  const float leftReferenceMotorSpeed = motorRadiansPerMeter * desiredLeftSpeed;
  const float rightReferenceMotorSpeed = motorRadiansPerMeter * desiredRightSpeed;

  const float leftError = leftReferenceMotorSpeed - measuredLeftAngularSpeed;
  const float rightError = rightReferenceMotorSpeed - measuredRightAngularSpeed;

  MotorVoltage output = {
      clampMotorVoltage(updatePIOutput(leftError, Kp, Ki, leftErrorIntegrator)),
      clampMotorVoltage(updatePIOutput(rightError, Kp, Ki, rightErrorIntegrator))};

  return output;
}

void SpeedController::reset()
{
  leftErrorIntegrator.reset();
  rightErrorIntegrator.reset();
}
