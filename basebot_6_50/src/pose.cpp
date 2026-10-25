#include "pose.h"

#include <math.h>

PoseEstimator::PoseEstimator(
    float sampleTime,
    float wheelBase,
    float turnRateFilterAlpha,
    bool turnRateFilterEnabled,
    IntegrationMethod method)
    : headingIntegrator(sampleTime, method),
      xIntegrator(sampleTime, method),
      yIntegrator(sampleTime, method),
      distanceIntegrator(sampleTime, method),
      turnRateFilter(turnRateFilterAlpha, turnRateFilterEnabled),
      wheelBase(wheelBase),
      poseX(0.0f),
      poseY(0.0f),
      poseHeading(0.0f),
      traveledDistance(0.0f),
      forwardVel(0.0f),
      rawTurnRateValue(0.0f),
      turnRateValue(0.0f)
{
}

void PoseEstimator::update(
    float leftWheelVelocity,
    float rightWheelVelocity,
    float rawLeftWheelVelocity,
    float rawRightWheelVelocity)
{
  forwardVel = (leftWheelVelocity + rightWheelVelocity) / 2.0f;
  rawTurnRateValue = (rawRightWheelVelocity - rawLeftWheelVelocity) / wheelBase;
  turnRateValue = turnRateFilter.update(rawTurnRateValue);

  poseHeading = headingIntegrator.update(turnRateValue);

  const float xVelocity = forwardVel * cosf(poseHeading);
  const float yVelocity = forwardVel * sinf(poseHeading);
  poseX = xIntegrator.update(xVelocity);
  poseY = yIntegrator.update(yVelocity);
  traveledDistance = distanceIntegrator.update(forwardVel);
}

void PoseEstimator::reset()
{
  poseX = 0.0f;
  poseY = 0.0f;
  poseHeading = 0.0f;
  traveledDistance = 0.0f;
  forwardVel = 0.0f;
  rawTurnRateValue = 0.0f;
  turnRateValue = 0.0f;

  turnRateFilter.reset(0.0f);
  headingIntegrator.reset();
  xIntegrator.reset();
  yIntegrator.reset();
  distanceIntegrator.reset();
}

float PoseEstimator::x() const
{
  return poseX;
}

float PoseEstimator::y() const
{
  return poseY;
}

float PoseEstimator::heading() const
{
  return poseHeading;
}

float PoseEstimator::distance() const
{
  return traveledDistance;
}

float PoseEstimator::forwardVelocity() const
{
  return forwardVel;
}

float PoseEstimator::rawTurnRate() const
{
  return rawTurnRateValue;
}

float PoseEstimator::turnRate() const
{
  return turnRateValue;
}