#include "pose.h"

#include <math.h>

PoseEstimator::PoseEstimator(float sampleTime, float wheelBase, IntegrationMethod method)
    : headingIntegrator(sampleTime, method),
      xIntegrator(sampleTime, method),
      yIntegrator(sampleTime, method),
      distanceIntegrator(sampleTime, method),
      wheelBase(wheelBase),
      poseX(0.0f),
      poseY(0.0f),
      poseHeading(0.0f),
      traveledDistance(0.0f),
      forwardVel(0.0f),
      angularVelocity(0.0f)
{
}

void PoseEstimator::update(float leftWheelVelocity, float rightWheelVelocity)
{
  forwardVel = (leftWheelVelocity + rightWheelVelocity) / 2.0f;
  angularVelocity = (rightWheelVelocity - leftWheelVelocity) / wheelBase;

  poseHeading = headingIntegrator.update(angularVelocity);

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
  angularVelocity = 0.0f;

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

float PoseEstimator::turnRate() const
{
  return angularVelocity;
}