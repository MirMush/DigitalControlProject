#ifndef POSE_H
#define POSE_H

#include "exponential_moving_average.h"
#include "integrator.h"

class PoseEstimator
{
public:
  PoseEstimator(
      float sampleTime,
      float wheelBase,
      float turnRateFilterAlpha,
      bool turnRateFilterEnabled,
      IntegrationMethod method = IntegrationMethod::Tustin);

  void update(
      float leftWheelVelocity,
      float rightWheelVelocity,
      float rawLeftWheelVelocity,
      float rawRightWheelVelocity);
  void reset();

  float x() const;
  float y() const;
  float heading() const;
  float distance() const;
  float forwardVelocity() const;
  float rawTurnRate() const;
  float turnRate() const;

private:
  Integrator headingIntegrator;
  Integrator xIntegrator;
  Integrator yIntegrator;
  Integrator distanceIntegrator;
  ExponentialMovingAverage turnRateFilter;
  float wheelBase;

  float poseX;
  float poseY;
  float poseHeading;
  float traveledDistance;
  float forwardVel;
  float rawTurnRateValue;
  float turnRateValue;
};

#endif