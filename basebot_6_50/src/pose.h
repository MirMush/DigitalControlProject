#ifndef POSE_H
#define POSE_H

#include "integrator.h"

class PoseEstimator
{
public:
  PoseEstimator(
      float sampleTime,
      float wheelBase,
      IntegrationMethod method = IntegrationMethod::Tustin);

  void update(float leftWheelVelocity, float rightWheelVelocity);
  void reset();

  float x() const;
  float y() const;
  float heading() const;
  float distance() const;
  float forwardVelocity() const;
  float turnRate() const;

private:
  Integrator headingIntegrator;
  Integrator xIntegrator;
  Integrator yIntegrator;
  Integrator distanceIntegrator;
  float wheelBase;

  float poseX;
  float poseY;
  float poseHeading;
  float traveledDistance;
  float forwardVel;
  float angularVelocity;
};

#endif