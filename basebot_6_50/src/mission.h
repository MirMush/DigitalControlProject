#ifndef MISSION_H
#define MISSION_H

#include "motion_reference.h"

class Mission
{
public:
  Mission(float cruiseVelocity, float acceleration);

  bool waitingForStart() const;
  void startStraight(float targetDistance, float currentTime, float traveledDistance);
  bool updateStraight(float currentTime, float traveledDistance);
  void stop(float currentTime, float delay);
  int state() const;
  MotionReference motionReference() const;

private:
  float profileVelocity(float elapsedTime, float remainingDistance) const;

  int currentState;
  float endTime;
  float profileStartTime;
  float targetDistance;
  float startDistance;
  float cruiseVelocity;
  float acceleration;
  MotionReference reference;
};

#endif