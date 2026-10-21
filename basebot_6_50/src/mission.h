#ifndef MISSION_H
#define MISSION_H

#include "motion_reference.h"

class Mission
{
public:
  Mission(float cruiseVelocity, float acceleration, float circleDiameter);

  bool waitingForStart() const;
  void startCourse(float straightDistance, float currentTime, float traveledDistance);
  bool updateCourse(float currentTime, float traveledDistance);
  void stop(float currentTime, float delay);
  int state() const;
  MotionReference motionReference() const;

private:
  float profileVelocity(float elapsedTime, float remainingDistance) const;
  void updateStraightSegment(float elapsedTime, float remainingDistance);
  void updateCircleSegment(float elapsedTime, float remainingDistance);

  int currentState;
  float endTime;
  float profileStartTime;
  float straightDistance;
  float totalCourseDistance;
  float startDistance;
  float cruiseVelocity;
  float acceleration;
  float circleRadius;
  MotionReference reference;
};

#endif