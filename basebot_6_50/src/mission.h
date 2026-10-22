#ifndef MISSION_H
#define MISSION_H

class Mission
{
public:
  explicit Mission(float cruiseVelocity);

  bool waitingForStart() const;
  void startStraight(float targetDistance, float currentTime, float traveledDistance);
  bool updateStraight(float currentTime, float traveledDistance);
  void stop(float currentTime, float delay);
  int state() const;
  float speedReference() const;

private:
  int currentState;
  float endTime;
  float targetDistance;
  float startDistance;
  float cruiseVelocity;
  float reference;
};

#endif