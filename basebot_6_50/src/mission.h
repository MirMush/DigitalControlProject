#ifndef MISSION_H
#define MISSION_H

class Mission
{
public:
  enum class State
  {
    Waiting,
    StartDelay,
    Driving,
    Finishing,
    Stopping
  };

  explicit Mission(float cruiseVelocity);

  bool waitingForStart() const;
  bool isActive() const;
  void startStraight(float targetDistance, float currentTime, float traveledDistance);
  bool updateStraight(float currentTime, float traveledDistance);
  void stop(float currentTime, float delay);
  State state() const;
  float speedReference() const;

private:
  State currentState;
  float endTime;
  float targetDistance;
  float startDistance;
  float cruiseVelocity;
  float reference;
};

#endif