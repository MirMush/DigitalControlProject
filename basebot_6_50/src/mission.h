#ifndef MISSION_H
#define MISSION_H

struct MotionReference
{
  float forwardSpeed;
  float turnRate;
};

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
  void startConstantTurn(
      float targetDistance,
      float turnRate,
      float currentTime,
      float traveledDistance);
  bool update(float currentTime, float traveledDistance);
  void stop(float currentTime, float delay);
  State state() const;
  MotionReference reference() const;

private:
  State currentState;
  float endTime;
  float targetDistance;
  float startDistance;
  float cruiseVelocity;
  float commandedTurnRate;
  MotionReference currentReference;

  void begin(float targetDistance, float turnRate, float currentTime, float traveledDistance);
};

#endif