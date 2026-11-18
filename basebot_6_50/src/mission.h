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

  void begin(float currentTime);
  bool waitingForStart() const;
  bool isActive() const;
  bool update(float currentTime);
  void stop(float currentTime, float delay);
  State state() const;
  float speedReference() const;

private:
  State currentState;
  float endTime;
  MotionReference currentReference;

};

#endif