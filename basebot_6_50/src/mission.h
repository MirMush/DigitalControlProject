#ifndef MISSION_H
#define MISSION_H

struct MotionReference
{
  float forwardSpeed;
  float heading;
};

struct MotorVoltageReference
{
  float left;
  float right;
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

  explicit Mission();

  void begin(float currentTime);
  bool waitingForStart() const;
  bool isActive() const;
  bool update(float currentTime, float traveledDistance);
  bool usesDirectMotorVoltage() const;
  void stop(float currentTime, float delay);
  State state() const;
  MotionReference reference() const;
  MotorVoltageReference motorVoltage() const;

private:
  State currentState;
  float endTime;
  float sequenceStartTime;
  float pathStartDistance;
  float circleArcEntryHeading;
  float circleArcLength;
  MotionReference currentReference;
  MotorVoltageReference currentMotorVoltage;

};

#endif