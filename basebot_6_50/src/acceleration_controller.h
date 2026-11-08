#ifndef ACCELERATION_CONTROLLER_H
#define ACCELERATION_CONTROLLER_H

class AccelerationController
{
public:
  AccelerationController(float Kp, float maxAcceleration, float sampleTime);

  float update(float reference);
  void reset(float initialVelocity = 0.0f);


private:
  float Kp;
  float maxAcceleration;
  float sampleTime;
  float limitedVelocity;
};

#endif