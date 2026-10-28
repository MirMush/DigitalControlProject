#ifndef HEADING_CONTROLLER_H
#define HEADING_CONTROLLER_H

class HeadingController
{
public:
  HeadingController(float Kc, float tau_z, float tau_p, float sampleTime);

  float update(float reference, float measuredHeading);
  void reset();

private:
  float Kc;
  float tau_z;
  float tau_p;
  float sampleTime;
  float previousError;
  float previousOutput;
  float K1; // constant in front of u[k-1]
  float K2; // constant in front of e[k-1]
  float K3; // constant in front of e[k]
  float K4; // denominator
};

#endif