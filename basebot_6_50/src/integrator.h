#ifndef INTEGRATOR_H
#define INTEGRATOR_H

enum class IntegrationMethod
{
  ForwardEuler,
  Tustin
};

class Integrator
{
public:
  explicit Integrator(float sampleTime, IntegrationMethod method = IntegrationMethod::Tustin);

  float update(float rate);
  float preview(float rate) const;
  float value() const;
  void reset();

private:
  float sampleTime;
  float previousRate;
  float integral;
  IntegrationMethod method;
};

#endif
