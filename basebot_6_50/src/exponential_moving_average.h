#ifndef EXPONENTIAL_MOVING_AVERAGE_H
#define EXPONENTIAL_MOVING_AVERAGE_H

class ExponentialMovingAverage
{
public:
  explicit ExponentialMovingAverage(float alpha, bool enabled = true);

  float update(float rawValue);
  void reset(float value);
  float value() const;

private:
  float alpha;
  bool enabled;
  bool initialized = false;
  float filteredValue = 0.0f;
};

#endif