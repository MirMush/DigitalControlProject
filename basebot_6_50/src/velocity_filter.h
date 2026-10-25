#ifndef VELOCITY_FILTER_H
#define VELOCITY_FILTER_H

class VelocityFilter
{
public:
  VelocityFilter(float alpha, bool enabled);

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