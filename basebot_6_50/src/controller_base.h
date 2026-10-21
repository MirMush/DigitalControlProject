#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

class Controller
{
public:
  virtual ~Controller() = default;
  virtual void reset() = 0;
};

#endif
