#ifndef CONTROLLEREVENT_H
#define CONTROLLEREVENT_H

class ControllerEvent {
public:
  ControllerEvent(double x, double y, double z);
  double getX();
  double getY();
  double getZ();

private:
  double x, y, z;
};

#endif