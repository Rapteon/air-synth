#include "ControllerEvent/ControllerEvent.h"

ControllerEvent::ControllerEvent(double x, double y, double z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

double ControllerEvent::getX() {
  return x;
}
double ControllerEvent::getY() {
  return y;
}
double ControllerEvent::getZ() {
  return z;
}