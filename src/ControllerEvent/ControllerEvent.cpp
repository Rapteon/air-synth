#include "ControllerEvent.h"

ControllerEvent::ControllerEvent(double x, double y, double z)
    : x(x), y(y), z(z) {
}

double ControllerEvent::getX() const noexcept {
    return x;
}

double ControllerEvent::getY() const noexcept {
    return y;
}

double ControllerEvent::getZ() const noexcept {
    return z;
}