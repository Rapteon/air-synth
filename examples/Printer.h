#ifndef PRINTER_H
#define PRINTER_H

#include "IMU/MPU6050.h"

class Printer : public MPU6050::ControllerCallbackInterface {
public:
  virtual void hasEvent(ControllerEvent &e) override {
    std::cerr << "x-acceleration: " << e.getX()
              << ";y-acceleration: " << e.getY()
              << ";z-acceleration: " << e.getZ() << '\n';
  }

  Printer(const char* name);
  ~Printer();
  void start();
  void stop();

private:
  std::atomic<bool> running;
  std::thread workerThread;
  const char* name;

  void worker();
};

#endif