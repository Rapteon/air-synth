#include "Printer.h"
#include <cstdlib>

Printer::Printer(const char* name) : running(false), name(name) {
  std::cerr << "Creater printer.\n";
}

Printer::~Printer() {
  if (!running)
    return;
  stop();
}
void Printer::start() {
  workerThread = std::thread(&Printer::worker, this);
  std::cerr << "Printer started.\n";
}

void Printer::stop() {
  if (!running)
    return;
  workerThread.join();
}

void Printer::worker() {
    while(running) {
        
    }
}

int main() {
  const char *GPIO_CHIP = "gpiochip0";
  constexpr int LEFT_BTN_OFFSET{20};
  constexpr int MPU_ADDR{0x68};

  Button leftButton{};
  MPU6050 leftSensor(MPU_ADDR);
  leftButton.registerCallback(&leftSensor);

  Printer leftPrinter{"leftPrinter"};
  leftSensor.registerCallback(&leftPrinter);

  leftButton.start(GPIO_CHIP);
  leftSensor.start();
  leftPrinter.start();

  return EXIT_SUCCESS;
}