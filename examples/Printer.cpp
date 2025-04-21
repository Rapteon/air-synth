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
  constexpr int RIGHT_BTN_OFFSET{21};
  constexpr int MPU_ADDR_LEFT{0x68};
  constexpr int MPU_ADDR_RIGHT{0x69};

  Button leftButton{};
  MPU6050 leftSensor(MPU_ADDR_LEFT);
  leftButton.registerCallback(&leftSensor);

  Printer leftPrinter{"leftPrinter"};
  leftSensor.registerCallback(&leftPrinter);

  leftButton.start(GPIO_CHIP, LEFT_BTN_OFFSET);
  leftSensor.start();
  leftPrinter.start();

  Button rightButton{};
  MPU6050 rightSensor(MPU_ADDR_RIGHT);
  rightButton.registerCallback(&rightSensor);

  Printer rightPrinter{"rightPrinter"};
  rightSensor.registerCallback(&rightPrinter);

  rightButton.start(GPIO_CHIP, RIGHT_BTN_OFFSET);
  rightSensor.start();
  rightPrinter.start();

  leftPrinter.stop();
  leftButton.stop();
  rightPrinter.stop();
  rightButton.stop();
  return EXIT_SUCCESS;
}
