#include "IMU/MPU6050.h"
#include "Button/Button.h"
#include "Synth/Synth.h"
#include <thread>

#define BUTTON_GPIO1 20
#define BUTTON_GPIO2 21

int main()
{
    const char *GPIO_CHIP{"gpiochip0"};
    constexpr int LEFT_BTN_OFFSET{20};
    constexpr int RIGHT_BTN_OFFSET{21};
    constexpr int MPU6050_ADDR1{0x68};
    constexpr int MPU6050_ADDR2{0x69};

    MPUSynth leftSynth {};
    leftSynth.start();

    MPU6050 leftSensor{MPU6050_ADDR1};
    leftSensor.registerCallback(&leftSynth);
    leftSynth.start();
    Button leftButton{};
    leftButton.registerCallback(&leftSensor);
    leftSensor.start();
    leftButton.start(GPIO_CHIP, LEFT_BTN_OFFSET);

    MPU6050 rightSensor{MPU6050_ADDR2};
    Button rightButton{};
    rightButton.registerCallback(&rightSensor);
    rightSensor.start();
    rightButton.start(GPIO_CHIP, RIGHT_BTN_OFFSET);

    // TODO: Should be replaced with callback registration code.
    // std::thread t1 (leftSensor.button_interrupt, ref(leftSensor), BUTTON_GPIO1).detach();
    // std::thread t2 (rightSensor.button_interrupt, ref(rightSensor), BUTTON_GPIO2).detach();

    // while (true) std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    fprintf(stdout, "Press any key to stop\n");
    getchar();
    leftButton.stop();
    rightButton.stop();
    leftSynth.stop();
    return 0;
}