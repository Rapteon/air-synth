#include "MPU6050.h"
#include "GPIOEVENT.h"
#include <iostream>
#include <chrono>
#include <thread>

class ButtonInterruptHandler : public GPIOPin::GPIOEventCallbackInterface {
private:
    std::chrono::time_point<std::chrono::steady_clock> lastPressTime;
    std::chrono::milliseconds debounceDelay = std::chrono::milliseconds(500); // Increased debounce delay
    int pressCount = 0;
    bool mpuRunning = false;
    MPU6050& mpu;

public:
    ButtonInterruptHandler(MPU6050& mpu) : mpu(mpu) {}

    void hasEvent(gpiod_line_event& e) override {
        auto now = std::chrono::steady_clock::now();
        std::cout << "Interrupt Time: " << std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count() << std::endl;
        std::cout << "Last Press Time: " << std::chrono::time_point_cast<std::chrono::milliseconds>(lastPressTime).time_since_epoch().count() << std::endl;
        std::cout << "Press Count: " << pressCount << std::endl;
        std::cout << "Event Type: " << e.event_type << std::endl;

        // Handle only rising edge events
        if (e.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPressTime) > debounceDelay) {
                pressCount++;
                lastPressTime = now;

                if (pressCount == 1) {
                    std::cout << "Button pressed once, starting MPU..." << std::endl;
                    mpuRunning = true;
                    mpu.start();  // Start the MPU data collection here
                } else if (pressCount == 2) {
                    std::cout << "Button pressed twice, stopping MPU..." << std::endl;
                    mpuRunning = false;
                    mpu.stop();  // Stop the MPU data collection here
                    pressCount = 0;  // Reset count after stopping
                }
            }
        }
    }

    bool isMpuRunning() {
        return mpuRunning;
    }
};

class MPUInterruptHandler : public GPIOPin::GPIOEventCallbackInterface {
private:
    ButtonInterruptHandler& buttonHandler;
    MPU6050& mpu;

public:
    MPUInterruptHandler(ButtonInterruptHandler& buttonHandler, MPU6050& mpu) : buttonHandler(buttonHandler), mpu(mpu) {}

    void hasEvent(gpiod_line_event& e) override {
        if (buttonHandler.isMpuRunning()) {
            std::cout << "MPU interrupt detected, retrieving data..." << std::endl;
            int16_t rawAccelX = mpu.getAccelerationX();
            int16_t rawAccelY = mpu.getAccelerationY();
            int16_t rawAccelZ = mpu.getAccelerationZ();
            
            // Print raw data
            std::cout << "Raw Accel: X=" << rawAccelX << std::endl;
            // std::cout << "Raw Accel: X=" << rawAccelX << ", Y=" << rawAccelY << ", Z=" << rawAccelZ << std::endl;
            // std::cout << "Raw Gyro: X=" << rawGyroX << ", Y=" << rawGyroY << ", Z=" << rawGyroZ << std::endl;
        }
    }
};

int main() {
    MPU6050 mpu;
    ButtonInterruptHandler buttonHandler(mpu);
    MPUInterruptHandler mpuHandler(buttonHandler, mpu);

    GPIOPin buttonPin;
    GPIOPin mpuPin;

    buttonPin.registerCallback(&buttonHandler);
    mpuPin.registerCallback(&mpuHandler);

    try {
        buttonPin.start(16, 0, GPIOPin::PULL_UP); // Button on GPIO 16, pull-up
        mpuPin.start(17, 0); // MPU interrupt on GPIO 17

        std::cout << "Waiting for button press..." << std::endl;
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        buttonPin.stop();
        mpuPin.stop();

    } catch (const char* e) {
        std::cerr << "Error: " << e << std::endl;
        return 1;
    }
    return 0;
}