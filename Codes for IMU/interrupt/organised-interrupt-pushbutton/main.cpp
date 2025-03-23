#include "MPU6050.h"
#include "GPIOEVENT.h"
#include <iostream>
#include <chrono>
#include <thread>

class ButtonInterruptHandler : public GPIOPin::GPIOEventCallbackInterface {
private:
    std::chrono::time_point<std::chrono::steady_clock> lastPressTime;
    std::chrono::milliseconds debounceDelay = std::chrono::milliseconds(200);
    int pressCount = 0;
    bool mpuRunning = false;
    MPU6050& mpu;

public:
    ButtonInterruptHandler(MPU6050& mpu) : mpu(mpu) {}

    void hasEvent(gpiod_line_event& e) override {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPressTime) > debounceDelay) {
            pressCount++;
            lastPressTime = now;

            if (pressCount == 1) {
                std::cout << "Button pressed once, starting MPU..." << std::endl;
                mpuRunning = true;
            } else if (pressCount == 2) {
                std::cout << "Button pressed twice, stopping MPU..." << std::endl;
                mpuRunning = false;
                pressCount = 0;
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
    int16_t prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0;

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

            if (prevAccelX != 0) {
                double change = std::abs((double)(rawAccelX - prevAccelX) / prevAccelX) * 100.0;
                if (change > 10.0) {
                    std::cout << "X-axis change > 10%: " << change << "%" << std::endl;
                }
            }
            
            // Similar approach for Y & Z to compute deviation
            /* if (prevAccelY != 0) {
                double change = std::abs((double)(rawAccelY - prevAccelY) / prevAccelY) * 100.0;
                if (change > 10.0) {
                    std::cout << "Y-axis change > 10%: " << change << "%" << std::endl;
                }
            }
            if (prevAccelZ != 0) {
                double change = std::abs((double)(rawAccelZ - prevAccelZ) / prevAccelZ) * 100.0;
                if (change > 10.0) {
                    std::cout << "Z-axis change > 10%: " << change << "%" << std::endl;
                }
            }
            */
            
            prevAccelX = rawAccelX;
            prevAccelY = rawAccelY;
            prevAccelZ = rawAccelZ;
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