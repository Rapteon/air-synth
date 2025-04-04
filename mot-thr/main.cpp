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

        // Handle only rising edge events
        if (e.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPressTime) > debounceDelay) {
                pressCount++;
                lastPressTime = now;

                if (pressCount == 1) {
                    std::cout << "Button pressed once, enabling MPU motion detection..." << std::endl;
                    mpuRunning = true;
                    mpu.enableMotionInterrupt();  // Enable motion detection but don't start data retrieval yet.
                } 
                else if (pressCount == 2) {
                    std::cout << "Button pressed twice, stopping MPU..." << std::endl;
                    mpuRunning = false;
                    mpu.disableMotionInterrupt(); // Disable motion detection
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
    MPUInterruptHandler(ButtonInterruptHandler& buttonHandler, MPU6050& mpu)
        : buttonHandler(buttonHandler), mpu(mpu) {}

    void hasEvent(gpiod_line_event& e) override {
        if (buttonHandler.isMpuRunning()) {
            std::cout << "MPU motion interrupt detected, retrieving data..." << std::endl;

            // Retrieve scaled acceleration values in m/s²
            double accelX = mpu.getAccelXMps2();
            double accelY = mpu.getAccelYMps2();
            double accelZ = mpu.getAccelZMps2();

            // Print formatted data with two decimal places
            std::cout << std::fixed << std::setprecision(2)
                      << "Accel: X=" << accelX << " m/s², "
                      << "Y=" << accelY << " m/s², "
                      << "Z=" << accelZ << " m/s²" << std::endl;
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
