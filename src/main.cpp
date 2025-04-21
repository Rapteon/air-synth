#include "IMU/MPU6050.h"
#include "Button/Button.h"
#include "Synth/Synth.h"
#include <iostream>
#include <csignal>
#include <stdexcept>
#include <string>

int main() {
    try {
        // Define chip name and button offsets
        const char *GPIO_CHIP = "gpiochip0";
        constexpr int LEFT_BTN_OFFSET{20};
        constexpr int RIGHT_BTN_OFFSET{21};
        constexpr int MPU6050_ADDR1{0x68};
        constexpr int MPU6050_ADDR2{0x69};
        
        // // Create and initialize sensors
        std::cout << "Initializing left sensor (0x68)..." << std::endl;
        MPU6050 leftSensor(MPU6050_ADDR1);
        leftSensor.currentRootNote = "C";
        leftSensor.currentScaleType = "Major";
        leftSensor.currentInstrumentType = "Piano";
        
        std::cout << "Initializing right sensor (0x69)..." << std::endl;
        MPU6050 rightSensor(MPU6050_ADDR2);
        rightSensor.currentRootNote = "A";
        rightSensor.currentScaleType = "Minor";
        rightSensor.currentInstrumentType = "Moog";
        
        // Create buttons
        std::cout << "Setting up buttons..." << std::endl;
        Button leftButton{};
        Button rightButton{};
        
        // Register callbacks
        leftButton.registerCallback(&leftSensor);
        rightButton.registerCallback(&rightSensor);
        
        // // Attach synthesizers
        std::cout << "Attaching synthesizers..." << std::endl;
        leftSensor.attach_synth();
        rightSensor.attach_synth();
        
        // Start sensors
        std::cout << "Starting sensors..." << std::endl;
        leftSensor.start();
        rightSensor.start();
        
        // Start buttons
        std::cout << "Starting buttons..." << std::endl;
        try {
            leftButton.start(GPIO_CHIP, LEFT_BTN_OFFSET);
            rightButton.start(GPIO_CHIP, RIGHT_BTN_OFFSET);
        }
        catch (const std::runtime_error& e) {
            std::string errorMsg = e.what();
            if (errorMsg.find("GPIO") != std::string::npos || 
                errorMsg.find("IRQ") != std::string::npos) {
                std::cerr << "GPIO Error: " << e.what() << std::endl;
                std::cerr << "This application requires access to GPIO pins." << std::endl;
                std::cerr << "Try running the application with sudo or adding your user to the gpio group:" << std::endl;
                std::cerr << "  sudo usermod -a -G gpio $USER" << std::endl;
                std::cerr << "Then log out and log back in for the changes to take effect." << std::endl;
                return 1;
            }
            throw; // Re-throw if it's not a GPIO error
        }
        
        // The program will continue running with the buttons active
        std::cout << "Stop buttons..." << std::endl;
        leftButton.stop();
        rightButton.stop();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    catch (const char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }

    return 0;
}