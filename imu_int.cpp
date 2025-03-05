#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <mutex>
#include <cstdint>
#include <gpiod.hpp>
#include <thread>
#include <functional>
#include <string>
#include <chrono>


#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE 0x38
#define MOT_THR 0x1F
#define MOT_DUR 0x20
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define I2C_BUS "/dev/i2c-1"

class MPU6050 {
private:
    int file;
    std::mutex i2cMutex;

public:
    MPU6050() {
        file = open(I2C_BUS, O_RDWR);
        if (file < 0) {
            std::cerr << "Failed to open I2C bus" << std::endl;
            exit(1);
        }
        if (ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
            std::cerr << "Failed to acquire bus access" << std::endl;
            close(file);
            exit(1);
        }
        init();
    }

    ~MPU6050() {
        close(file);
    }

    void init() {
        writeRegister(PWR_MGMT_1, 0x00); // Wake up
	        writeRegister(ACCEL_CONFIG, 0x10); // ±8g range
        writeRegister(INT_ENABLE, 0x40); // Enable motion interrupt
        writeRegister(MOT_THR, 0x05); // Motion threshold adjust as needed,
        writeRegister(MOT_DUR, 0x05); // Motion duration (adjust as needed)
    }

    void writeRegister(uint8_t reg, uint8_t value) {
        std::lock_guard<std::mutex> lock(i2cMutex);
        uint8_t data[2] = {reg, value};
        write(file, data, 2);
    }

    int16_t readWord(uint8_t reg) {
        std::lock_guard<std::mutex> lock(i2cMutex);
        uint8_t buffer[2];
        write(file, &reg, 1);
        read(file, buffer, 2);
        return (buffer[0] << 8) | buffer[1];
    }
    int16_t getAccelerationX() { return readWord(ACCEL_XOUT_H); }
    int16_t getAccelerationY() { return readWord(ACCEL_YOUT_H); }
    int16_t getAccelerationZ() { return readWord(ACCEL_ZOUT_H); }
    int16_t getGyroscopeX() { return readWord(GYRO_XOUT_H); }
    int16_t getGyroscopeY() { return readWord(GYRO_YOUT_H); }
    int16_t getGyroscopeZ() { return readWord(GYRO_ZOUT_H); }
};


class GPIOInterruptHandler {
private:
    gpiod::line interruptLine;
    std::function<void()> callback;

public:
    GPIOInterruptHandler(const std::string& chipName, int lineNum, std::function<void()> callback)
        : callback(callback) {
        gpiod::chip chip(chipName);  // Open the GPIO chip
        interruptLine = chip.get_line(lineNum);  // Get the correct GPIO line

          if (!interruptLine) {
            throw std::runtime_error("Failed to find GPIO line");
        }

        interruptLine.request({__FUNCTION__, gpiod::line_request::EVENT_RISING_EDGE});
    }

    void handleInterrupt() {
        callback();
    }

    void waitForInterrupts() {
        while (true) { // Loop to continuously wait for interrupts
            try {
                interruptLine.event_wait(std::chrono::seconds(10)); // Wait up to 10 seconds
                gpiod::line_event event = interruptLine.event_read();
                handleInterrupt(); // Call the callback when interrupt occurs
            } catch (const std::exception& e) {
                std::cerr << "Timeout or Error: " << e.what() << std::endl;
            }
        }
    }
};


int main() {
    MPU6050 mpu; // Declare mpu BEFORE the lambda

    auto interruptCallback = [&mpu]() {
        std::cout << "Interrupt callback started." << std::endl;

        int16_t rawAccelX = mpu.getAccelerationX();
        int16_t rawAccelY = mpu.getAccelerationY();
        int16_t rawAccelZ = mpu.getAccelerationZ();
        int16_t rawGyroX = mpu.getGyroscopeX();
        int16_t rawGyroY = mpu.getGyroscopeY();
        int16_t rawGyroZ = mpu.getGyroscopeZ();

        // Accelerometer conversion (m/s²)
        double accelX = (double)rawAccelX / 4096.0 * 9.81;
        double accelY = (double)rawAccelY / 4096.0 * 9.81;
        double accelZ = (double)rawAccelZ / 4096.0 * 9.81;

        // Gyroscope conversion (degrees/second)
        double gyroX = (double)rawGyroX / 131.0;
        double gyroY = (double)rawGyroY / 131.0;
        double gyroZ = (double)rawGyroZ / 131.0;

        std::cout << "Acceleration X: " << accelX << " m/s², Y: " << accelY << " m/s² " << std::endl;
        	// std::cout << "Gyroscope X: " << gyroX << " °/s, Y: " << gyroY << " °/s " << std::endl;
    };

    try {
        GPIOInterruptHandler gpioHandler("gpiochip0", 17, interruptCallback);
        gpioHandler.waitForInterrupts(); // Start waiting for interrupts
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
