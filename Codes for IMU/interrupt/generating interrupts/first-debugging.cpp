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
        writeRegister(ACCEL_CONFIG, 0x00); // ±2g range
        writeRegister(INT_ENABLE, 0x40); // Enable motion interrupt
        writeRegister(MOT_THR, 0x20); // Motion threshold adjust as needed,
        writeRegister(MOT_DUR, 0x30); // Motion duration (adjust as needed)
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
        std::cout << "readWord() buffer: " << (int)buffer[0] << ", " << (int)buffer[1] << std::endl;
        int16_t result = (buffer[0] << 8) | buffer[1];
        std::cout << "readWord() result: " << result << std::endl;
        return result;
    }

    int16_t getAccelerationX() {
        std::cout << "getAccelerationX() called." << std::endl;
        int16_t result = readWord(ACCEL_XOUT_H);
        std::cout << "getAccelerationX() result: " << result << std::endl;
        return result;
    }
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
        gpiod::chip chip(chipName);
        interruptLine = chip.get_line(lineNum);

        if (!interruptLine) {
            std::cerr << "Failed to find GPIO line" << std::endl;
            throw std::runtime_error("Failed to find GPIO line");
        }

        interruptLine.request({__FUNCTION__, gpiod::line_request::EVENT_RISING_EDGE});
        std::cout << "GPIO line requested successfully." << std::endl;
    }

    void handleInterrupt() {
        callback();
    }

    void waitForInterrupts() {
        std::cout << "Starting waitForInterrupts loop." << std::endl;
        while (true) {
            try {
                std::cout << "Waiting for interrupt..." << std::endl;
                interruptLine.event_wait(std::chrono::seconds(10));
                std::cout << "Interrupt detected!" << std::endl;
                gpiod::line_event event = interruptLine.event_read();
                handleInterrupt();
            } catch (const std::exception& e) {
                std::cerr << "Timeout or Error: " << e.what() << std::endl;
            }
            std::cout << "GPIO Line Value: " << interruptLine.get_value() << std::endl;
        }
    }
};

int main() {
    MPU6050 mpu;

    auto interruptCallback = [&mpu]() {
        std::cout << "Interrupt callback started." << std::endl;
        int16_t accelX = mpu.getAccelerationX();
        int16_t accelY = mpu.getAccelerationY();
        int16_t accelZ = mpu.getAccelerationZ();
        int16_t gyroX = mpu.getGyroscopeX();
        int16_t gyroY = mpu.getGyroscopeY();
        int16_t gyroZ = mpu.getGyroscopeZ();
        std::cout << "Interrupt occurred! Acceleration X: " << accelX << ", Y: " << accelY << ", Z: " << accelZ << std::endl;
        std::cout << "Gyroscope X: " << gyroX << ", Y: " << gyroY << ", Z: " << gyroZ << std::endl;
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