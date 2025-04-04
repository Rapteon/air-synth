#include "MPU6050.h"

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
#define MOT_DETECT_CTRL 0x69
#define I2C_BUS "/dev/i2c-1"

MPU6050::MPU6050() {
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

MPU6050::~MPU6050() {
    close(file);
}

void MPU6050::init() {
    std::cout << "Initializing MPU6050..." << std::endl;
    writeRegister(PWR_MGMT_1, 0x00); // Wake up
    writeRegister(ACCEL_CONFIG, 0x00); // ±2g range
    writeRegister(INT_ENABLE, 0x00); // Disable interrupts by default
    std::cout << "MPU6050 initialized." << std::endl;
}

void MPU6050::enableMotionInterrupt() {
    writeRegister(MOT_THR, 0x05);    // Small motion threshold 
    writeRegister(MOT_DUR, 0x05);    // Motion must last for 5 samples
    writeRegister(MOT_DETECT_CTRL, 0x40); // Enable motion detection with 1ms delay
    writeRegister(INT_ENABLE, 0x40); // Enable motion interrupt
    std::cout << "Motion interrupt enabled." << std::endl;
}

void MPU6050::disableMotionInterrupt() {
    writeRegister(INT_ENABLE, 0x00); // Disable all interrupts
    std::cout << "Motion interrupt disabled." << std::endl;
}

void MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    std::lock_guard<std::mutex> lock(i2cMutex);
    uint8_t data[2] = {reg, value};
    if (write(file, data, 2) != 2) {
        std::cerr << "Failed to write to register " << (int)reg << std::endl;
    }
}

int16_t MPU6050::readWord(uint8_t reg) {
    std::lock_guard<std::mutex> lock(i2cMutex);
    uint8_t buffer[2];
    if (write(file, &reg, 1) != 1) {
        std::cerr << "Failed to write to register " << (int)reg << std::endl;
        return 0;
    }
    if (read(file, buffer, 2) != 2) {
        std::cerr << "Failed to read from register " << (int)reg << std::endl;
        return 0;
    }
    return (buffer[0] << 8) | buffer[1];
}

int16_t MPU6050::getAccelerationX() { return readWord(ACCEL_XOUT_H); }
int16_t MPU6050::getAccelerationY() { return readWord(ACCEL_YOUT_H); }
int16_t MPU6050::getAccelerationZ() { return readWord(ACCEL_ZOUT_H); }
int16_t MPU6050::getGyroscopeX() { return readWord(GYRO_XOUT_H); }
int16_t MPU6050::getGyroscopeY() { return readWord(GYRO_YOUT_H); }
int16_t MPU6050::getGyroscopeZ() { return readWord(GYRO_ZOUT_H); }
