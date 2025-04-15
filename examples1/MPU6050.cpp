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
#define ACCEL_SCALE 16384.0
#define G 9.81

// default threshold and increment values
static const double DEFAULT_ACCEL_THRESHOLD_POS = 5.0;
static const double DEFAULT_ACCEL_THRESHOLD_NEG = -5.0;
static const int DEFAULT_INCREMENT_VALUE = 5;

MPU6050::MPU6050() :
    file(-1),
    counter_x(0),
    counter_y(0),
    threshold_crossed_x(false),
    threshold_crossed_y(false),
    threshold_pos(DEFAULT_ACCEL_THRESHOLD_POS),
    threshold_neg(DEFAULT_ACCEL_THRESHOLD_NEG),
    increment_value(DEFAULT_INCREMENT_VALUE)
{
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
    // it is assumed that motion interrupt has been disabled beforehand.
    close(file);
}

void MPU6050::init() {
    std::cout << "Initializing MPU6050..." << std::endl;
    writeRegister(PWR_MGMT_1, 0x00);    // wake up
    writeRegister(ACCEL_CONFIG, 0x00);   // +-2g range
    writeRegister(INT_ENABLE, 0x00);     // disable interrupts by default
    std::cout << "MPU6050 started." << std::endl;
}


void MPU6050::enableMotionInterrupt() {
    writeRegister(MOT_THR, 0x04);         // set motion threshold = 4
    writeRegister(MOT_DUR, 0x04);         // set motion duration = 4 samples
    writeRegister(MOT_DETECT_CTRL, 0x40); // enable motion detection with 1ms delay
    writeRegister(INT_ENABLE, 0x40);      // enable motion interrupt
    std::cout << "Motion interrupt enabled." << std::endl;
}

void MPU6050::disableMotionInterrupt() {
    writeRegister(INT_ENABLE, 0x00); // disable all interrupts
    std::cout << "Motion interrupt disabled." << std::endl;
}

void MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    std::lock_guard<std::mutex> lock(i2cMutex);
    uint8_t data[2] = {reg, value};
    if (write(file, data, 2) != 2) {
        std::cerr << "failed to write to register " << (int)reg << std::endl;
    } else {
        std::cout << "Wrote " << (int)value << " to register " << (int)reg << std::endl;
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

    // combine high and low bytes into a signed 16-bit integer
    int16_t value = (int16_t)((buffer[0] << 8) | buffer[1]);

    
    return value;
}	

// raw acceleration getters using the MPU6050 registers
int16_t MPU6050::getAccelerationX() { return readWord(ACCEL_XOUT_H); }
int16_t MPU6050::getAccelerationY() { return readWord(ACCEL_YOUT_H); }
int16_t MPU6050::getAccelerationZ() { return readWord(ACCEL_ZOUT_H); }
int16_t MPU6050::getGyroscopeX()  { return readWord(GYRO_XOUT_H); }
int16_t MPU6050::getGyroscopeY()  { return readWord(GYRO_YOUT_H); }
int16_t MPU6050::getGyroscopeZ()  { return readWord(GYRO_ZOUT_H); }

// to keep modularity, if you want raw acceleration getters with clearer names:
int16_t MPU6050::getRawAccelX() { return getAccelerationX(); }
int16_t MPU6050::getRawAccelY() { return getAccelerationY(); }
int16_t MPU6050::getRawAccelZ() { return getAccelerationZ(); }

// converts raw accelerometer value to m/s² 
double MPU6050::rawToMps2(int16_t rawValue) {
    return (rawValue / 16384.0) * 9.81;
}

// scaled acceleration getters: return acceleration in m/s²
double MPU6050::getAccelXMps2() { return rawToMps2(getRawAccelX()); }
double MPU6050::getAccelYMps2() { return rawToMps2(getRawAccelY()); }
double MPU6050::getAccelZMps2() { return rawToMps2(getRawAccelZ()); }

void MPU6050::updateCounters(double ax, double ay) {
    std::lock_guard<std::mutex> lock(i2cMutex);

    // X-axis counter logic
    if (ax > threshold_pos && !threshold_crossed_x) {
        counter_x += increment_value;
        threshold_crossed_x = true;
    } else if (ax < threshold_neg && !threshold_crossed_x) {
        counter_x -= increment_value;
        threshold_crossed_x = true;
    } else if (ax > threshold_neg && ax < threshold_pos) {
        threshold_crossed_x = false;
    }

    // Y-axis counter logic
    if (ay > threshold_pos && !threshold_crossed_y) {
        counter_y += increment_value;
        threshold_crossed_y = true;
    } else if (ay < threshold_neg && !threshold_crossed_y) {
        counter_y -= increment_value;
        threshold_crossed_y = true;
    } else if (ay > threshold_neg && ay < threshold_pos) {
        threshold_crossed_y = false;
    }
}

void MPU6050::getCounters(int& x, int& y) const {
    std::lock_guard<std::mutex> lock(i2cMutex);
    x = counter_x;
    y = counter_y;
}

void MPU6050::setThresholds(double pos, double neg) {
    std::lock_guard<std::mutex> lock(i2cMutex);
    threshold_pos = pos;
    threshold_neg = neg;
}

void MPU6050::setIncrementValue(int value) {
    std::lock_guard<std::mutex> lock(i2cMutex);
    increment_value = value;
}