#ifndef MPU6050_H
#define MPU6050_H

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <mutex>
#include <cstdint>
#include <atomic>

class MPU6050 {
private:
    int file;
	mutable std::mutex i2cMutex; 
    int counter_x;
    int counter_y;
    bool threshold_crossed_x;
    bool threshold_crossed_y;
    double threshold_pos;
    double threshold_neg;
    int increment_value;

    void init();
    void writeRegister(uint8_t reg, uint8_t value);
    int16_t readWord(uint8_t reg); // high and low bytes

public:
    MPU6050();
    ~MPU6050();

    // motion interrupt control
    void enableMotionInterrupt();
    void disableMotionInterrupt();

    // raw data
    int16_t getAccelerationX();
    int16_t getAccelerationY();
    int16_t getAccelerationZ();
    int16_t getGyroscopeX();
    int16_t getGyroscopeY();
    int16_t getGyroscopeZ();

    // getters for scaled acceleration in m/s²
    int16_t getRawAccelX();
    int16_t getRawAccelY();
    int16_t getRawAccelZ();
    double rawToMps2(int16_t rawValue);
    double getAccelXMps2();
    double getAccelYMps2();
    double getAccelZMps2();

    // counter control and retrieval
    void updateCounters(double ax, double ay);
    void getCounters(int& x, int& y) const;
    void setThresholds(double pos, double neg);
    void setIncrementValue(int value);
};

#endif