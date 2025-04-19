// MPU6050_H

#ifndef MPU6050_H
#define MPU6050_H

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <mutex>
#include <cstdint>
#include <thread>
#include <atomic>

class MPU6050 {
private:
    int file;
    std::mutex i2cMutex;
    std::atomic<bool> running;
    std::thread dataThread;

    void init();
    void writeRegister(uint8_t reg, uint8_t value);
    int16_t readWord(uint8_t reg);
    void collectData();

public:
    MPU6050();
    ~MPU6050();

    void start();
    void stop();
    int16_t getAccelerationX();
    int16_t getAccelerationY();
    int16_t getAccelerationZ();
    int16_t getGyroscopeX();
    int16_t getGyroscopeY();
    int16_t getGyroscopeZ();
};

#endif