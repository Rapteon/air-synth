#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <mutex>
#include <atomic> // Include atomic header for calibration thread

// main function
/*
int main() {
    std::thread calibration(calibrationThread);
    std::thread readData(readMPUData);
    std::thread processData(processData);
    std::thread gesture(gestureRecognition);
    std::thread encapsulation(dataEncapsulation);

    calibration.join();
    readData.join();
    processData.join();
    gesture.join();
    encapsulation.join();

    return 0;
}
*/

std::atomic<bool> calibrationDone(false); // Global flag

// Calibration Thread 
void calibrationThread() {
    int i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "Failed to open I2C device for calibration: " << strerror(errno) << std::endl;
        return;
    }
    ioctl(i2c_fd, I2C_SLAVE, 0x68);

    IMUCalibrator calibrator(i2c_fd, &calibrationHandler);
    calibrator.calibrate();

    close(i2c_fd);
    calibrationDone.store(true); // Set calibration flag
    dataReady.notify_all(); // Notify threads that calibration is complete
}




// ... (Other includes and declarations: bufferMutex, writeBuffer, readBuffer, newDataAvailable, dataReady, IMUData) ...

void configureMPU6050FIFO(int i2c_fd) {
    uint8_t configValue = 0x03;
    uint8_t configReg = 0x1A;
    write(i2c_fd, &configReg, 1);
    write(i2c_fd, &configValue, 1);

    uint8_t smplrtDivValue = 0x03;
    uint8_t smplrtDivReg = 0x19;
    write(i2c_fd, &smplrtDivReg, 1);
    write(i2c_fd, &smplrtDivValue, 1);

    uint8_t intEnableValue = 0x10;
    uint8_t intEnableReg = 0x38;
    write(i2c_fd, &intEnableReg, 1);
    write(i2c_fd, &intEnableValue, 1);

    uint8_t fifoEnValue = 0x78;
    uint8_t fifoEnReg = 0x23;
    write(i2c_fd, &fifoEnReg, 1);
    write(i2c_fd, &fifoEnValue, 1);

    uint8_t userCtrlValue = 0x40;
    uint8_t userCtrlReg = 0x6A;
    write(i2c_fd, &userCtrlReg, 1);
    write(i2c_fd, &userCtrlValue, 1);

    userCtrlValue = 0x44;
    write(i2c_fd, &userCtrlReg, 1);
    write(i2c_fd, &userCtrlValue, 1);
}

bool validateData(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    // Example validation: Check if sensor data is within reasonable ranges
    if (ax > 32767 || ax < -32768 || ay > 32767 || ay < -32768 || az > 32767 || az < -32768 ||
        gx > 32767 || gx < -32768 || gy > 32767 || gy < -32768 || gz > 32767 || gz < -32768) {
        std::cerr << "Warning: Sensor data out of range: ax=" << ax << ", ay=" << ay << ", az=" << az
                  << ", gx=" << gx << ", gy=" << gy << ", gz=" << gz << std::endl;
        return false; // Data invalid
    }
    return true; // Data valid
}

void readMPUData() {
    int i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "Failed to open I2C device: " << strerror(errno) << std::endl;
        return;
    }
    ioctl(i2c_fd, I2C_SLAVE, 0x68);

    configureMPU6050FIFO(i2c_fd);

    while (true) {
        uint8_t fifoCountReg = 0x72;
        uint8_t fifoCountData[2] = {0};
        write(i2c_fd, &fifoCountReg, 1);

        if (read(i2c_fd, fifoCountData, 2) != 2) {
            std::cerr << "Error reading FIFO count: " << strerror(errno) << std::endl;
            continue;
        }

        int fifoCount = (fifoCountData[0] << 8) | fifoCountData[1];

        if (fifoCount > 900) {
            std::cerr << " FIFO near overflow: " << fifoCount << " bytes. Flushing data..." << std::endl;
            uint8_t flushReg = 0x74;
            write(i2c_fd, &flushReg, 1);
            std::vector<uint8_t> flushBuffer(fifoCount);
            read(i2c_fd, flushBuffer.data(), fifoCount);

            uint8_t userCtrlReg = 0x6A;
            uint8_t userCtrlResetValue = 0x44;
            write(i2c_fd, &userCtrlReg, 1);
            write(i2c_fd, &userCtrlResetValue, 1);

            continue;
        }

        if (fifoCount >= 12) {
            int samples = fifoCount / 12;
            std::vector<uint8_t> fifoData(samples * 12);

            uint8_t fifoRWReg = 0x74;
            write(i2c_fd, &fifoRWReg, 1);
            if (read(i2c_fd, fifoData.data(), fifoCount) != fifoCount) {
                std::cerr << "Error reading FIFO data: " << strerror(errno) << std::endl;
                continue;
            }

            for (int i = 0; i < samples; i++) {
                int index = i * 12;

                int16_t ax = (fifoData[index] << 8) | fifoData[index + 1];
                int16_t ay = (fifoData[index + 2] << 8) | fifoData[index + 3];
                int16_t az = (fifoData[index + 4] << 8) | fifoData[index + 5];
                int16_t gx = (fifoData[index + 6] << 8) | fifoData[index + 7];
                int16_t gy = (fifoData[index + 8] << 8) | fifoData[index + 9];
                int16_t gz = (fifoData[index + 10] << 8) | fifoData[index + 11];

                if (validateData(ax, ay, az, gx, gy, gz)) {
                    {
                        std::lock_guard<std::mutex> lock(bufferMutex);
                        std::swap(writeBuffer, readBuffer);
                        writeBuffer->ax = ax;
                        writeBuffer->ay = ay;
                        writeBuffer->az = az;
                        writeBuffer->gx = gx;
                        writeBuffer->gy = gy;
                        writeBuffer->gz = gz;
                        newDataAvailable = true;
                    }
                }
            }
            dataReady.notify_all();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    close(i2c_fd);
}