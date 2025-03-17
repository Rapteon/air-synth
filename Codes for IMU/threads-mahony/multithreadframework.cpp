#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <mutex>
#include <condition_variable>
#include <cstring> // For strerror
#include <errno.h> // For errno
#include "MahonyAHRS.h"

using namespace std;
using namespace Eigen;

// Configuration
constexpr double samplePeriod = 1.0 / 256.0;
constexpr float filtCutOff = 0.1;

struct IMUData {
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;
    Vector3f linPos = Vector3f::Zero();
};

// Global Shared Data and Synchronization - Use two buffers and a flag
std::condition_variable dataReady;



//  two buffers and a flag
std::mutex dataMutex;
IMUData sharedData;

std::mutex bufferMutex;
IMUData buffer1, buffer2;
IMUData* writeBuffer = &buffer1;
IMUData* readBuffer = &buffer2;

bool newDataAvailable = false;

// Callback Interface for Filtered Data
struct FilteredDataCallbackInterface {
    virtual void onFilteredData(const Vector3f& linVel, const Vector3f& linPos) = 0;
};



// IMU Filtering&dataprocessing Class (Callback-Driven)
class IMUFilter {
private:
    double samplePeriod;
    int filterOrder;
    float filtCutOff;
    vector<Vector3f> linVel;
    vector<Vector3f> linPos;
    FilteredDataCallbackInterface* callback; // Pointer to callback object

    // Butterworth High-Pass Filter Coefficients
    pair<vector<float>, vector<float>> butterworthHP(float cutoff, double dt, int order) {
        float wc = 2 * M_PI * cutoff;
        float alpha = wc * dt / (wc * dt + 1);
        vector<float> b = {1.0f - alpha, alpha - 1.0f}; // Numerator coefficients
        vector<float> a = {1.0f, alpha - 1.0f}; // Denominator coefficients
        return {b, a};
    }


    // Apply Butterworth High-Pass Filter
    vector<Vector3f> applyHighPassFilter(const vector<Vector3f>& data, float cutoff, double dt, int order) {
        if (data.size() < 2) return data;
        auto [b, a] = butterworthHP(cutoff, dt, order);
        vector<Vector3f> filteredData(data.size(), Vector3f::Zero());
        filteredData[0] = data[0]; // Initialize with first value
        for (size_t i = 1; i < data.size(); ++i) {
            filteredData[i] = b[0] * data[i] + b[1] * data[i - 1] - a[1] * filteredData[i - 1];
        }
        return filteredData;
    }

public:
    IMUFilter(double samplePeriod, float cutoff, int order, FilteredDataCallbackInterface* cb)
        : samplePeriod(samplePeriod), filtCutOff(cutoff), filterOrder(order), callback(cb) {
        linVel.reserve(1000); // Pre-allocate space for 1000 velocity samples
        linPos.reserve(1000); // Pre-allocate space for 1000 position samples
        linVel.push_back(Vector3f::Zero());
        linPos.push_back(Vector3f::Zero());
    }

    void update(const Vector3f& linAcc) {
        // Compute Linear Velocity (Integration of Acceleration)
        linVel.push_back(linVel.back() + linAcc * samplePeriod);

        // Apply High-Pass Filter to Velocity
        linVel = applyHighPassFilter(linVel, filtCutOff, samplePeriod, filterOrder);

        // Compute Linear Position (Integration of Velocity)
        linPos.push_back(linPos.back() + linVel.back() * samplePeriod);

        // Apply High-Pass Filter to Position
        linPos = applyHighPassFilter(linPos, filtCutOff, samplePeriod, filterOrder);

        // Limit Vector Size to Prevent Memory Issues
        const size_t maxSize = 1000; // Keep only the last 1000 samples
        if (linVel.size() > maxSize) linVel.erase(linVel.begin());
        if (linPos.size() > maxSize) linPos.erase(linPos.begin());


         // Callback for Event Handling
    if (callback) {
        // Multiply by a scaling factor (e.g., 1000) to get larger values
        Vector3f scaledLinVel = linVel.back() * 1000.0f;
        Vector3f scaledLinPos = linPos.back() * 1000.0f;

        callback->onFilteredData(scaledLinVel, scaledLinPos);
        }
    }
};


// Thread 1: Read MPU6050 Data
void readMPUData() {
    int i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0) {
        cerr << "Failed to open I2C device: " << strerror(errno) << endl;
        return;
    }
    ioctl(i2c_fd, I2C_SLAVE, 0x68);

    // Wake up MPU6050
    #define PWR_MGMT_1 0x6B
    uint8_t wakeUpData[2];
    wakeUpData[0] = PWR_MGMT_1;
    wakeUpData[1] = 0x00; // Set to 0 to wake up the sensor
    if (write(i2c_fd, wakeUpData, 2) != 2) {
        std::cerr << "Failed to wake up MPU6050: " << strerror(errno) << std::endl;
        close(i2c_fd);
        return; // Corrected
    }

    // Set Accelerometer Scale to ±4g
    uint8_t accelConfigReg = 0x1C;
    uint8_t accelConfigValue = 0x08; // 00001000 in binary, setting AFS_SEL = 1 (±4g)
    write(i2c_fd, &accelConfigReg, 1);
    write(i2c_fd, &accelConfigValue, 1);

    while (true) {
        uint8_t reg = 0x3B;
        uint8_t data[14];
        write(i2c_fd, &reg, 1);
        int bytesRead = read(i2c_fd, data, 14);
        if (bytesRead != 14) {
            cerr << "I2C read error: " << strerror(errno) << endl;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            std::swap(writeBuffer, readBuffer);
            writeBuffer->ax = (data[0] << 8) | data[1];
            writeBuffer->ay = (data[2] << 8) | data[3];
            writeBuffer->az = (data[4] << 8) | data[5];
            writeBuffer->gx = (data[8] << 8) | data[9];
            writeBuffer->gy = (data[10] << 8) | data[11];
            writeBuffer->gz = (data[12] << 8) | data[13];
            newDataAvailable = true;
        }

        dataReady.notify_all();
        this_thread::sleep_for(chrono::microseconds(3906));
    }

    close(i2c_fd);
}


// Thread 2: Process IMU Data (MahonyAHRS, Filtering)
void processData() {
    MahonyAHRS ahrs(1.0f, 0.0f);
    ahrs.samplePeriod = samplePeriod; // Set the sample period

// Callback class for receiving filtered data

    struct LocalFilteredDataCallback : public FilteredDataCallbackInterface {
        void onFilteredData(const Vector3f& linVel, const Vector3f& linPos) override {
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                sharedData.linPos = linPos; // Store position in sharedData
            }

                        // Print linear velocity and position
        std::cout << "LinVel: [" << linVel.x() << ", " << linVel.y() << ", " << linVel.z() << "] ";
        std::cout << "LinPos: [" << linPos.x() << ", " << linPos.y() << ", " << linPos.z() << "]" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(250)); // Sleep for 250ms

        }
    };

    LocalFilteredDataCallback localCallback;
    IMUFilter imuFilter(samplePeriod, filtCutOff, 1, &localCallback);

    while (true) {
        std::unique_lock<std::mutex> lock(bufferMutex);
        dataReady.wait(lock, [] { return newDataAvailable; });

        IMUData* localData = readBuffer;
        newDataAvailable = false;
        lock.unlock();


        // Scale accelerometer data to m/s²
        float axFloat = localData->ax * (4.0f * 9.81f) / 8192.0f;
        float ayFloat = localData->ay * (4.0f * 9.81f) / 8192.0f;
        float azFloat = localData->az * (4.0f * 9.81f) / 8192.0f;

        // Scale gyroscope data to radians/second
        float gxFloat = localData->gx / 131.0f * (M_PI / 180.0f);
        float gyFloat = localData->gy / 131.0f * (M_PI / 180.0f);
        float gzFloat = localData->gz / 131.0f * (M_PI / 180.0f);

        ahrs.UpdateIMU(gxFloat, gyFloat, gzFloat, axFloat, ayFloat, azFloat);

        Vector3f g(0, 0, -9.81);
        Matrix3f R = ahrs.getRotationMatrix();
        Vector3f linAcc = R * Vector3f(axFloat, ayFloat, azFloat) - g;

        imuFilter.update(linAcc);
    }
}

// Thread 3: Gesture Recognition (Future)
void gestureRecognition() {
    while (true) {
        // Future: Implement gesture recognition logic here
        // ...
        // If a gesture is recognized, trigger a callback or setter
        this_thread::sleep_for(chrono::milliseconds(100)); // Example delay
    }
}

// Thread 4: Data Encapsulation/Conversion (Future)
void dataEncapsulation() {
    while (true) {
        // Future: Implement data encapsulation or conversion here
        // ...
        // Access sharedData.linPos to get position data
        this_thread::sleep_for(chrono::milliseconds(50)); // Example delay
    }
}

int main() {
    thread t1(readMPUData);
    thread t2(processData);
    thread t3(gestureRecognition);
    thread t4(dataEncapsulation);

    t1.join();
    t2.join();
    t3.join();
    t4.join();

    return 0;
}
