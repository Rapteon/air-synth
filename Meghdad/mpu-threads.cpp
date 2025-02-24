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

// Callback Interface for Calibration Completion
struct CalibrationCompleteCallbackInterface {
    virtual void onCalibrationComplete(int16_t axBias, int16_t ayBias, int16_t azBias,
                                       int16_t gxBias, int16_t gyBias, int16_t gzBias) = 0;
};

								


// Class - Calibration Handler
class CalibrationHandler : public CalibrationCompleteCallbackInterface {
public:
    int16_t axBias = 0, ayBias = 0, azBias = 0;
    int16_t gxBias = 0, gyBias = 0, gzBias = 0;

    void onCalibrationComplete(int16_t axBias, int16_t ayBias, int16_t azBias,
                               int16_t gxBias, int16_t gyBias, int16_t gzBias) override {
        this->axBias = axBias;
        this->ayBias = ayBias;
        this->azBias = azBias;
        this->gxBias = gxBias;
        this->gyBias = gyBias;
        this->gzBias = gzBias;
        std::cout << "Calibration biases received." << std::endl;
    }
};

CalibrationHandler calibrationHandler; // Global instance

// IMU Calibration Class - callback driven

class IMUCalibrator {
private:
    int i2c_fd;
    CalibrationCompleteCallbackInterface* callback;

public:
    IMUCalibrator(int fd, CalibrationCompleteCallbackInterface* cb) : i2c_fd(fd), callback(cb) {}

    void calibrate() {
        if (i2c_fd < 0)
			{
            cerr << "Failed to open I2C device for calibration: " << strerror(errno) << endl;
            return;  // in calibration we stop the loop if error 
        }
		
// we can introduce a counter for 3 times in calibration for example:
/* int retryCount = 0;
        bool readSuccess = false;

        while (retryCount < 3 && !readSuccess) {
            if (write(i2c_fd, &reg, 1) != 1) {
                cerr << "I2C write error during calibration: " << strerror(errno) << endl;
                retryCount++;
                continue;
            }

            int bytesRead = read(i2c_fd, data, 14);

            if (bytesRead == 14) {
                readSuccess = true;
            } else {
                cerr << "I2C read error during calibration (retry " << retryCount + 1 << "): " << strerror(errno) << endl;
                retryCount++;
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Add a small delay before retrying
            }
*/




        const int calibrationSamples = 1000;
        int32_t axSum = 0, aySum = 0, azSum = 0;
        int32_t gxSum = 0, gySum = 0, gzSum = 0;

        for (int i = 0; i < calibrationSamples; ++i) {
            uint8_t reg = 0x3B;
            uint8_t data[14];
            write(i2c_fd, &reg, 1);
            int bytesRead = read(i2c_fd, data, 14);

            if (bytesRead != 14) {
                cerr << "I2C read error (calibration): " << strerror(errno) << endl;
                return;
            }

            int16_t ax = (data[0] << 8) | data[1];
            int16_t ay = (data[2] << 8) | data[3];
            int16_t az = (data[4] << 8) | data[5];
            int16_t gx = (data[8] << 8) | data[9];
            int16_t gy = (data[10] << 8) | data[11];
            int16_t gz = (data[12] << 8) | data[13];

            axSum += ax;
            aySum += ay;
            azSum += az;
            gxSum += gx;
            gySum += gy;
            gzSum += gz;

            this_thread::sleep_for(chrono::milliseconds(1));
        }

        if (callback) {
            callback->onCalibrationComplete(axSum / calibrationSamples, aySum / calibrationSamples, azSum / calibrationSamples,
                                            gxSum / calibrationSamples, gySum / calibrationSamples, gzSum / calibrationSamples);
        }
    }
};



	
	
	// IMU Filtering Class (Callback-Driven)

			class IMUFilter

{

				private:

				double samplePeriod;

				int filterOrder;

				float filtCutOff;

				vector<Vector3f> linVel;

				vector<Vector3f> linPos;

				FilteredDataCallbackInterface* callback; // Pointer to callback object



				// Butterworth High-Pass Filter Coefficients

					pair<vector<float>, vector<float>> butterworthHP(float cutoff, double dt, int order) 
					{

						float wc = 2 * M_PI * cutoff;

						float alpha = wc * dt / (wc * dt + 1);

						vector<float> b = {1.0f - alpha, alpha - 1.0f}; // Numerator coefficients

						vector<float> a = {1.0f, alpha - 1.0f}; // Denominator coefficients

						return {b, a};

					}



				// Apply Butterworth High-Pass Filter
	
					vector<Vector3f> applyHighPassFilter(const vector<Vector3f>& data, float cutoff, double dt, int order) 
					{

						if (data.size() < 2) return data;

						auto [b, a] = butterworthHP(cutoff, dt, order);

						vector<Vector3f> filteredData(data.size(), Vector3f::Zero());

						filteredData[0] = data[0]; // Initialize with first value

						for (size_t i = 1; i < data.size(); ++i) 
						{

							filteredData[i] = b[0] * data[i] + b[1] * data[i - 1] - a[1] * filteredData[i - 1];

						}

						return filteredData;

					}



				public:

				IMUFilter(double samplePeriod, float cutoff, int order, FilteredDataCallbackInterface* cb)

				: samplePeriod(samplePeriod), filtCutOff(cutoff), filterOrder(order), callback(cb) 
				{

					linVel.reserve(1000); // Pre-allocate space for 1000 velocity samples

					linPos.reserve(1000); // Pre-allocate space for 1000 position samples

					linVel.push_back(Vector3f::Zero());

					linPos.push_back(Vector3f::Zero());

				}



					void update(const Vector3f& linAcc) 
					{

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

							if (callback) 
							{

								callback->onFilteredData(linVel.back(), linPos.back());

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

    IMUCalibrator calibrator(i2c_fd, &calibrationHandler);
    calibrator.calibrate();

    while (true) {
        uint8_t reg = 0x3B;
        uint8_t data[14];
        write(i2c_fd, &reg, 1);
        int bytesRead = read(i2c_fd, data, 14);
        if (bytesRead != 14) {
            cerr << "I2C read error: " << strerror(errno) << endl;
            continue;
        }

        // Swap buffers safely
        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            std::swap(writeBuffer, readBuffer);
            writeBuffer->ax = (data[0] << 8) | data[1] - calibrationHandler.axBias;
            writeBuffer->ay = (data[2] << 8) | data[3] - calibrationHandler.ayBias;
            writeBuffer->az = (data[4] << 8) | data[5] - calibrationHandler.azBias;
            writeBuffer->gx = (data[8] << 8) | data[9] - calibrationHandler.gxBias;
            writeBuffer->gy = (data[10] << 8) | data[11] - calibrationHandler.gyBias;
            writeBuffer->gz = (data[12] << 8) | data[13] - calibrationHandler.gzBias;
            newDataAvailable = true;
        }

        // Notify all processing threads
        dataReady.notify_all();
        this_thread::sleep_for(chrono::milliseconds(3906)); // Simulate sensor read delay-Approximately 256 Hz
    }

		 // Future: Interrupt-driven data retrieval
		 // ...
		
		
    close(i2c_fd);
}

       
		
		
// Thread 2: Process IMU Data (MahonyAHRS, Filtering)
void processData() {
    MahonyAHRS ahrs(1.0f, 0.0f);
    ahrs.SamplePeriod = samplePeriod; // Set the sample period
    
    // Callback class for receiving filtered data
    struct LocalFilteredDataCallback : public FilteredDataCallbackInterface {
        void onFilteredData(const Vector3f& linVel, const Vector3f& linPos) override {
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                sharedData.linPos = linPos;  // Store position in sharedData
            }

            // Print linear velocity and position
            std::cout << "LinVel: [" << linVel.x() << ", " << linVel.y() << ", " << linVel.z() << "] ";
            std::cout << "LinPos: [" << linPos.x() << ", " << linPos.y() << ", " << linPos.z() << "]" << std::endl;
        }
    };

    LocalFilteredDataCallback localCallback;
    IMUFilter imuFilter(samplePeriod, filtCutOff, 1, &localCallback);

    while (true) {
        // 1. Wait for new data
        std::unique_lock<std::mutex> lock(bufferMutex);
        dataReady.wait(lock, [] { return newDataAvailable; });

        // 2. Swap buffers
        IMUData* localData = readBuffer;
        newDataAvailable = false;
        lock.unlock();  // Unlock early  to avoid blocking other threads

        // 3. Convert raw IMU data to floating-point values
        float gxFloat = (localData->gx) * (M_PI / 180.0f);
        float gyFloat = (localData->gy) * (M_PI / 180.0f);
        float gzFloat = (localData->gz) * (M_PI / 180.0f);
        float axFloat = localData->ax;
        float ayFloat = localData->ay;
        float azFloat = localData->az;

        // 4. Pass data to MahonyAHRS for orientation estimation
        ahrs.UpdateIMU(gxFloat, gyFloat, gzFloat, axFloat, ayFloat, azFloat);

        // 5. Compute linear acceleration
        Vector3f g(0, 0, -9.81);
        Matrix3f R = ahrs.getRotationMatrix();
        Vector3f linAcc = R * Vector3f(axFloat, ayFloat, azFloat) - g;

        // 6. Update filter (this will trigger the callback)
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