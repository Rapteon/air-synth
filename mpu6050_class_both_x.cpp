#include <iostream>
#include <fstream>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal>

#define MPU6050_ADDR 0x68 // 0x72 other address.
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

#define I2C_BUS "/dev/i2c-1" // Specify the I2C bus
#define ACCEL_SCALE 16384.0   // Scale factor for ±2g range
#define G 9.81                // Gravity in m/s²
#define ACCEL_THRESHOLD 3.0   // Acceleration threshold in m/s²
#define INCREMENT_VALUE 5     // Value to increment when threshold is crossed
#define DECREMENT_VALUE 5     // Value to decrement when threshold is crossed

using namespace std;

mutex mtx;
atomic<bool> running(true); // Atomic flag to control the loop

class MPU6050 {
private:
    int file;
    int counter;
    bool positive_threshold_crossed;
    bool negative_threshold_crossed;
    
public:
    MPU6050() : counter(0), positive_threshold_crossed(false), negative_threshold_crossed(false) {
        if ((file = open(I2C_BUS, O_RDWR)) < 0) {
            cerr << "Failed to open I2C bus" << endl;
            exit(1);
        }

        if (ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
            cerr << "Failed to acquire bus access" << endl;
            close(file);
            exit(1);
        }

        initialize();
    }

    ~MPU6050() {
        close(file);
        cout << "I2C file closed. Exiting program.\n";
    }

    void initialize() {
        uint8_t config[2];

        config[0] = PWR_MGMT_1;
        config[1] = 0x00;
        if (write(file, config, 2) != 2) {
            cerr << "Failed to initialize MPU6050" << endl;
            close(file);
            exit(1);
        }

        config[0] = ACCEL_CONFIG;
        config[1] = 0x00;
        if (write(file, config, 2) != 2) {
            cerr << "Failed to configure accelerometer" << endl;
            close(file);
            exit(1);
        }
    }

    int16_t read_word(uint8_t reg) {
        uint8_t buffer[2];
        if (write(file, &reg, 1) != 1) {
            cerr << "Failed to write register address" << endl;
            close(file);
            exit(1);
        }
        if (read(file, buffer, 2) != 2) {
            cerr << "Failed to read data" << endl;
            close(file);
            exit(1);
        }
        return (buffer[0] << 8) | buffer[1];
    }

    void monitor_threshold() {
        double last_ax = 0.0;
        while (running) {
            int16_t raw_x = read_word(ACCEL_XOUT_H);
            double ax = (raw_x / ACCEL_SCALE) * G;

            {
                lock_guard<mutex> lock(mtx);
                if (ax > ACCEL_THRESHOLD && !positive_threshold_crossed) {
                    counter += INCREMENT_VALUE;
                    positive_threshold_crossed = true;
                } else if (ax < ACCEL_THRESHOLD) {
                    positive_threshold_crossed = false;
                }

                if (ax < -ACCEL_THRESHOLD && !negative_threshold_crossed) {
                    counter -= DECREMENT_VALUE;
                    negative_threshold_crossed = true;
                } else if (ax > -ACCEL_THRESHOLD) {
                    negative_threshold_crossed = false;
                }

                if (fabs(ax - last_ax) > 0.1) {
                    cout << "Acceleration: " << ax << " m/s² | Counter: " << counter << endl;
                    last_ax = ax;
                }
            }
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
};

void signal_handler(int signum) {
    cout << "\nTerminating program safely...\n";
    running = false;
}

int main() {
    signal(SIGINT, signal_handler);
    MPU6050 sensor;
    thread monitor_thread(&MPU6050::monitor_threshold, &sensor);
    monitor_thread.join();
    return 0;
}
