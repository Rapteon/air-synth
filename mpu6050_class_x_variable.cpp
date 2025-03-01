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

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

#define I2C_BUS "/dev/i2c-1"
#define ACCEL_SCALE 16384.0
#define G 9.81
#define ACCEL_THRESHOLD 5.0
#define INCREMENT_VALUE 5

using namespace std;

class MPU6050 {
private:
    int file;
    mutex mtx;
    atomic<bool> running;
    int counter;
    bool threshold_crossed;
    
public:
    MPU6050() : running(true), counter(0), threshold_crossed(false) {
        signal(SIGINT, signal_handler);
        if ((file = open(I2C_BUS, O_RDWR)) < 0) {
            cerr << "Failed to open I2C bus" << endl;
           // exit(1);
        }
        if (ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
            cerr << "Failed to acquire bus access" << endl;
        //    close(file);
         //   exit(1);
        }
        uint8_t config[2] = {PWR_MGMT_1, 0x00};
        if (write(file, config, 2) != 2) {
            cerr << "Failed to initialize MPU6050" << endl;
           // close(file);
           // exit(1);
        }
        config[0] = ACCEL_CONFIG;
        config[1] = 0x00;
        if (write(file, config, 2) != 2) {
            cerr << "Failed to configure accelerometer" << endl;
           // close(file);
           // exit(1);
        }
    }
    
    ~MPU6050() {
        close(file);
        cout << "I2C file closed. Exiting program." << endl;
    }
    
    static void signal_handler(int signum) {
        cout << "\nTerminating program safely...\n";
        exit(0);
    }
    
    int16_t read_word(uint8_t reg) {
        uint8_t buffer[2];
        if (write(file, &reg, 1) != 1) {
            cerr << "Failed to write register address" << endl;
           // exit(1);
        }
        if (read(file, buffer, 2) != 2) {
            cerr << "Failed to read data" << endl;
           // exit(1);
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
                if (ax > ACCEL_THRESHOLD && !threshold_crossed) {
                    counter += INCREMENT_VALUE;
                    threshold_crossed = true;
                } else if (ax < ACCEL_THRESHOLD) {
                    threshold_crossed = false;
                }
                if (fabs(ax - last_ax) > 0.1) {
                    cout << "Acceleration: " << ax << " m/s² | Counter: " << counter << endl;
                    last_ax = ax;
                }
            }
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
    
    void start_monitoring() {
        thread monitor_thread(&MPU6050::monitor_threshold, this);
        monitor_thread.join();
    }
};

int main() {
    MPU6050 sensor;
    sensor.start_monitoring();
    return 0;
}
