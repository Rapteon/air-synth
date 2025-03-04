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
#include <gpiod.h>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

#define I2C_BUS "/dev/i2c-1"
#define ACCEL_SCALE 16384.0
#define G 9.81
#define ACCEL_THRESHOLD 5.0
#define INCREMENT_VALUE 5
#define BUTTON_PIN 4 // GPIO pin for the push button

using namespace std;

class MPU6050 {
private:
    int file;
    mutex mtx;
    atomic<bool> running;
    int counter;
    bool threshold_crossed;
    atomic<bool> active;
    
public:
    MPU6050() : running(true), counter(0), threshold_crossed(false), active(false) {
        signal(SIGINT, signal_handler);
        if ((file = open(I2C_BUS, O_RDWR)) < 0) {
            cerr << "Failed to open I2C bus" << endl;
        }
        if (ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
            cerr << "Failed to acquire bus access" << endl;
        }
        uint8_t config[2] = {PWR_MGMT_1, 0x00};
        if (write(file, config, 2) != 2) {
            cerr << "Failed to initialize MPU6050" << endl;
        }
        config[0] = ACCEL_CONFIG;
        config[1] = 0x00;
        if (write(file, config, 2) != 2) {
            cerr << "Failed to configure accelerometer" << endl;
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
        }
        if (read(file, buffer, 2) != 2) {
            cerr << "Failed to read data" << endl;
        }
        return (buffer[0] << 8) | buffer[1];
    }
    
    void monitor_threshold() {
        double last_ax = 0.0;
        while (running) {
            if (!active) {
                this_thread::sleep_for(chrono::milliseconds(100));
                continue;
            }

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
        monitor_thread.detach();
    }
    
    void set_active(bool state) {
        active = state;
    }
};

void button_interrupt() {
    gpiod_chip *chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        cerr << "Failed to open GPIO chip" << endl;
        return;
    }
    gpiod_line *line = gpiod_chip_get_line(chip, BUTTON_PIN);
    if (!line) {
        cerr << "Failed to get GPIO line" << endl;
        gpiod_chip_close(chip);
        return;
    }
    if (gpiod_line_request_falling_edge_events(line, "button_listener") < 0) {
        cerr << "Failed to set line request" << endl;
        gpiod_chip_close(chip);
        return;
    }
    MPU6050 sensor;
    sensor.start_monitoring();
    while (true) {
        gpiod_line_event event;
        if (gpiod_line_event_wait(line, NULL) > 0 && gpiod_line_event_read(line, &event) == 0) {
            cout << "Button pressed. Starting MPU6050 monitoring..." << endl;
            sensor.set_active(true);
        }
    }
    gpiod_chip_close(chip);
}

int main() {
    thread button_thread(button_interrupt);
    button_thread.join();
    return 0;
}
