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

#define MPU6050_ADDR 0x68 // 0x72 other address.
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B   //Obtained from Datasheet
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F

#define I2C_BUS "/dev/i2c-1" // Specify the I2C bus
#define ACCEL_SCALE 16384.0   // Scale factor for ±2g range
#define G 9.81                // Gravity in m/s²
#define ACCEL_THRESHOLD_POS 5.0   // Acceleration threshold in m/s²
#define ACCEL_THRESHOLD_NEG -5.0 //Negative acceleration threshold in m/s2
#define INCREMENT_VALUE 5     // Value to increment when threshold is crossed
#define DECREMENT_VALUE 5     // Value to decrement when threshold is crossed

#define GPIO_CHIP "gpiochip0"
#define BUTTON_GPIO 4

using namespace std;

class MPU6050 {
private:
    int file;
    std::mutex mtx;
    std::atomic<bool> running;
    std::atomic<bool> active; // Indicates if MPU6050 is reading data or not
    int counter_x, counter_y;
    bool threshold_crossed_x, threshold_crossed_y;

public:
    MPU6050() : running(true), active(false), counter_x(0), counter_y(0), threshold_crossed_x(false), threshold_crossed_y(false) {
        signal(SIGINT, signal_handler);
        if ((file = open(I2C_BUS, O_RDWR)) < 0) {
            std::cerr << "Failed to open I2C bus" << std::endl;
        }

        if (ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
            std::cerr << "Failed to initialise MPU6050" << std::endl;
        }

        uint8_t config[2] = {PWR_MGMT_1, 0x00};
        if (write(file, config, 2) != 2) {
            std::cerr << "Failed to initialise MPU6050" << std::endl;
        }
    }

    ~MPU6050() {
        close(file);
        std::cout << "I2C Closed. Exiting Program." << std::endl;
    }

    static void signal_handler(int signum) {
        std::cout << "\nTerminating Program Safely...\n";
        exit(0);
    }

    int16_t read_word(int8_t reg) {
        uint8_t buffer[2];
        if (write(file, &reg, 1) != 1) {
            std::cerr << "Failed to write register address" << std::endl;
        }

        if (read(file, buffer, 2) != 2) {
            std::cerr << "Failed to read data" << endl;
        }
        return (buffer[0] << 8) | buffer[1];
    }

    void monitor_threshold() {
        double last_ax = 0.0, last_ay = 0.0;
        while (running) {
            if (!active) {
                this_thread::sleep_for(chrono::milliseconds(100)); // Sleep if button is not pressed
                continue;
            }

            int16_t raw_x = read_word(ACCEL_XOUT_H);
            int16_t raw_y = read_word(ACCEL_YOUT_H);
            double ax = (raw_x / ACCEL_SCALE) * G;
            double ay = (raw_y / ACCEL_SCALE) * G;

            {
                lock_guard<std::mutex> lock(mtx);
                // X-axis acceleration processing
                if (ax > ACCEL_THRESHOLD_POS && !threshold_crossed_x) {
                    counter_x += INCREMENT_VALUE;
                    threshold_crossed_x = true;
                } else if ((ax < ACCEL_THRESHOLD_POS) && (ax > ACCEL_THRESHOLD_NEG)) {
                    threshold_crossed_x = false;
                } else if (ax < ACCEL_THRESHOLD_NEG && !threshold_crossed_x) {
                    counter_x -= INCREMENT_VALUE;
                    threshold_crossed_x = true;
                }

                // Y-axis acceleration processing
                if (ay > ACCEL_THRESHOLD_POS && !threshold_crossed_y) {
                    counter_y += INCREMENT_VALUE;
                    threshold_crossed_y = true;
                } else if ((ay < ACCEL_THRESHOLD_POS) && (ay > ACCEL_THRESHOLD_NEG)) {
                    threshold_crossed_y = false;
                } else if (ay < ACCEL_THRESHOLD_NEG && !threshold_crossed_y) {
                    counter_y -= INCREMENT_VALUE;
                    threshold_crossed_y = true;
                }

                if (fabs(ax - last_ax) > 0.1 || fabs(ay - last_ay) > 0.1) {
                    std::cout << "Acceleration X: " << ax << " m/s2 | Counter X: " << counter_x 
                              << " | Acceleration Y: " << ay << " m/s2 | Counter Y: " << counter_y << std::endl;
                    last_ax = ax;
                    last_ay = ay;
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

// Function to handle GPIO interrupts (Button connected to GND)
void button_interrupt(MPU6050& sensor) {
    struct gpiod_chip* chip = gpiod_chip_open_by_name(GPIO_CHIP);
    if (!chip) {
        cerr << "Error: Failed to open GPIO chip. Check permissions or hardware connection." << endl;
        return;
    }

    struct gpiod_line* line = gpiod_chip_get_line(chip, BUTTON_GPIO);
    if (!line) {
        cerr << "Error: Failed to get GPIO line." << endl;
        gpiod_chip_close(chip);
        return;
    }

    struct gpiod_line_request_config config = {};
    config.consumer = "button_monitor";
    config.request_type = GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE;
    config.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;

    if (gpiod_line_request(line, &config, 0) < 0) {
        cerr << "Error: Failed to configure GPIO with pull-up resistor." << endl;
        gpiod_chip_close(chip);
        return;
    }

    cout << "Waiting for button press..." << endl;

    while (true) {
        struct gpiod_line_event event;
        int ret = gpiod_line_event_wait(line, nullptr);  // Wait indefinitely for button press

        if (ret > 0 && gpiod_line_event_read(line, &event) == 0) {
            if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
                cout << "Button pressed! Starting MPU6050 data collection..." << endl;
                sensor.set_active(true);

                // Wait until button is released
                while (gpiod_line_get_value(line) == 0) {
                    this_thread::sleep_for(chrono::milliseconds(50));  // Debounce delay
                }

                cout << "Button released! Stopping MPU6050 data collection..." << endl;
                sensor.set_active(false);
            }
        }
    }

    gpiod_chip_close(chip);
}

int main() {
    MPU6050 sensor;
    sensor.start_monitoring();

    thread gpio_thread(button_interrupt, ref(sensor));
    gpio_thread.detach();

    while (true) {
        this_thread::sleep_for(chrono::milliseconds(1000));
    }
    return 0;
}