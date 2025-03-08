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
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F

#define I2C_BUS "/dev/i2c-1"
#define ACCEL_SCALE 16384.0
#define G 9.81
#define ACCEL_THRESHOLD_POS 5.0
#define ACCEL_THRESHOLD_NEG -5.0
#define ACCEL_THRESHOLD_POS_Z 3.0
#define ACCEL_THRESHOLD_NEG_Z -3.0
#define INCREMENT_VALUE 5
#define DECREMENT_VALUE 5

#define GPIO_CHIP "gpiochip0"
#define BUTTON_GPIO 4

using namespace std;

class MPU6050 {
private:
    int file;
    std::mutex mtx;
    std::atomic<bool> running;
    std::atomic<bool> active;
    int counter_x, counter_y, counter_z;
    bool threshold_crossed_x, threshold_crossed_y, threshold_crossed_z;
    double z_offset;

public:
    MPU6050() : running(true), active(false), counter_x(0), counter_y(0), counter_z(0), 
                threshold_crossed_x(false), threshold_crossed_y(false), threshold_crossed_z(false) {
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
            std::cerr << "Failed to read data" << std::endl;
        }
        return (buffer[0] << 8) | buffer[1];
    }

    void calibrate_z_axis() {
        constexpr int CALIBRATION_SAMPLES = 100;
        double sum_az = 0.0;
    
        std::cout << "Calibrating Z-axis... Please keep the sensor stationary." << std::endl;
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            int16_t raw_z = read_word(ACCEL_ZOUT_H);
            double az = (raw_z / ACCEL_SCALE) * G;
            sum_az += az;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    
        z_offset = sum_az / CALIBRATION_SAMPLES;  // Compute the average offset
        std::cout << "Z-axis calibration completed. Offset: " << z_offset << " m/s²" << std::endl;
    }
    

    void monitor_threshold() {
        double last_ax = 0.0, last_ay = 0.0, last_az = 0.0;
        while (running) {
            if (!active) {
                this_thread::sleep_for(chrono::milliseconds(100));
                continue;
            }

            int16_t raw_x = read_word(ACCEL_XOUT_H);
            int16_t raw_y = read_word(ACCEL_YOUT_H);
            int16_t raw_z = read_word(ACCEL_ZOUT_H);
            double ax = (raw_x / ACCEL_SCALE) * G;
            double ay = (raw_y / ACCEL_SCALE) * G;
            //double az = (raw_z / ACCEL_SCALE) * G - G;
            double az = ((read_word(ACCEL_ZOUT_H) / ACCEL_SCALE) * G) - z_offset;  // Remove gravity effect


            {
                lock_guard<std::mutex> lock(mtx);

                // X-axis processing
                if (ax > ACCEL_THRESHOLD_POS && !threshold_crossed_x) {
                    counter_x += INCREMENT_VALUE;
                    threshold_crossed_x = true;
                } else if ((ax < ACCEL_THRESHOLD_POS) && (ax > ACCEL_THRESHOLD_NEG)) {
                    threshold_crossed_x = false;
                } else if (ax < ACCEL_THRESHOLD_NEG && !threshold_crossed_x) {
                    counter_x -= INCREMENT_VALUE;
                    threshold_crossed_x = true;
                }

                // Y-axis processing
                if (ay > ACCEL_THRESHOLD_POS && !threshold_crossed_y) {
                    counter_y += INCREMENT_VALUE;
                    threshold_crossed_y = true;
                } else if ((ay < ACCEL_THRESHOLD_POS) && (ay > ACCEL_THRESHOLD_NEG)) {
                    threshold_crossed_y = false;
                } else if (ay < ACCEL_THRESHOLD_NEG && !threshold_crossed_y) {
                    counter_y -= INCREMENT_VALUE;
                    threshold_crossed_y = true;
                }

                // Z-axis processing
                if (az > ACCEL_THRESHOLD_POS_Z && !threshold_crossed_z) {
                    counter_z += INCREMENT_VALUE;
                    threshold_crossed_z = true;
                } else if ((az < ACCEL_THRESHOLD_POS_Z) && (az > ACCEL_THRESHOLD_NEG_Z)) {
                    threshold_crossed_z = false;
                } else if (az < ACCEL_THRESHOLD_NEG_Z && !threshold_crossed_z) {
                    counter_z -= INCREMENT_VALUE;
                    threshold_crossed_z = true;
                }

                if (fabs(ax - last_ax) > 0.1 || fabs(ay - last_ay) > 0.1 || fabs(az - last_az) > 0.1) {
                    //std::cout << "Acceleration X: " << ax << " m/s² | Counter X: " << counter_x 
                    //          << " | Acceleration Y: " << ay << " m/s² | Counter Y: " << counter_y
                    //          << " | Acceleration Z: " << az << " m/s² | Counter Z: " << counter_z << std::endl;
                    std::cout << "Counter_X = " << counter_x << " || Counter_Y = " << counter_y << " || Counter_Z = " << counter_z << std::endl;
                    last_ax = ax;
                    last_ay = ay;
                    last_az = az;
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

// GPIO button handling
void button_interrupt(MPU6050& sensor) {
    struct gpiod_chip* chip = gpiod_chip_open_by_name(GPIO_CHIP);
    if (!chip) {
        cerr << "Error: Failed to open GPIO chip." << endl;
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
        cerr << "Error: Failed to configure GPIO." << endl;
        gpiod_chip_close(chip);
        return;
    }

    cout << "Waiting for button press..." << endl;

    while (true) {
        struct gpiod_line_event event;
        int ret = gpiod_line_event_wait(line, nullptr);

        if (ret > 0 && gpiod_line_event_read(line, &event) == 0) {
            if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
                cout << "Button pressed! Starting MPU6050 data collection..." << endl;
                sensor.set_active(true);

                while (gpiod_line_get_value(line) == 0) {
                    this_thread::sleep_for(chrono::milliseconds(50));
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
    sensor.calibrate_z_axis();
    sensor.start_monitoring();

    thread gpio_thread(button_interrupt, ref(sensor));
    gpio_thread.detach();

    while (true) {
        this_thread::sleep_for(chrono::milliseconds(1000));
    }
    return 0;
}
