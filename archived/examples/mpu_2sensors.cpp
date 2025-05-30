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


#define PWR_MGMT_1 0x6B
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

using namespace std;

class MPU6050 {

public:
    MPU6050(int addr) : address(addr), running(true), active(false), counter_x(0), counter_y(0), counter_z(0), 
                        threshold_crossed_x(false), threshold_crossed_y(false), threshold_crossed_z(false) {
        if ((file = open(I2C_BUS, O_RDWR)) < 0) {
            std::cerr << "Failed to open I2C bus" << std::endl;
        }
        if (ioctl(file, I2C_SLAVE, address) < 0) {
            std::cerr << "Failed to initialize MPU6050 at address " << address << std::endl;
        }
        uint8_t config[2] = {PWR_MGMT_1, 0x00};
        if (write(file, config, 2) != 2) {
            std::cerr << "Failed to initialize MPU6050 at address " << address << std::endl;
        }
    }

    ~MPU6050() {
        close(file);
        std::cout << "I2C Closed for address " << address << "\n";
    }

    int16_t read_word(int8_t reg) {
        uint8_t buffer[2];
        write(file, &reg, 1);
        read(file, buffer, 2);
        return (buffer[0] << 8) | buffer[1];
    }

    void calibrate_z_axis() {
        constexpr int CALIBRATION_SAMPLES = 100;
        double sum_az = 0.0;

        std::cout << "Calibrating Z-axis... Please keep the sensor stationary. Device Address: "<<address << std::endl;
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            sum_az += (read_word(ACCEL_ZOUT_H) / ACCEL_SCALE) * G;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        z_offset = sum_az / CALIBRATION_SAMPLES;
        std::cout << "Z-axis calibration completed. Offset: " << z_offset << " m/s²" << std::endl;
    }

    void monitor_threshold() {
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
            double az = ((raw_z / ACCEL_SCALE) * G) - z_offset;
            {
                lock_guard<std::mutex> lock(mtx);
                if (ax > ACCEL_THRESHOLD_POS && !threshold_crossed_x) counter_x += INCREMENT_VALUE, threshold_crossed_x = true;
                else if (ax < ACCEL_THRESHOLD_NEG && !threshold_crossed_x) counter_x -= INCREMENT_VALUE, threshold_crossed_x = true;
                else if (ax > ACCEL_THRESHOLD_NEG && ax < ACCEL_THRESHOLD_POS) threshold_crossed_x = false;
                
                if (ay > ACCEL_THRESHOLD_POS && !threshold_crossed_y) counter_y += INCREMENT_VALUE, threshold_crossed_y = true;
                else if (ay < ACCEL_THRESHOLD_NEG && !threshold_crossed_y) counter_y -= INCREMENT_VALUE, threshold_crossed_y = true;
                else if (ay > ACCEL_THRESHOLD_NEG && ay < ACCEL_THRESHOLD_POS) threshold_crossed_y = false;
                
                if (az > ACCEL_THRESHOLD_POS_Z && !threshold_crossed_z) counter_z += INCREMENT_VALUE, threshold_crossed_z = true;
                else if (az < ACCEL_THRESHOLD_NEG_Z && !threshold_crossed_z) counter_z -= INCREMENT_VALUE, threshold_crossed_z = true;
                else if (az > ACCEL_THRESHOLD_NEG_Z && az < ACCEL_THRESHOLD_POS_Z) threshold_crossed_z = false;
                
                std::cout << "[MPU " << address << "] Counter_X = " << counter_x << " || Counter_Y = " << counter_y << " || Counter_Z = " << counter_z << std::endl;
            }
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

    void start_monitoring() {
        thread(&MPU6050::monitor_threshold, this).detach();
    }

    void set_active(bool state) { active = state; }
};

void button_interrupt(MPU6050& sensor, int button_gpio) {
    struct gpiod_chip* chip = gpiod_chip_open_by_name(GPIO_CHIP);
    struct gpiod_line* line = gpiod_chip_get_line(chip, button_gpio);
    struct gpiod_line_request_config config = {"button_monitor", GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP};
    gpiod_line_request(line, &config, 0);
    while (true) {
        struct gpiod_line_event event;
        if (gpiod_line_event_wait(line, nullptr) > 0 && gpiod_line_event_read(line, &event) == 0) {
            if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
                sensor.set_active(true);
                while (gpiod_line_get_value(line) == 0) this_thread::sleep_for(chrono::milliseconds(50));
                sensor.set_active(false);
            }
        }
    }
    gpiod_chip_close(chip);
}
