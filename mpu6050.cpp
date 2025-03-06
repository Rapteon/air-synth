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
#define ACCEL_XOUT_H 0x3B

#define I2C_BUS "/dev/i2c-1" // Specify the I2C bus
#define ACCEL_SCALE 16384.0   // Scale factor for ±2g range
#define G 9.81                // Gravity in m/s²
#define ACCEL_THRESHOLD_POS 5.0   // Acceleration threshold in m/s²
#define ACCEL_THRESHOLE_NEG -5.0 //Negative acceleration threshole in m/s2
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
    std::atomic<bool> active; //Indicates if MPU6050 is reading data or not
    int counter;
    bool threshold_crossed;

    public:
    MPU6050() : running(true), active(false), counter(0), threshold_crossed(false) {
        signal(SIGINT, signal_handler);
        if((file = open(I2C_BUS,O_RDWR))<0){
            std::cerr << "Failed to open I2C bus"<<std::endl;
        }

        if(ioctl(file,I2C_SLAVE,MPU6050_ADDR) < 0) {
            std::cerr << "Failed to initialise MPU6050" << std::endl;
        }

        uint8_t config[2] = {PWR_MGMT_1, 0x00};
        if(write(file, config, 2) != 2) {
            std::cerr << "Failed to initialise MPU6050" << std::endl;
        }
    }

    ~MPU6050() {
        close(file);
        std::cout << "I2C Closed. Exiting Program." << std::endl;
    }

    static void signal_handler(int signum){
        std::cout <<"\nTerminating Program Safely...\n";
        exit(0);
    }

    int16_t read_word(int8_t reg) {
        uint8_t buffer[2];
        if(write(file, &reg, 1) != 1) {
            std::cerr << "Failed to write register address" << std::endl;
        }

        if (read(file, buffer, 2) != 2) {
            std::cerr << "Failed to read data" << endl;
        }
        return (buffer[0] << 8) | buffer[1];
    }

    void monitor_threshold() {
        double last_ax = 0.0;
        while(running) {
            if (!active) {
                this_thread::sleep_for(chrono::milliseconds(100)); //sleep if button is not pressed
                continue;
            }

            int16_t raw_x = read_word(ACCEL_XOUT_H);
            double ax = (raw_x / ACCEL_SCALE) * G;

            {
                lock_guard<std::mutex> lock(mtx);
                if(ax > ACCEL_THRESHOLD_POS && !threshold_crossed){
                    counter += INCREMENT_VALUE;
                    threshold_crossed = true;
                } else if((ax < ACCEL_THRESHOLD_POS) && (ax > ACCEL_THRESHOLE_NEG)) {
                    threshold_crossed = false;
                } else if(ax < ACCEL_THRESHOLE_NEG && !threshold_crossed){
                    counter -= INCREMENT_VALUE;
                    threshold_crossed = true;
                }
                if (fabs(ax - last_ax) > 0.1) {
                    std::cout << "Acceleration: " << ax << "m/s2 | Counter " << counter << std::endl;
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

    // Configure GPIO as input with an internal pull-up resistor
    struct gpiod_line_request_config config = {};
    config.consumer = "button_monitor";
    config.request_type = GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE;
    config.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP; // Enable internal pull-up

    if (gpiod_line_request(line, &config, 0) < 0) {  // Third argument (0) sets initial GPIO state
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

    thread gpio_thread(button_interrupt, ref(sensor));  // Create a thread for GPIO handling
    gpio_thread.detach();  // Run GPIO monitoring in the background

    while (true) {
        this_thread::sleep_for(chrono::milliseconds(1000));  // Keep main thread alive
    }

    return 0;
}
