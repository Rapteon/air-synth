#ifndef MPU6050_H
#define MPU6050_H

#include "Bus/I2CBus.h"
#include "IMU/MPU6050.h"
#include "Button/Button.h"

#include <iostream>
#include <mutex>
#include <atomic>

// TODO: These global defines should be part of
// a config file. The user of this library shouldn't
// have to worry about configuring the right registers.
// Calling setters for configuring these registers
// based on a struct based by the user would be a great idea.
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

class MPU6050 : public Button::ButtonCallbackInterface
{
public:
    virtual void hasEvent(gpiod_line_event &event) override
    {
        // TODO: Should handle rising and falling separately
        // to set active as true/false based on rising/falling.
        // if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE)
        // {
        //     set_active(true);
        //     while (gpiod_line_get_value(line) == 0)
        //         this_thread::sleep_for(chrono::milliseconds(50));
        //     set_active(false);
        // }
        if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
            std::cerr << "Handling event: falling\n";
            active = true;
        }
        else if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
            std::cerr << "Handling event: rising\n";
            active = false;
        }
        else {
            std::cerr << "Event type: " << event.event_type << '\n';
        }
    }
    MPU6050(int addr);
    MPU6050(I2CBus &bus);

    ~MPU6050();

    // TODO convert to private method.

    void worker();

    void start();

    // TODO replace content of this method within hasEvent() inherited method.
    // void button_interrupt(MPU6050 &sensor, int button_gpio);
    void setAddress(int address);
    int getAddress();
    I2CBus getBus();
private:
    int file;
    int address;
    I2CBus bus;
    std::mutex mtx;
    std::atomic<bool> running;
    std::atomic<bool> active;
    int counter_x, counter_y, counter_z;
    bool threshold_crossed_x, threshold_crossed_y, threshold_crossed_z;
    double z_offset;

    void calibrate_z_axis();
    int16_t read_word(int8_t reg);
};

#endif