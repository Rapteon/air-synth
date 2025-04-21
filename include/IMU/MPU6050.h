#ifndef MPU6050_H
#define MPU6050_H

#include "IMU/MPU6050.h"
#include "Button/Button.h"
#include "ControllerEvent/ControllerEvent.h"

#include <iostream>
#include <mutex>
#include <atomic>
#include <memory>

// Forward declaration
class MPUSynth;

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

/**
 * @class MPU6050
 * @brief Handles communication with the MPU6050 motion sensor and manages its state.
 * 
 * This class provides a thread-safe interface to the MPU6050 sensor, handling
 * I2C communication, motion detection, and synthesizer control. It implements
 * proper resource management and thread synchronization.
 */
class MPU6050 : public Button::ButtonCallbackInterface{

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

    ~MPU6050();

    // TODO convert to private method.

    void worker();

    void start();

    struct ControllerCallbackInterface {
        virtual void hasEvent(ControllerEvent &e) = 0;
    };

    /**
     * Registers callback handlers which would be called
     * upon movement of the sensor.
     * Callback handler classes must implement the ControllerCallbackInterface.
     */
     void registerCallback(ControllerCallbackInterface *ci);
    
    // Current musical configuration
    std::string currentRootNote = "C";
    std::string currentScaleType = "Major";
    std::string currentInstrumentType = "Piano";
    
    /**
     * @brief Attaches a synthesizer to the sensor.
     */
    void attach_synth();
    
    // Getters for counter values
    int get_counter_x() const noexcept { return counter_x.load(); }
    int get_counter_y() const noexcept { return counter_y.load(); }
    int get_counter_z() const noexcept { return counter_z.load(); }

    /**
     * @brief Sets the active state of the sensor
     * @param state True to activate, false to deactivate
     */
    void set_active(bool state);
    
    /**
     * @brief Handles controller events
     * @param event The controller event
     */
    void hasEvent(ControllerEvent& event);
    
private:
    int file;  // I2C file descriptor
    const int address;  // I2C address
    std::mutex mtx;  // Mutex for thread-safe operations
    std::atomic<bool> running;  // Thread control flag
    std::atomic<bool> active;  // Sensor active state
    std::atomic<int> counter_x, counter_y, counter_z;  // Motion counters
    std::atomic<bool> threshold_crossed_x, threshold_crossed_y, threshold_crossed_z;  // Threshold state
    std::atomic<double> z_offset;  // Z-axis calibration offset
    std::vector<ControllerCallbackInterface *> registeredCallbacks;
    std::unique_ptr<MPUSynth> synth;  // Synthesizer instance
    

    /**
     * @brief Calibrates the Z-axis of the sensor.
     * This method should be called when the sensor is stationary.
     */
    void calibrate_z_axis();

    /**
     * @brief Reads a 16-bit word from the specified register.
     * @param reg Register address to read from
     * @return 16-bit value from the register
     */
    int16_t read_word(int8_t reg);

    /**
     * @brief Triggers events for registered callbacks
     * @param event The controller event to forward
     */
    void onEvent(ControllerEvent &event);
};

#endif // MPU_MAIN_H