#ifndef MPU_MAIN_H
#define MPU_MAIN_H

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
#include <queue>
#include <memory>
#include <algorithm>

// Forward declarations
class MPUSynth;
class ControllerEvent;
enum class ScaleType;
enum class InstrumentType;

// MPU6050 constants
namespace MPU6050Constants {
    constexpr int ADDR1 = 0x68;
    constexpr int ADDR2 = 0x69;
    constexpr int PWR_MGMT_1 = 0x6B;
    constexpr int ACCEL_XOUT_H = 0x3B;
    constexpr int ACCEL_YOUT_H = 0x3D;
    constexpr int ACCEL_ZOUT_H = 0x3F;
    constexpr const char* I2C_BUS = "/dev/i2c-1";
    constexpr double ACCEL_SCALE = 16384.0;
    constexpr double G = 9.81;
    constexpr double ACCEL_THRESHOLD_POS = 3.0;  // Changed from 5.0
    constexpr double ACCEL_THRESHOLD_NEG = -3.0; // Changed from -5.0
    constexpr double ACCEL_THRESHOLD_POS_Z = 2.0; // Changed from 3.0
    constexpr double ACCEL_THRESHOLD_NEG_Z = -2.0; // Changed from -3.0
    constexpr int INCREMENT_VALUE = 8;   // Changed from 5
    constexpr int DECREMENT_VALUE = 8;   // Changed from 5
    constexpr const char* GPIO_CHIP = "gpiochip0";
    constexpr int BUTTON_GPIO1 = 20;
    constexpr int BUTTON_GPIO2 = 21;
    constexpr int SAMPLE_RATE = 44100;
}

/**
 * @class MPU6050
 * @brief Handles communication with the MPU6050 motion sensor and manages its state.
 * 
 * This class provides a thread-safe interface to the MPU6050 sensor, handling
 * I2C communication, motion detection, and synthesizer control. It implements
 * proper resource management and thread synchronization.
 */
class MPU6050 {
private:
    int file;  // I2C file descriptor
    const int address;  // I2C address
    std::mutex mtx;  // Mutex for thread-safe operations
    std::atomic<bool> running;  // Thread control flag
    std::atomic<bool> active;  // Sensor active state
    std::atomic<int> counter_x, counter_y, counter_z;  // Motion counters
    std::atomic<int> prev_counter_x, prev_counter_y, prev_counter_z;  // Previous counter values
    std::atomic<bool> threshold_crossed_x, threshold_crossed_y, threshold_crossed_z;  // Threshold state
    std::atomic<double> z_offset;  // Z-axis calibration offset
    std::unique_ptr<MPUSynth> synth;  // Synthesizer instance
    std::thread event_thread;  // Event generation thread
    std::atomic<bool> event_thread_running;  // Event thread control flag

    /**
     * @brief Generates motion events based on sensor data.
     * This method runs in a separate thread and continuously monitors sensor data.
     */
    void generate_events();

    /**
     * @brief Reads a 16-bit word from the specified register.
     * @param reg Register address to read from
     * @return 16-bit value from the register
     */
    int16_t read_word(int8_t reg) const;

public:
    // Current musical configuration
    std::string currentRootNote = "C";
    std::string currentScaleType = "Major";
    std::string currentInstrumentType = "Piano";
    
    /**
     * @brief Constructs a new MPU6050 instance.
     * @param addr I2C address of the MPU6050 sensor
     * @throws std::runtime_error if sensor initialization fails
     */
    explicit MPU6050(int addr);
    
    /**
     * @brief Destructor ensures proper cleanup of resources.
     */
    ~MPU6050();

    // Delete copy constructor and assignment operator
    MPU6050(const MPU6050&) = delete;
    MPU6050& operator=(const MPU6050&) = delete;

    /**
     * @brief Calibrates the Z-axis of the sensor.
     * This method should be called when the sensor is stationary.
     */
    void calibrate_z_axis();

    /**
     * @brief Starts monitoring the sensor for motion.
     * This method launches a background thread for continuous monitoring.
     */
    void start_monitoring();

    /**
     * @brief Attaches a synthesizer to the sensor.
     * @param rootNote The root note for the scale
     * @param scaleType The type of musical scale
     * @param instrumentType The type of instrument to use
     */
    void attach_synth(const std::string& rootNote, ScaleType scaleType, InstrumentType instrumentType);

    /**
     * @brief Starts the synthesizer.
     */
    void start_synth();

    /**
     * @brief Stops the synthesizer.
     */
    void stop_synth();

    /**
     * @brief Sets the active state of the sensor.
     * @param state True to activate, false to deactivate
     */
    void set_active(bool state);

    /**
     * @brief Changes the root note of the synthesizer.
     * @param rootNote The new root note
     */
    void change_root_note(const std::string& rootNote);
    
    /**
     * @brief Changes the scale type of the synthesizer.
     * @param scaleTypeName The name of the new scale type
     */
    void change_scale_type(const std::string& scaleTypeName);
    
    /**
     * @brief Changes the instrument type of the synthesizer.
     * @param instrumentTypeName The name of the new instrument type
     */
    void change_instrument_type(const std::string& instrumentTypeName);
    
    /**
     * @brief Cycles to the next root note in the available list.
     */
    void cycle_root_note();
    
    /**
     * @brief Cycles to the next scale type in the available list.
     */
    void cycle_scale_type();
    
    /**
     * @brief Cycles to the next instrument type in the available list.
     */
    void cycle_instrument_type();
    
    /**
     * @brief Updates the synthesizer with the current musical configuration.
     */
    void update_synth_configuration();
    
    /**
     * @brief Displays the current musical configuration.
     */
    void display_current_configuration() const;

    // Getters for counter values
    int get_counter_x() const noexcept { return counter_x.load(); }
    int get_counter_y() const noexcept { return counter_y.load(); }
    int get_counter_z() const noexcept { return counter_z.load(); }
};

/**
 * @brief Handles button interrupts for a sensor.
 * @param sensor Reference to the MPU6050 sensor
 * @param button_gpio GPIO pin number for the button
 */
void button_interrupt(MPU6050& sensor, int button_gpio);

/**
 * @brief Signal handler for clean shutdown.
 * @param signum Signal number
 */
void signal_handler(int signum);

#endif // MPU_MAIN_H    