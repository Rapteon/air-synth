#include <iostream>
#include <cassert>
#include <cstdint>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <vector>
#include <functional>
#include <condition_variable>
#include <map>  // Added missing include for std::map

// Mock structures and constants to simulate hardware interfaces
// I2C mocks
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define I2C_BUS "/dev/i2c-1"
#define I2C_SLAVE 0x0703

// GPIO mocks
#define GPIO_CHIP "gpiochip0"
#define BUTTON_GPIO 4

// Constant values
#define ACCEL_SCALE 16384.0
#define G 9.81
#define ACCEL_THRESHOLD 5.0
#define INCREMENT_VALUE 5

// Mock I2C interface
class I2CMock {
private:
    int file_descriptor;
    std::map<uint8_t, uint8_t> registers;
    std::vector<std::pair<uint8_t, uint8_t>> write_history;
    
public:
    I2CMock() : file_descriptor(10) {
        // Initialize with default register values
        registers[PWR_MGMT_1] = 0x40;  // Default power management (sleep mode)
        registers[ACCEL_CONFIG] = 0x00; // Default accelerometer config
        
        // Set up accelerometer data registers with initial values
        registers[ACCEL_XOUT_H] = 0x00;
        registers[ACCEL_XOUT_H + 1] = 0x00;
        registers[ACCEL_XOUT_H + 2] = 0x00;
        registers[ACCEL_XOUT_H + 3] = 0x00;
        registers[ACCEL_XOUT_H + 4] = 0x10; // Z-axis has a small offset to simulate gravity
        registers[ACCEL_XOUT_H + 5] = 0x00;
        
        std::cout << "I2C Mock initialized" << std::endl;
    }
    
    int open(const char* bus) {
        std::cout << "Opening I2C bus: " << bus << std::endl;
        return file_descriptor;
    }
    
    int close(int fd) {
        std::cout << "Closing I2C file descriptor: " << fd << std::endl;
        return 0;
    }
    
    int ioctl(int fd, unsigned long request, void* arg) {
        if (fd != file_descriptor) {
            std::cout << "Invalid file descriptor in ioctl" << std::endl;
            return -1;
        }
        
        if (request == I2C_SLAVE) {
            // Note: MPU6050_ADDR is now handled correctly as a value, not an address
            uint8_t addr = *static_cast<uint8_t*>(arg);
            std::cout << "Setting I2C slave address to 0x" << std::hex << static_cast<int>(addr) << std::dec << std::endl;
            return 0;
        }
        
        std::cout << "Unsupported ioctl request: " << request << std::endl;
        return -1;
    }
    
    int write(int fd, const void* buffer, size_t count) {
        if (fd != file_descriptor) {
            std::cout << "Invalid file descriptor in write" << std::endl;
            return -1;
        }
        
        if (count < 1) {
            std::cout << "Invalid write count" << std::endl;
            return -1;
        }
        
        const uint8_t* data = static_cast<const uint8_t*>(buffer);
        
        if (count == 1) {
            // Single byte write - this is usually a register address for a subsequent read
            std::cout << "Setting register pointer to 0x" << std::hex << static_cast<int>(data[0]) << std::dec << std::endl;
            return 1;
        } else if (count == 2) {
            // Two byte write - register address and value
            uint8_t reg = data[0];
            uint8_t value = data[1];
            registers[reg] = value;
            write_history.push_back(std::make_pair(reg, value));
            
            std::cout << "Writing 0x" << std::hex << static_cast<int>(value) << " to register 0x" 
                      << static_cast<int>(reg) << std::dec << std::endl;
            
            // Check for special registers
            if (reg == PWR_MGMT_1 && value == 0x00) {
                std::cout << "MPU6050 woken up from sleep mode" << std::endl;
            } else if (reg == ACCEL_CONFIG) {
                std::cout << "MPU6050 accelerometer configured with value: 0x" << std::hex 
                          << static_cast<int>(value) << std::dec << std::endl;
            }
            
            return 2;
        }
        
        std::cout << "Unsupported write length: " << count << std::endl;
        return -1;
    }
    
    int read(int fd, void* buffer, size_t count) {
        if (fd != file_descriptor) {
            std::cout << "Invalid file descriptor in read" << std::endl;
            return -1;
        }
        
        uint8_t* data = static_cast<uint8_t*>(buffer);
        
        // Read from the last accessed register
        uint8_t last_reg = write_history.empty() ? 0 : write_history.back().first;
        
        for (size_t i = 0; i < count; i++) {
            data[i] = registers[last_reg + i];
        }
        
        std::cout << "Reading " << count << " bytes from register 0x" << std::hex 
                  << static_cast<int>(last_reg) << std::dec << std::endl;
        return count;
    }
    
    // Method to simulate accelerometer readings
    void set_acceleration(double x_accel, double y_accel, double z_accel) {
        // Convert acceleration in m/s² to raw values
        int16_t x_raw = static_cast<int16_t>((x_accel / G) * ACCEL_SCALE);
        int16_t y_raw = static_cast<int16_t>((y_accel / G) * ACCEL_SCALE);
        int16_t z_raw = static_cast<int16_t>((z_accel / G) * ACCEL_SCALE);
        
        // Set high and low bytes for each axis
        registers[ACCEL_XOUT_H] = (x_raw >> 8) & 0xFF;
        registers[ACCEL_XOUT_H + 1] = x_raw & 0xFF;
        
        registers[ACCEL_XOUT_H + 2] = (y_raw >> 8) & 0xFF;
        registers[ACCEL_XOUT_H + 3] = y_raw & 0xFF;
        
        registers[ACCEL_XOUT_H + 4] = (z_raw >> 8) & 0xFF;
        registers[ACCEL_XOUT_H + 5] = z_raw & 0xFF;
        
        std::cout << "Setting acceleration - X: " << x_accel 
                  << " m/s², Y: " << y_accel 
                  << " m/s², Z: " << z_accel << " m/s²" << std::endl;
    }
};

// Mock GPIO interface
class GPIOMock {
private:
    std::atomic<bool> button_pressed;
    std::thread button_thread;
    std::atomic<bool> thread_running;
    std::condition_variable cv;
    std::mutex mtx;
    std::function<void()> button_press_callback;
    
public:
    GPIOMock() : button_pressed(false), thread_running(false) {
        std::cout << "GPIO Mock initialized" << std::endl;
    }
    
    ~GPIOMock() {
        if (thread_running) {
            thread_running = false;
            if (button_thread.joinable()) {
                button_thread.join();
            }
        }
    }
    
    void register_button_callback(std::function<void()> callback) {
        button_press_callback = callback;
    }
    
    // Method to simulate button press and release
    void simulate_button_press(int duration_ms) {
        std::unique_lock<std::mutex> lock(mtx);
        button_pressed = true;
        cv.notify_all();
        
        std::cout << "Button pressed" << std::endl;
        
        // Call the callback if registered
        if (button_press_callback) {
            button_press_callback();
        }
        
        // Wait for duration
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
        
        button_pressed = false;
        std::cout << "Button released" << std::endl;
    }
    
    // Method to start a thread that monitors button state
    void start_monitoring() {
        thread_running = true;
        button_thread = std::thread([this]() {
            while (thread_running) {
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock, [this]() { 
                    return button_pressed || !thread_running; 
                });
                
                if (!thread_running) break;
                
                // Button is pressed, wait for release
                while (button_pressed && thread_running) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        });
    }
    
    bool is_button_pressed() const {
        return button_pressed;
    }
};

// Comprehensive MPU6050 class that uses the mocks
class MPU6050 {
private:
    I2CMock i2c;
    GPIOMock gpio;
    int file;
    std::mutex mtx;
    std::atomic<bool> running;
    std::atomic<bool> active;
    int counter;
    bool threshold_crossed;
    std::thread monitor_thread;
    
public:
    MPU6050() : running(true), active(false), counter(0), threshold_crossed(false) {
        // Initialize I2C
        file = i2c.open(I2C_BUS);
        if (file < 0) {
            std::cerr << "Failed to open I2C bus" << std::endl;
            throw std::runtime_error("I2C initialization failed");
        }
        
        // Fix: Pass the address value correctly to ioctl
        uint8_t addr = MPU6050_ADDR;
        if (i2c.ioctl(file, I2C_SLAVE, &addr) < 0) {
            std::cerr << "Failed to acquire bus access" << std::endl;
            throw std::runtime_error("I2C bus access failed");
        }
        
        // Initialize MPU6050
        uint8_t config[2] = {PWR_MGMT_1, 0x00};
        if (i2c.write(file, config, 2) != 2) {
            std::cerr << "Failed to initialize MPU6050" << std::endl;
            throw std::runtime_error("MPU6050 initialization failed");
        }
        
        // Set accelerometer configuration
        config[0] = ACCEL_CONFIG;
        config[1] = 0x00;  // ±2g range
        if (i2c.write(file, config, 2) != 2) {
            std::cerr << "Failed to configure accelerometer" << std::endl;
            throw std::runtime_error("Accelerometer configuration failed");
        }
        
        // Setup GPIO button callback
        gpio.register_button_callback([this]() {
            this->set_active(true);
        });
        
        std::cout << "MPU6050 initialized successfully" << std::endl;
    }
    
    ~MPU6050() {
        running = false;
        if (monitor_thread.joinable()) {
            monitor_thread.join();
        }
        i2c.close(file);
        std::cout << "MPU6050 resources cleaned up" << std::endl;
    }
    
    int16_t read_word(uint8_t reg) {
        uint8_t buffer[2];
        uint8_t reg_addr = reg;
        
        if (i2c.write(file, &reg_addr, 1) != 1) {
            std::cerr << "Failed to write register address" << std::endl;
            return 0;
        }
        
        if (i2c.read(file, buffer, 2) != 2) {
            std::cerr << "Failed to read data" << std::endl;
            return 0;
        }
        
        return (buffer[0] << 8) | buffer[1];
    }
    
    void monitor_threshold() {
        double last_ax = 0.0;
        
        while (running) {
            if (!active) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            int16_t raw_x = read_word(ACCEL_XOUT_H);
            double ax = (raw_x / ACCEL_SCALE) * G;
            
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (ax > ACCEL_THRESHOLD && !threshold_crossed) {
                    counter += INCREMENT_VALUE;
                    threshold_crossed = true;
                    std::cout << "Threshold crossed! Counter increased to " << counter << std::endl;
                } else if (ax < ACCEL_THRESHOLD) {
                    threshold_crossed = false;
                }
                
                if (std::abs(ax - last_ax) > 0.1) {
                    std::cout << "Acceleration: " << ax << " m/s² | Counter: " << counter << std::endl;
                    last_ax = ax;
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    void start_monitoring() {
        monitor_thread = std::thread(&MPU6050::monitor_threshold, this);
    }
    
    void set_active(bool state) {
        active = state;
        std::cout << "MPU6050 active state set to: " << (state ? "true" : "false") << std::endl;
    }
    
    bool is_active() const {
        return active;
    }
    
    int get_counter() const {
        return counter;
    }
    
    void reset_counter() {
        std::lock_guard<std::mutex> lock(mtx);
        counter = 0;
        std::cout << "Counter reset to 0" << std::endl;
    }
    
    // Interface for tests to simulate acceleration
    void simulate_acceleration(double x_accel) {
        i2c.set_acceleration(x_accel, 0.0, G);  // Keep Z as gravity
    }
    
    // Interface for tests to simulate button press
    void simulate_button_press(int duration_ms) {
        gpio.simulate_button_press(duration_ms);
    }
};

// Comprehensive test function
void run_comprehensive_tests() {
    std::cout << "\n=== Running MPU6050 comprehensive tests ===\n" << std::endl;
    
    try {
        MPU6050 sensor;
        
        // Test 1: Initialization and initial state
        assert(!sensor.is_active());
        assert(sensor.get_counter() == 0);
        std::cout << "Test 1 passed: Initialization successful" << std::endl;
        
        // Start monitoring in a separate thread
        sensor.start_monitoring();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Test 2: Manual activation and deactivation
        sensor.set_active(true);
        assert(sensor.is_active());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        sensor.set_active(false);
        assert(!sensor.is_active());
        std::cout << "Test 2 passed: Manual activation control works" << std::endl;
        
        // Test 3: Acceleration threshold detection
        sensor.set_active(true);
        
        // Simulate normal acceleration (below threshold)
        sensor.simulate_acceleration(2.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        int counter_before = sensor.get_counter();
        
        // Simulate high acceleration (above threshold)
        sensor.simulate_acceleration(8.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        assert(sensor.get_counter() == counter_before + INCREMENT_VALUE);
        
        // Keep acceleration high, counter should not increase again
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        assert(sensor.get_counter() == counter_before + INCREMENT_VALUE);
        
        // Bring acceleration back down
        sensor.simulate_acceleration(1.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Increase acceleration again, counter should increase
        sensor.simulate_acceleration(7.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        assert(sensor.get_counter() == counter_before + (2 * INCREMENT_VALUE));
        
        std::cout << "Test 3 passed: Acceleration threshold detection works" << std::endl;
        
        // Test 4: Counter reset
        int counter_value = sensor.get_counter();
        assert(counter_value > 0);
        
        sensor.reset_counter();
        assert(sensor.get_counter() == 0);
        std::cout << "Test 4 passed: Counter reset works" << std::endl;
        
        // Test 5: Button press simulation
        sensor.set_active(false);
        assert(!sensor.is_active());
        
        // Simulate button press (activates the sensor)
        std::thread button_thread([&sensor]() {
            sensor.simulate_button_press(500);
        });
        
        // Wait for button press to be processed
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        assert(sensor.is_active());
        
        // Wait for button release
        button_thread.join();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::cout << "Test 5 passed: Button press/release simulation works" << std::endl;
        
        // Test 6: Combined button and acceleration
        sensor.reset_counter();
        
        // Simulate button press and acceleration at the same time
        std::thread combined_test([&sensor]() {
            sensor.simulate_button_press(1000);
        });
        
        // Wait for button press to activate the sensor
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Simulate acceleration during button press
        sensor.simulate_acceleration(1.0);  // Below threshold
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        sensor.simulate_acceleration(9.0);  // Above threshold
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Threshold should be crossed and counter increased
        assert(sensor.get_counter() == INCREMENT_VALUE);
        
        // Wait for button release
        combined_test.join();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // After button release, sensor should be inactive
        assert(!sensor.is_active());
        
        std::cout << "Test 6 passed: Button and acceleration combined test" << std::endl;
        
        std::cout << "\nAll comprehensive tests passed successfully!" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        assert(false);
    }
}

int main() {
    run_comprehensive_tests();
    return 0;
}