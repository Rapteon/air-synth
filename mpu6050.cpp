#include "mpu6050.h"
#include "Synth/Synth.h"
#include "ControllerEvent/ControllerEvent.h"
#include <stdexcept>

using namespace MPU6050Constants;

// Implementation of MPU6050 methods
MPU6050::MPU6050(int addr) 
    : address(addr), 
      running(true), 
      active(false),
      counter_x(0), counter_y(0), counter_z(0),
      prev_counter_x(0), prev_counter_y(0), prev_counter_z(0),
      threshold_crossed_x(false), threshold_crossed_y(false), threshold_crossed_z(false),
      z_offset(0.0),
      event_thread_running(false) {
    
    // Open I2C bus
    if ((file = open(I2C_BUS, O_RDWR)) < 0) {
        throw std::runtime_error("Failed to open I2C bus");
    }
    
    // Set I2C slave address
    if (ioctl(file, I2C_SLAVE, address) < 0) {
        close(file);
        throw std::runtime_error("Failed to initialize MPU6050 at address " + std::to_string(address));
    }
    
    // Initialize power management
    uint8_t config[2] = {PWR_MGMT_1, 0x00};
    if (write(file, config, 2) != 2) {
        close(file);
        throw std::runtime_error("Failed to initialize MPU6050 power management");
    }
}

MPU6050::~MPU6050() {
    // Stop all threads
    running = false;
    if (event_thread_running && event_thread.joinable()) {
        event_thread.join();
    }
    
    // Close I2C connection
    if (file >= 0) {
        close(file);
    }
    
    // Clean up synth
    if (synth) {
        synth->stop();
    }
}

int16_t MPU6050::read_word(int8_t reg) const {
    uint8_t buffer[2];
    if (write(file, &reg, 1) != 1) {
        throw std::runtime_error("Failed to write to MPU6050 register");
    }
    if (read(file, buffer, 2) != 2) {
        throw std::runtime_error("Failed to read from MPU6050 register");
    }
    return (buffer[0] << 8) | buffer[1];
}

void MPU6050::calibrate_z_axis() {
    constexpr int CALIBRATION_SAMPLES = 100;
    double sum_az = 0.0;

    std::cout << "Calibrating Z-axis... Please keep the sensor stationary. Device Address: " << address << std::endl;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        try {
            sum_az += (read_word(ACCEL_ZOUT_H) / ACCEL_SCALE) * G;
        } catch (const std::runtime_error& e) {
            std::cerr << "Calibration error: " << e.what() << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    z_offset = sum_az / CALIBRATION_SAMPLES;
    std::cout << "Z-axis calibration completed. Offset: " << z_offset << " m/s²" << std::endl;
}

void MPU6050::attach_synth(const std::string& rootNote, ScaleType scaleType, InstrumentType instrumentType) {
    // Create the synthesizer
    synth = std::make_unique<MPUSynth>(this, SAMPLE_RATE);
    
    // Configure the synthesizer with wider note range and more responsive settings
    synth->setScale(scaleType, rootNote);
    synth->setInstrument(instrumentType);
    synth->setVolume(0.8f);  // Slightly louder
    synth->setNoteDuration(150.0f);  // Shorter notes for more responsiveness
    synth->setMappingMode(true, true, true);  // Enable all axes
    synth->setAxisNoteSpreads(12, 12, 12);  // More notes per axis for wider range
    
    // Higher sensitivity values make smaller movements have bigger effects
    float xSens = 15.0f / ACCEL_THRESHOLD_POS;
    float ySens = 15.0f / ACCEL_THRESHOLD_POS;
    float zSens = 15.0f / ACCEL_THRESHOLD_POS_Z;
    synth->setAxisVelocitySensitivity(xSens, ySens, zSens);
    synth->setContinuousMode(true);
    
    // Start event generation thread with more frequent events
    event_thread_running = true;
    event_thread = std::thread(&MPU6050::generate_events, this);
}

void MPU6050::generate_events() {
    int event_counter = 0;
    constexpr int EVENT_RATE = 5;  // Generate events more frequently (was 5)
    
    while (running && event_thread_running) {
        if (!active || !synth) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        
        event_counter++;
        
        if (event_counter >= EVENT_RATE) {
            event_counter = 0;
            
            std::lock_guard<std::mutex> lock(mtx);
            
            // Get current counter values
            int x_val = counter_x.load();
            int y_val = counter_y.load();
            int z_val = counter_z.load();
            
            // Check if there's been a significant change
            bool significant_change = 
                std::abs(x_val - prev_counter_x.load()) > 2 ||
                std::abs(y_val - prev_counter_y.load()) > 2 ||
                std::abs(z_val - prev_counter_z.load()) > 2;
            
            // Normalize counter values with more dynamic range
            double normalized_x = x_val / 20.0;  // Increased range (was 10.0)
            double normalized_y = y_val / 20.0;
            double normalized_z = z_val / 20.0;
            
            // Create and send event
            ControllerEvent event(normalized_x, normalized_y, normalized_z);
            synth->addEvent(event);
            
            // Update previous counters
            prev_counter_x = x_val;
            prev_counter_y = y_val;
            prev_counter_z = z_val;
            
            // Log all events for debugging
            std::cout << "Generated event: X=" << normalized_x 
                      << ", Y=" << normalized_y 
                      << ", Z=" << normalized_z 
                      << (significant_change ? " (SIGNIFICANT CHANGE)" : "")
                      << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(25));  // More frequent checks
    }
}

void MPU6050::start_synth() {
    if (synth) {
        synth->start();
    }
}

void MPU6050::stop_synth() {
    if (synth) {
        synth->stop();
    }
}

void MPU6050::start_monitoring() {
    std::thread([this]() {
        while (running) {
            if (!active) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            try {
                // Read accelerometer values
                int16_t raw_x = read_word(ACCEL_XOUT_H);
                int16_t raw_y = read_word(ACCEL_YOUT_H);
                int16_t raw_z = read_word(ACCEL_ZOUT_H);
                
                double ax = (raw_x / ACCEL_SCALE) * G;
                double ay = (raw_y / ACCEL_SCALE) * G;
                double az = ((raw_z / ACCEL_SCALE) * G) - z_offset.load();
                
                std::lock_guard<std::mutex> lock(mtx);
                
                // Calculate motion deltas (change since last reading)
                static double prev_ax = 0, prev_ay = 0, prev_az = 0;
                double delta_ax = ax - prev_ax;
                double delta_ay = ay - prev_ay;
                double delta_az = az - prev_az;
                
                // Save current values for next iteration
                prev_ax = ax;
                prev_ay = ay;
                prev_az = az;
                
                // Update X axis with more dynamic response
                if (std::abs(delta_ax) > 0.5) {  // More sensitive to changes
                    int increment = static_cast<int>(delta_ax * INCREMENT_VALUE);
                    counter_x.fetch_add(increment);
                    threshold_crossed_x.store(true);
                }
                else if (ax > ACCEL_THRESHOLD_NEG && ax < ACCEL_THRESHOLD_POS) {
                    // Gradually return counter to zero when motion stops
                    if (counter_x.load() > 0) counter_x.fetch_sub(1);
                    else if (counter_x.load() < 0) counter_x.fetch_add(1);
                    threshold_crossed_x.store(false);
                }
                
                // Update Y axis with more dynamic response
                if (std::abs(delta_ay) > 0.5) {
                    int increment = static_cast<int>(delta_ay * INCREMENT_VALUE);
                    counter_y.fetch_add(increment);
                    threshold_crossed_y.store(true);
                }
                else if (ay > ACCEL_THRESHOLD_NEG && ay < ACCEL_THRESHOLD_POS) {
                    if (counter_y.load() > 0) counter_y.fetch_sub(1);
                    else if (counter_y.load() < 0) counter_y.fetch_add(1);
                    threshold_crossed_y.store(false);
                }
                
                // Update Z axis with more dynamic response
                if (std::abs(delta_az) > 0.4) {  // Even more sensitive for Z axis
                    int increment = static_cast<int>(delta_az * INCREMENT_VALUE);
                    counter_z.fetch_add(increment);
                    threshold_crossed_z.store(true);
                }
                else if (az > ACCEL_THRESHOLD_NEG_Z && az < ACCEL_THRESHOLD_POS_Z) {
                    if (counter_z.load() > 0) counter_z.fetch_sub(1);
                    else if (counter_z.load() < 0) counter_z.fetch_add(1);
                    threshold_crossed_z.store(false);
                }
                
                // Ensure counters stay within reasonable bounds
                constexpr int MAX_COUNTER = 100;
                counter_x.store(std::min(MAX_COUNTER, std::max(-MAX_COUNTER, counter_x.load())));
                counter_y.store(std::min(MAX_COUNTER, std::max(-MAX_COUNTER, counter_y.load())));
                counter_z.store(std::min(MAX_COUNTER, std::max(-MAX_COUNTER, counter_z.load())));
                
                // Check for specific gesture patterns to trigger configuration changes
                // High Z motion with little X/Y motion = change instrument
                if (std::abs(counter_z.load()) > 50 && std::abs(counter_x.load()) < 10 && std::abs(counter_y.load()) < 10) {
                    // Reset Z counter
                    counter_z.store(0);
                    // Change instrument
                    cycle_instrument_type();
                    // Add a small delay to prevent multiple triggers
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }

                // High X motion with little Y/Z motion = change scale
                if (std::abs(counter_x.load()) > 50 && std::abs(counter_y.load()) < 10 && std::abs(counter_z.load()) < 10) {
                    // Reset X counter
                    counter_x.store(0);
                    // Change scale
                    cycle_scale_type();
                    // Add a small delay to prevent multiple triggers
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }

                // High Y motion with little X/Z motion = change root note
                if (std::abs(counter_y.load()) > 50 && std::abs(counter_x.load()) < 10 && std::abs(counter_z.load()) < 10) {
                    // Reset Y counter
                    counter_y.store(0);
                    // Change root note
                    cycle_root_note();
                    // Add a small delay to prevent multiple triggers
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                
                // Log counter values and deltas
                std::cout << "[MPU " << address << "] Counter_X = " << counter_x.load() 
                          << " || Counter_Y = " << counter_y.load() 
                          << " || Counter_Z = " << counter_z.load() 
                          << " || Delta_X = " << delta_ax
                          << " || Delta_Y = " << delta_ay
                          << " || Delta_Z = " << delta_az
                          << std::endl;
            }
            catch (const std::runtime_error& e) {
                std::cerr << "Error reading MPU6050: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Faster sampling rate
        }
    }).detach();
}

void MPU6050::set_active(bool state) {
    active = state;
    
    if (state && synth) {
        start_synth();
    } else if (!state && synth) {
        stop_synth();
    }
}

// New methods for dynamic musical configuration
void MPU6050::change_root_note(const std::string& rootNote) {
    // Validate root note
    auto it = std::find(AVAILABLE_ROOT_NOTES.begin(), AVAILABLE_ROOT_NOTES.end(), rootNote);
    if (it != AVAILABLE_ROOT_NOTES.end()) {
        currentRootNote = rootNote;
        std::cout << "Changed root note to: " << rootNote << std::endl;
        update_synth_configuration();
    } else {
        std::cerr << "Invalid root note: " << rootNote << std::endl;
    }
}

void MPU6050::change_scale_type(const std::string& scaleTypeName) {
    // Validate scale type
    auto it = AVAILABLE_SCALES.find(scaleTypeName);
    if (it != AVAILABLE_SCALES.end()) {
        currentScaleType = scaleTypeName;
        std::cout << "Changed scale type to: " << scaleTypeName << std::endl;
        update_synth_configuration();
    } else {
        std::cerr << "Invalid scale type: " << scaleTypeName << std::endl;
    }
}

void MPU6050::change_instrument_type(const std::string& instrumentTypeName) {
    // Validate instrument type
    auto it = AVAILABLE_INSTRUMENTS.find(instrumentTypeName);
    if (it != AVAILABLE_INSTRUMENTS.end()) {
        currentInstrumentType = instrumentTypeName;
        std::cout << "Changed instrument type to: " << instrumentTypeName << std::endl;
        update_synth_configuration();
    } else {
        std::cerr << "Invalid instrument type: " << instrumentTypeName << std::endl;
    }
}

void MPU6050::cycle_root_note() {
    // Find current root note in the list
    auto it = std::find(AVAILABLE_ROOT_NOTES.begin(), AVAILABLE_ROOT_NOTES.end(), currentRootNote);
    if (it != AVAILABLE_ROOT_NOTES.end()) {
        // Move to next root note, wrap around if needed
        it++;
        if (it == AVAILABLE_ROOT_NOTES.end()) {
            it = AVAILABLE_ROOT_NOTES.begin();
        }
        currentRootNote = *it;
        std::cout << "Cycled to root note: " << currentRootNote << std::endl;
        update_synth_configuration();
    }
}

void MPU6050::cycle_scale_type() {
    // Get list of scale type names
    std::vector<std::string> scaleNames;
    for (const auto& scale : AVAILABLE_SCALES) {
        scaleNames.push_back(scale.first);
    }
    
    // Find current scale type in the list
    auto it = std::find(scaleNames.begin(), scaleNames.end(), currentScaleType);
    if (it != scaleNames.end()) {
        // Move to next scale type, wrap around if needed
        it++;
        if (it == scaleNames.end()) {
            it = scaleNames.begin();
        }
        currentScaleType = *it;
        std::cout << "Cycled to scale type: " << currentScaleType << std::endl;
        update_synth_configuration();
    }
}

void MPU6050::cycle_instrument_type() {
    // Get list of instrument type names
    std::vector<std::string> instrumentNames;
    for (const auto& instrument : AVAILABLE_INSTRUMENTS) {
        instrumentNames.push_back(instrument.first);
    }
    
    // Find current instrument type in the list
    auto it = std::find(instrumentNames.begin(), instrumentNames.end(), currentInstrumentType);
    if (it != instrumentNames.end()) {
        // Move to next instrument type, wrap around if needed
        it++;
        if (it == instrumentNames.end()) {
            it = instrumentNames.begin();
        }
        currentInstrumentType = *it;
        std::cout << "Cycled to instrument type: " << currentInstrumentType << std::endl;
        update_synth_configuration();
    }
}

void MPU6050::display_current_configuration() const {
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "SENSOR " << (address == MPU6050Constants::ADDR1 ? "1" : "2") << " CONFIGURATION" << std::endl;
    std::cout << "Root Note: " << currentRootNote << std::endl;
    std::cout << "Scale Type: " << currentScaleType << std::endl;
    std::cout << "Instrument: " << currentInstrumentType << std::endl;
    std::cout << "-------------------------------------" << std::endl;
}

void MPU6050::update_synth_configuration() {
    if (!synth) {
        return;
    }
    
    // Get scale type enum from name
    ScaleType scaleType = AVAILABLE_SCALES.at(currentScaleType);
    
    // Get instrument type enum from name
    InstrumentType instrumentType = AVAILABLE_INSTRUMENTS.at(currentInstrumentType);
    
    // Update synthesizer configuration
    synth->setScale(scaleType, currentRootNote);
    synth->setInstrument(instrumentType);
    
    // Display the new configuration
    display_current_configuration();
}

// Button interrupt handler implementation
void button_interrupt(MPU6050& sensor, int button_gpio) {
    struct gpiod_chip* chip = gpiod_chip_open_by_name(GPIO_CHIP);
    if (!chip) {
        std::cerr << "Failed to open GPIO chip" << std::endl;
        return;
    }
    
    struct gpiod_line* line = gpiod_chip_get_line(chip, button_gpio);
    if (!line) {
        std::cerr << "Failed to get GPIO line" << std::endl;
        gpiod_chip_close(chip);
        return;
    }
    
    struct gpiod_line_request_config config = {
        "button_monitor",
        GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE,
        GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
    };
    
    if (gpiod_line_request(line, &config, 0) < 0) {
        std::cerr << "Failed to request GPIO line" << std::endl;
        gpiod_chip_close(chip);
        return;
    }
    
    while (true) {
        struct gpiod_line_event event;
        if (gpiod_line_event_wait(line, nullptr) > 0 && 
            gpiod_line_event_read(line, &event) == 0) {
            if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
                sensor.set_active(true);
                std::cout << "Button pressed - activating synth" << std::endl;
                while (gpiod_line_get_value(line) == 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                std::cout << "Button released - deactivating synth" << std::endl;
                sensor.set_active(false);
            }
        }
    }
    
    gpiod_chip_close(chip);
}

// Signal handler implementation
std::atomic<bool> keep_running(true);

void signal_handler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    keep_running = false;
}

// Main function
int main() {
    signal(SIGINT, signal_handler);
    
    try {
        // Create and initialize sensors
        MPU6050 sensor1(MPU6050Constants::ADDR1);
        MPU6050 sensor2(MPU6050Constants::ADDR2);
        
        sensor1.calibrate_z_axis();
        sensor2.calibrate_z_axis();
        
        // Set initial configurations for each sensor
        sensor1.currentRootNote = "C";
        sensor1.currentScaleType = "Major";
        sensor1.currentInstrumentType = "Piano";
        
        sensor2.currentRootNote = "A";
        sensor2.currentScaleType = "Minor";
        sensor2.currentInstrumentType = "Moog";
        
        // Initialize synthesizers
        sensor1.attach_synth(sensor1.currentRootNote, 
                            AVAILABLE_SCALES.at(sensor1.currentScaleType), 
                            AVAILABLE_INSTRUMENTS.at(sensor1.currentInstrumentType));
        
        sensor2.attach_synth(sensor2.currentRootNote, 
                            AVAILABLE_SCALES.at(sensor2.currentScaleType), 
                            AVAILABLE_INSTRUMENTS.at(sensor2.currentInstrumentType));
        
        sensor1.start_monitoring();
        sensor2.start_monitoring();
        
        std::thread(button_interrupt, std::ref(sensor1), MPU6050Constants::BUTTON_GPIO1).detach();
        std::thread(button_interrupt, std::ref(sensor2), MPU6050Constants::BUTTON_GPIO2).detach();
        
        std::cout << "MPU6050 Music Synthesizer Ready" << std::endl;
        std::cout << "Press Ctrl+C to exit" << std::endl;
        std::cout << std::endl;
        std::cout << "Motion Controls:" << std::endl;
        std::cout << "- Strong X-axis motion: Change scale type" << std::endl;
        std::cout << "- Strong Y-axis motion: Change root note" << std::endl;
        std::cout << "- Strong Z-axis motion: Change instrument" << std::endl;
        std::cout << std::endl;
        std::cout << "Sensor 1 Initial Config: " << sensor1.currentRootNote << " " 
                  << sensor1.currentScaleType << " with " << sensor1.currentInstrumentType << std::endl;
        std::cout << "Sensor 2 Initial Config: " << sensor2.currentRootNote << " " 
                  << sensor2.currentScaleType << " with " << sensor2.currentInstrumentType << std::endl;
        
        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        std::cout << "Shutting down..." << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}