/**
 * @file AirSynth.cpp
 * @brief Implementation of the main Air Synth application
 * @author Your Name
 * @date March 14, 2025
 */

#include "AirSynth.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>

// Include platform-specific GPIO handling
#ifdef RASPBERRY_PI
#include <pigpio.h>
#define GPIO_API_AVAILABLE true
#else
#define GPIO_API_AVAILABLE false
#endif

// GPIO pin definitions
#define LED_PIN 17
#define MODE_BUTTON_PIN 27
#define VOLUME_UP_PIN 22
#define VOLUME_DOWN_PIN 23
#define SOUND_TOGGLE_PIN 24

// Button debounce time in milliseconds
#define BUTTON_DEBOUNCE_MS 50

// Thread update rates
#define SENSOR_UPDATE_RATE_HZ 100
#define GESTURE_UPDATE_RATE_HZ 50
#define DISPLAY_UPDATE_RATE_HZ 5

// Maximum queue size for thread communication
#define MAX_QUEUE_SIZE 100

AirSynth::AirSynth()
    : _running(false)
    , _initialized(false)
    , _gpioLedPin(LED_PIN)
    , _gpioButtonPins({MODE_BUTTON_PIN, VOLUME_UP_PIN, VOLUME_DOWN_PIN, SOUND_TOGGLE_PIN})
    , _gpioButtonStates(_gpioButtonPins.size(), false)
    , _gpioButtonDebounce(_gpioButtonPins.size())
    , _metrics{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0} {
    
    // Initialize button debounce timestamps
    auto now = std::chrono::high_resolution_clock::now();
    for (auto& ts : _gpioButtonDebounce) {
        ts = now;
    }
}

AirSynth::~AirSynth() {
    // Ensure shutdown is called
    if (_initialized) {
        shutdown();
    }
}

bool AirSynth::initialize() {
    std::cout << "Initializing Air Synth..." << std::endl;
    
    try {
        // Set up GPIO
        setupGPIO();
        
        // Create components
        _mpu = std::make_unique<MPU6050>();
        _gestureDetector = std::make_unique<GestureDetector>();
        _synthesizer = std::make_unique<Synthesizer>();
        
        // Initialize MPU6050
        std::cout << "Initializing MPU6050 sensor..." << std::endl;
        if (!_mpu->initialize()) {
            std::cerr << "Failed to initialize MPU6050" << std::endl;
            return false;
        }
        
        // Start continuous sampling
        if (!_mpu->startSampling()) {
            std::cerr << "Failed to start MPU6050 sampling" << std::endl;
            return false;
        }
        
        // Wait for sensor to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Calibrate gyroscope
        std::cout << "Calibrating gyroscope..." << std::endl;
        blinkLED(2, 200);  // Blink to indicate calibration start
        if (!_mpu->calibrateGyro()) {
            std::cerr << "Gyroscope calibration failed" << std::endl;
            return false;
        }
        
        // Collect initial data for gesture detector
        std::cout << "Collecting initial sensor data..." << std::endl;
        for (int i = 0; i < 50; ++i) {
            auto data = _mpu->getLatestData(1);
            if (!data.empty()) {
                _gestureDetector->addSensorData(data[0]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Calibrate gesture detector
        std::cout << "Calibrating gesture detector..." << std::endl;
        blinkLED(3, 200);  // Blink to indicate calibration start
        if (!_gestureDetector->calibrate()) {
            std::cerr << "Warning: Gesture detector calibration failed, using defaults" << std::endl;
        }
        
        // Initialize synthesizer
        std::cout << "Initializing audio synthesizer..." << std::endl;
        if (!_synthesizer->initialize()) {
            std::cerr << "Failed to initialize synthesizer" << std::endl;
            return false;
        }
        
        // Start real-time processing threads
        _running = true;
        _sensorThread = std::make_unique<std::thread>(&AirSynth::sensorThreadFunc, this);
        _gestureThread = std::make_unique<std::thread>(&AirSynth::gestureThreadFunc, this);
        _audioThread = std::make_unique<std::thread>(&AirSynth::audioThreadFunc, this);
        _displayThread = std::make_unique<std::thread>(&AirSynth::displayThreadFunc, this);
        
        // Set thread priorities if available
        #ifdef RASPBERRY_PI
        // Set thread priorities for real-time performance
        // This requires appropriate permissions
        #endif
        
        _initialized = true;
        std::cout << "Air Synth initialized successfully" << std::endl;
        blinkLED(5, 100);  // Blink to indicate successful initialization
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Initialization error: " << e.what() << std::endl;
        return false;
    }
}

void AirSynth::update() {
    if (!_initialized) {
        return;
    }
    
    // Process GPIO inputs
    updateGPIO();
    
    // Update performance metrics
    updatePerformanceMetrics();
}

void AirSynth::shutdown() {
    std::cout << "Shutting down Air Synth..." << std::endl;
    
    // Stop threads
    _running = false;
    
    if (_sensorThread && _sensorThread->joinable()) {
        _sensorThread->join();
    }
    
    if (_gestureThread && _gestureThread->joinable()) {
        _gestureThread->join();
    }
    
    if (_audioThread && _audioThread->joinable()) {
        _audioThread->join();
    }
    
    if (_displayThread && _displayThread->joinable()) {
        _displayThread->join();
    }
    
    // Shutdown components
    if (_synthesizer) {
        _synthesizer->shutdown();
    }
    
    if (_mpu) {
        _mpu->stopSampling();
    }
    
    // Clean up GPIO
    cleanupGPIO();
    
    _initialized = false;
    std::cout << "Air Synth shutdown complete" << std::endl;
}

void AirSynth::setRunning(bool running) {
    _running = running;
}

bool AirSynth::isRunning() const {
    return _running;
}

bool AirSynth::processCommand(const std::string& command) {
    if (command.empty()) {
        return false;
    }
    
    // Process command
    if (command == "quit" || command == "exit") {
        _running = false;
        return true;
    }
    else if (command == "status") {
        printStatus();
        return true;
    }
    else if (command == "help") {
        std::cout << "Available commands:" << std::endl;
        std::cout << "  help                 - Show this help" << std::endl;
        std::cout << "  status               - Show current status" << std::endl;
        std::cout << "  quit, exit           - Exit the application" << std::endl;
        std::cout << "  mode <0-3>           - Set synthesizer mode" << std::endl;
        std::cout << "  volume <0.0-1.0>     - Set volume level" << std::endl;
        std::cout << "  waveform <0-4>       - Set waveform type" << std::endl;
        std::cout << "  calibrate            - Recalibrate sensors" << std::endl;
        return true;
    }
    else if (command.substr(0, 5) == "mode ") {
        try {
            int mode = std::stoi(command.substr(5));
            _synthesizer->setMode(static_cast<SynthMode>(mode));
            std::cout << "Mode set to " << mode << std::endl;
            return true;
        } catch (...) {
            std::cerr << "Invalid mode format. Use 'mode <0-3>'" << std::endl;
            return false;
        }
    }
    else if (command.substr(0, 7) == "volume ") {
        try {
            float volume = std::stof(command.substr(7));
            _synthesizer->setVolume(volume);
            std::cout << "Volume set to " << volume << std::endl;
            return true;
        } catch (...) {
            std::cerr << "Invalid volume format. Use 'volume <0.0-1.0>'" << std::endl;
            return false;
        }
    }
    else if (command.substr(0, 9) == "waveform ") {
        try {
            int waveform = std::stoi(command.substr(9));
            _synthesizer->setWaveform(static_cast<WaveformType>(waveform));
            std::cout << "Waveform set to " << waveform << std::endl;
            return true;
        } catch (...) {
            std::cerr << "Invalid waveform format. Use 'waveform <0-4>'" << std::endl;
            return false;
        }
    }
    else if (command == "calibrate") {
        std::cout << "Recalibrating sensors..." << std::endl;
        blinkLED(2, 200);
        _mpu->calibrateGyro();
        _gestureDetector->calibrate();
        std::cout << "Calibration complete" << std::endl;
        return true;
    }
    
    std::cerr << "Unknown command: " << command << std::endl;
    return false;
}

std::string AirSynth::getPerformanceMetrics() const {
    std::lock_guard<std::mutex> lock(_metricsMutex);
    
    std::ostringstream oss;
    oss << "{";
    oss << "\"sensorRate\": " << std::fixed << std::setprecision(1) << _metrics.sensorRate << ", ";
    oss << "\"gestureRate\": " << std::fixed << std::setprecision(1) << _metrics.gestureRate << ", ";
    oss << "\"audioRate\": " << std::fixed << std::setprecision(1) << _metrics.audioRate << ", ";
    oss << "\"cpuUsage\": " << std::fixed << std::setprecision(1) << _metrics.cpuUsage << ", ";
    oss << "\"latency\": " << std::fixed << std::setprecision(2) << _metrics.latency << ", ";
    oss << "\"missedFrames\": " << _metrics.missedFrames << ", ";
    oss << "\"totalFrames\": " << _metrics.totalFrames;
    oss << "}";
    
    return oss.str();
}

void AirSynth::sensorThreadFunc() {
    const auto intervalUs = std::chrono::microseconds(1000000 / SENSOR_UPDATE_RATE_HZ);
    auto nextSampleTime = std::chrono::high_resolution_clock::now();
    uint64_t frameCount = 0;
    
    std::cout << "Sensor thread started" << std::endl;
    
    while (_running) {
        auto cycleStart = std::chrono::high_resolution_clock::now();
        
        try {
            // Get latest sensor data
            auto data = _mpu->getLatestData(1);
            if (!data.empty()) {
                // Add data to gesture detector
                _gestureDetector->addSensorData(data[0]);
                
                // Push data to queue for other threads
                SensorData sensorData = {data[0], GestureType::NONE};
                
                {
                    std::lock_guard<std::mutex> lock(_sensorQueueMutex);
                    
                    // Limit queue size
                    if (_sensorQueue.size() >= MAX_QUEUE_SIZE) {
                        _sensorQueue.pop();
                        
                        std::lock_guard<std::mutex> metricsLock(_metricsMutex);
                        _metrics.missedFrames++;
                    }
                    
                    _sensorQueue.push(sensorData);
                }
                
                // Update frame count
                frameCount++;
                
                {
                    std::lock_guard<std::mutex> lock(_metricsMutex);
                    _metrics.totalFrames = frameCount;
                }
            }
            
            // Calculate processing time
            auto processingTime = std::chrono::high_resolution_clock::now() - cycleStart;
            
            // Sleep until next sample time
            nextSampleTime += intervalUs;
            auto sleepTime = nextSampleTime - std::chrono::high_resolution_clock::now();
            
            if (sleepTime.count() > 0) {
                std::this_thread::sleep_for(sleepTime);
            } else {
                // We're falling behind schedule
                nextSampleTime = std::chrono::high_resolution_clock::now() + intervalUs;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error in sensor thread: " << e.what() << std::endl;
        }
    }
    
    std::cout << "Sensor thread stopped" << std::endl;
}

void AirSynth::gestureThreadFunc() {
    const auto intervalUs = std::chrono::microseconds(1000000 / GESTURE_UPDATE_RATE_HZ);
    auto nextUpdateTime = std::chrono::high_resolution_clock::now();
    
    std::cout << "Gesture thread started" << std::endl;
    
    while (_running) {
        auto cycleStart = std::chrono::high_resolution_clock::now();
        
        try {
            // Detect gestures
            GestureType gesture = _gestureDetector->detectGesture();
            
            // Handle detected gesture
            if (gesture != GestureType::NONE) {
                handleGesture(gesture);
            }
            
            // Sleep until next update time
            nextUpdateTime += intervalUs;
            auto sleepTime = nextUpdateTime - std::chrono::high_resolution_clock::now();
            
            if (sleepTime.count() > 0) {
                std::this_thread::sleep_for(sleepTime);
            } else {
                // We're falling behind schedule
                nextUpdateTime = std::chrono::high_resolution_clock::now() + intervalUs;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error in gesture thread: " << e.what() << std::endl;
        }
    }
    
    std::cout << "Gesture thread stopped" << std::endl;
}

void AirSynth::audioThreadFunc() {
    std::cout << "Audio thread started" << std::endl;
    
    // The synthesizer handles its own timing internally
    while (_running) {
        // Sleep to avoid hogging CPU, let the synthesizer do its work
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "Audio thread stopped" << std::endl;
}

void AirSynth::displayThreadFunc() {
    const auto interval = std::chrono::milliseconds(1000 / DISPLAY_UPDATE_RATE_HZ);
    
    std::cout << "Display thread started" << std::endl;
    
    while (_running) {
        try {
            // Print status periodically
            printStatus();
            
            // Sleep until next update
            std::this_thread::sleep_for(interval);
            
        } catch (const std::exception& e) {
            std::cerr << "Error in display thread: " << e.what() << std::endl;
        }
    }
    
    std::cout << "Display thread stopped" << std::endl;
}

void AirSynth::setupGPIO() {
    if (!GPIO_API_AVAILABLE) {
        std::cout << "GPIO API not available on this platform" << std::endl;
        return;
    }
    
    #ifdef RASPBERRY_PI
    if (gpioInitialise() < 0) {
        throw std::runtime_error("Failed to initialize GPIO");
    }
    
    // Set up LED output
    gpioSetMode(_gpioLedPin, PI_OUTPUT);
    gpioWrite(_gpioLedPin, 0);
    
    // Set up button inputs with pull-up resistors
    for (int pin : _gpioButtonPins) {
        gpioSetMode(pin, PI_INPUT);
        gpioSetPullUpDown(pin, PI_PUD_UP);
    }
    #endif
    
    std::cout << "GPIO initialized" << std::endl;
}

void AirSynth::cleanupGPIO() {
    if (!GPIO_API_AVAILABLE) {
        return;
    }
    
    #ifdef RASPBERRY_PI
    // Turn off LED
    gpioWrite(_gpioLedPin, 0);
    
    // Terminate GPIO library
    gpioTerminate();
    #endif
    
    std::cout << "GPIO cleaned up" << std::endl;
}

void AirSynth::updateGPIO() {
    if (!GPIO_API_AVAILABLE) {
        return;
    }
    
    #ifdef RASPBERRY_PI
    auto now = std::chrono::high_resolution_clock::now();
    
    // Read button states and handle debouncing
    for (size_t i = 0; i < _gpioButtonPins.size(); ++i) {
        bool state = (gpioRead(_gpioButtonPins[i]) == 0);  // Inverted due to pull-up
        
        // Check for button state change with debouncing
        if (state != _gpioButtonStates[i]) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - _gpioButtonDebounce[i]).count();
                
            if (elapsed >= BUTTON_DEBOUNCE_MS) {
                // Button state changed and debounce time elapsed
                _gpioButtonStates[i] = state;
                _gpioButtonDebounce[i] = now;
                
                // Handle button press (not release)
                if (state) {
                    // Button is pressed
                    switch (_gpioButtonPins[i]) {
                        case MODE_BUTTON_PIN:
                            // Cycle through synthesizer modes
                            _synthesizer->cycleMode();
                            break;
                            
                        case VOLUME_UP_PIN:
                            // Increase volume
                            _synthesizer->adjustVolume(0.1f);
                            break;
                            
                        case VOLUME_DOWN_PIN:
                            // Decrease volume
                            _synthesizer->adjustVolume(-0.1f);
                            break;
                            
                        case SOUND_TOGGLE_PIN:
                            // Toggle sound on/off
                            _synthesizer->toggleSound();
                            break;
                    }
                }
            }
        }
    }
    #endif
}

void AirSynth::blinkLED(int count, int durationMs) {
    if (!GPIO_API_AVAILABLE) {
        return;
    }
    
    #ifdef RASPBERRY_PI
    for (int i = 0; i < count; ++i) {
        gpioWrite(_gpioLedPin, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
        gpioWrite(_gpioLedPin, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
    }
    #endif
}

void AirSynth::updatePerformanceMetrics() {
    std::lock_guard<std::mutex> lock(_metricsMutex);
    
    // Update sensor rate from MPU6050
    _metrics.sensorRate = _mpu->getEffectiveSampleRate();
    
    // Other metrics would be updated from their respective components
    // or calculated based on system measurements
}

void AirSynth::printStatus() const {
    // Clear screen (ANSI escape code)
    std::cout << "\033[2J\033[1;1H";
    
    std::cout << "=== Air Synth Status ===" << std::endl;
    
    // Print synthesizer status
    std::cout << "Mode: " << _synthesizer->getModeString() << std::endl;
    std::cout << "Waveform: " << _synthesizer->getWaveformString() << std::endl;
    std::cout << "Volume: " << std::fixed << std::setprecision(2) << _synthesizer->getVolume() << std::endl;
    std::cout << "Sound: " << (_synthesizer->isSoundEnabled() ? "ON" : "OFF") << std::endl;
    
    // Print last detected gesture
    std::cout << "Last gesture: " << gestureToString(_gestureDetector->getLastGesture()) << std::endl;
    
    // Print performance metrics
    std::cout << "Sensor rate: " << std::fixed << std::setprecision(1) 
              << _metrics.sensorRate << " Hz" << std::endl;
    std::cout << "Missed frames: " << _metrics.missedFrames << " / " 
              << _metrics.totalFrames << std::endl;
    
    // Print controls
    std::cout << "\n=== Controls ===" << std::endl;
    std::cout << "Mode button: Change synthesizer mode" << std::endl;
    std::cout << "Vol+/Vol- buttons: Adjust volume" << std::endl;
    std::cout << "Sound button: Toggle sound on/off" << std::endl;
    std::cout << "Ctrl+C: Exit application" << std::endl;
}

void AirSynth::handleGesture(GestureType gesture) {
    // Process the detected gesture
    std::cout << "Detected gesture: " << gestureToString(gesture) << std::endl;
    
    // Activate LED to indicate gesture detection
    if (GPIO_API_AVAILABLE) {
        #ifdef RASPBERRY_PI
        gpioWrite(_gpioLedPin, 1);
        
        // Use a timer to turn off the LED after a short time
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            gpioWrite(_gpioLedPin, 0);
        }).detach();
        #endif
    }
    
    // Send the gesture to the synthesizer
    _synthesizer->processGesture(gesture);
}