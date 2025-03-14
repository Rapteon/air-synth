/**
 * @file AirSynth.hpp
 * @brief Main interface for the Air Synth application
 * @author Mohammed
 * @date Feb 25, 2025
 */

#ifndef AIR_SYNTH_HPP
#define AIR_SYNTH_HPP

#include "MPU6050.hpp"
#include "GestureDetector.hpp"
#include "Synthesizer.hpp"
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <queue>
#include <chrono>
#include <functional>

/**
 * @class AirSynth
 * @brief Main application class for the Air Synth gesture-controlled synthesizer
 */
class AirSynth {
public:
    /**
     * @brief Constructor
     */
    AirSynth();
    
    /**
     * @brief Destructor
     */
    ~AirSynth();
    
    /**
     * @brief Initialize the application
     * @return True if initialization successful
     */
    bool initialize();
    
    /**
     * @brief Update the application state (called in main loop)
     */
    void update();
    
    /**
     * @brief Shut down the application
     */
    void shutdown();
    
    /**
     * @brief Set the application running state
     * @param running True to run, false to stop
     */
    void setRunning(bool running);
    
    /**
     * @brief Check if the application is running
     * @return True if running
     */
    bool isRunning() const;
    
    /**
     * @brief Process a command from the user interface
     * @param command The command string
     * @return True if the command was processed
     */
    bool processCommand(const std::string& command);
    
    /**
     * @brief Get performance metrics as a JSON-formatted string
     * @return JSON string with performance data
     */
    std::string getPerformanceMetrics() const;

private:
    // Core components
    std::unique_ptr<MPU6050> _mpu;
    std::unique_ptr<GestureDetector> _gestureDetector;
    std::unique_ptr<Synthesizer> _synthesizer;
    
    // Application state
    std::atomic<bool> _running;
    std::atomic<bool> _initialized;
    
    // GPIO management
    int _gpioLedPin;
    std::vector<int> _gpioButtonPins;
    std::vector<bool> _gpioButtonStates;
    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> _gpioButtonDebounce;
    
    // Threading for real-time processing
    std::unique_ptr<std::thread> _sensorThread;
    std::unique_ptr<std::thread> _gestureThread;
    std::unique_ptr<std::thread> _audioThread;
    std::unique_ptr<std::thread> _displayThread;
    
    // Thread communication
    struct SensorData {
        MPU6050::SensorData rawData;
        GestureType gesture;
    };
    
    std::queue<SensorData> _sensorQueue;
    std::mutex _sensorQueueMutex;
    
    // Performance metrics
    struct PerformanceMetrics {
        float sensorRate;
        float gestureRate;
        float audioRate;
        float cpuUsage;
        float latency;
        uint64_t missedFrames;
        uint64_t totalFrames;
    };
    
    PerformanceMetrics _metrics;
    std::mutex _metricsMutex;
    
    // Thread functions
    void sensorThreadFunc();
    void gestureThreadFunc();
    void audioThreadFunc();
    void displayThreadFunc();
    
    // Helper functions
    void setupGPIO();
    void cleanupGPIO();
    void updateGPIO();
    void blinkLED(int count, int durationMs);
    void updatePerformanceMetrics();
    void printStatus() const;
    void handleGesture(GestureType gesture);
};

#endif // AIR_SYNTH_HPP