/**
 * @file GestureDetector.hpp
 * @brief Interface for real-time gesture detection
 * @author Mohammed
 * @date Feb 25, 2025
 */

#ifndef GESTURE_DETECTOR_HPP
#define GESTURE_DETECTOR_HPP

#include "MPU6050.hpp"
#include <vector>
#include <array>
#include <deque>
#include <mutex>
#include <chrono>
#include <string>

/**
 * @brief Enumeration of detected gesture types
 */
enum class GestureType {
    NONE,           ///< No gesture detected
    TILT_LEFT,      ///< Tilt left
    TILT_RIGHT,     ///< Tilt right
    TILT_FORWARD,   ///< Tilt forward
    TILT_BACKWARD,  ///< Tilt backward
    SHAKE_X,        ///< Shake along X axis
    SHAKE_Y,        ///< Shake along Y axis
    SHAKE_Z,        ///< Shake along Z axis
    PUNCH,          ///< Punch/thrust motion
    SWIPE_LEFT,     ///< Swipe left motion
    SWIPE_RIGHT,    ///< Swipe right motion
    CIRCULAR        ///< Circular motion
};

/**
 * @brief Convert GestureType to string
 * @param gesture The gesture to convert
 * @return String representation of the gesture
 */
std::string gestureToString(GestureType gesture);

/**
 * @class GestureDetector
 * @brief Class for real-time detection of gestures from MPU6050 sensor data
 */
class GestureDetector {
public:
    /**
     * @brief Constructor
     * @param sampleRate Sensor sampling rate in Hz
     * @param windowSize Time window for gesture detection in seconds
     */
    GestureDetector(float sampleRate = 100.0f, float windowSize = 0.5f);
    
    /**
     * @brief Add sensor data to the detector
     * @param data Sensor data from MPU6050
     */
    void addSensorData(const MPU6050::SensorData& data);
    
    /**
     * @brief Calibrate the detector with the current orientation
     * @param duration Calibration duration in seconds
     * @return True if calibration successful
     */
    bool calibrate(float duration = 2.0f);
    
    /**
     * @brief Detect gestures from the current sensor data buffer
     * @return Detected gesture type
     */
    GestureType detectGesture();
    
    /**
     * @brief Get the last detected gesture
     * @return The last detected gesture
     */
    GestureType getLastGesture() const;
    
    /**
     * @brief Adjust the detection threshold for a specific gesture type
     * @param thresholdName Name of the threshold to adjust
     * @param value New threshold value
     */
    void adjustThreshold(const std::string& thresholdName, float value);
    
    /**
     * @brief Get the current value of a threshold
     * @param thresholdName Name of the threshold
     * @return Current threshold value or -1 if not found
     */
    float getThreshold(const std::string& thresholdName) const;
    
    /**
     * @brief Get a JSON-formatted string of all thresholds
     * @return String with all threshold values
     */
    std::string getAllThresholds() const;
    
    /**
     * @brief Clear the data buffer
     */
    void clearBuffer();
    
    /**
     * @brief Set the detection debounce time
     * @param debounceTime Debounce time in seconds
     */
    void setDebounceTime(float debounceTime);
    
    /**
     * @brief Get the detection debounce time
     * @return Debounce time in seconds
     */
    float getDebounceTime() const;

private:
    // Sensor data buffers
    std::deque<std::array<float, 3>> _accelBuffer;
    std::deque<std::array<float, 3>> _gyroBuffer;
    std::deque<std::chrono::time_point<std::chrono::high_resolution_clock>> _timestampBuffer;
    mutable std::mutex _bufferMutex;
    
    // Gesture detection parameters
    float _sampleRate;
    float _windowSize;
    size_t _windowSamples;
    
    // Thresholds for gesture detection
    std::unordered_map<std::string, float> _thresholds;
    
    // Gesture state
    GestureType _lastGesture;
    std::chrono::time_point<std::chrono::high_resolution_clock> _lastGestureTime;
    
    // Calibration values
    std::array<float, 3> _gravityVector;
    bool _isCalibrated;
    
    /**
     * @brief Apply low-pass filter to sensor readings
     * @param data New sensor data point
     * @param filterBuffer Buffer of filtered data
     * @param alpha Filter coefficient (0.0-1.0)
     * @return Filtered data point
     */
    std::array<float, 3> applyFilter(const std::array<float, 3>& data,
                                     const std::deque<std::array<float, 3>>& filterBuffer,
                                     float alpha) const;
    
    /**
     * @brief Calculate the magnitude of a 3D vector
     * @param vec The vector
     * @return Magnitude (length) of the vector
     */
    float magnitude(const std::array<float, 3>& vec) const;
    
    /**
     * @brief Calculate the dot product of two 3D vectors
     * @param a First vector
     * @param b Second vector
     * @return Dot product
     */
    float dotProduct(const std::array<float, 3>& a, const std::array<float, 3>& b) const;
    
    /**
     * @brief Calculate the cross product of two 3D vectors
     * @param a First vector
     * @param b Second vector
     * @return Cross product
     */
    std::array<float, 3> crossProduct(const std::array<float, 3>& a, 
                                      const std::array<float, 3>& b) const;
    
    /**
     * @brief Normalize a 3D vector to unit length
     * @param vec Vector to normalize
     * @return Normalized vector
     */
    std::array<float, 3> normalize(const std::array<float, 3>& vec) const;
    
    /**
     * @brief Check if enough time has passed since the last gesture detection
     * @return True if debounce time has elapsed
     */
    bool checkDebounceTime() const;
    
    /**
     * @brief Initialize default threshold values
     */
    void initializeThresholds();
};

#endif // GESTURE_DETECTOR_HPP