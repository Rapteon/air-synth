/**
 * @file GestureDetector.cpp
 * @brief Implementation of real-time gesture detection
 * @author Mohammed
 * @date Feb 25, 2025
 */

#include "GestureDetector.hpp"
#include <cmath>
#include <iostream>
#include <numeric>
#include <algorithm>
#include <sstream>
#include <iomanip>

// Convert gesture type to string
std::string gestureToString(GestureType gesture) {
    switch (gesture) {
        case GestureType::NONE:          return "NONE";
        case GestureType::TILT_LEFT:     return "TILT_LEFT";
        case GestureType::TILT_RIGHT:    return "TILT_RIGHT";
        case GestureType::TILT_FORWARD:  return "TILT_FORWARD";
        case GestureType::TILT_BACKWARD: return "TILT_BACKWARD";
        case GestureType::SHAKE_X:       return "SHAKE_X";
        case GestureType::SHAKE_Y:       return "SHAKE_Y";
        case GestureType::SHAKE_Z:       return "SHAKE_Z";
        case GestureType::PUNCH:         return "PUNCH";
        case GestureType::SWIPE_LEFT:    return "SWIPE_LEFT";
        case GestureType::SWIPE_RIGHT:   return "SWIPE_RIGHT";
        case GestureType::CIRCULAR:      return "CIRCULAR";
        default:                         return "UNKNOWN";
    }
}

GestureDetector::GestureDetector(float sampleRate, float windowSize)
    : _sampleRate(sampleRate)
    , _windowSize(windowSize)
    , _windowSamples(static_cast<size_t>(sampleRate * windowSize))
    , _lastGesture(GestureType::NONE)
    , _lastGestureTime(std::chrono::high_resolution_clock::now())
    , _gravityVector({0.0f, 0.0f, 1.0f})
    , _isCalibrated(false) {
    
    // Initialize thresholds
    initializeThresholds();
    
    // Reserve buffer space
    const size_t reserveSize = std::max<size_t>(_windowSamples * 2, 1000);
    _accelBuffer.reserve(reserveSize);
    _gyroBuffer.reserve(reserveSize);
    _timestampBuffer.reserve(reserveSize);
}

void GestureDetector::addSensorData(const MPU6050::SensorData& data) {
    std::lock_guard<std::mutex> lock(_bufferMutex);
    
    // Add accelerometer data to buffer
    _accelBuffer.push_back({data.ax, data.ay, data.az});
    
    // Add gyroscope data to buffer
    _gyroBuffer.push_back({data.gx, data.gy, data.gz});
    
    // Add timestamp to buffer
    _timestampBuffer.push_back(data.timestamp);
    
    // Keep buffer size limited
    while (_accelBuffer.size() > _windowSamples) {
        _accelBuffer.pop_front();
        _gyroBuffer.pop_front();
        _timestampBuffer.pop_front();
    }
}

bool GestureDetector::calibrate(float duration) {
    std::cout << "Calibrating gesture detector..." << std::endl;
    std::cout << "Keep the sensor in its neutral position." << std::endl;
    
    // Get current time
    auto startTime = std::chrono::high_resolution_clock::now();
    auto endTime = startTime + std::chrono::duration<float>(duration);
    
    // Wait for enough data
    while (std::chrono::high_resolution_clock::now() < endTime) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::lock_guard<std::mutex> lock(_bufferMutex);
        if (_accelBuffer.size() >= static_cast<size_t>(duration * _sampleRate * 0.8f)) {
            break;
        }
    }
    
    // Process calibration data
    std::lock_guard<std::mutex> lock(_bufferMutex);
    
    if (_accelBuffer.size() < 10) {
        std::cerr << "Not enough data for calibration!" << std::endl;
        return false;
    }
    
    // Calculate average acceleration vector (should be gravity)
    std::array<float, 3> sumAccel = {0.0f, 0.0f, 0.0f};
    
    for (const auto& accel : _accelBuffer) {
        sumAccel[0] += accel[0];
        sumAccel[1] += accel[1];
        sumAccel[2] += accel[2];
    }
    
    std::array<float, 3> avgAccel = {
        sumAccel[0] / _accelBuffer.size(),
        sumAccel[1] / _accelBuffer.size(),
        sumAccel[2] / _accelBuffer.size()
    };
    
    // Normalize to unit vector
    float norm = magnitude(avgAccel);
    if (norm > 0.5f) {  // Check for reasonable magnitude
        _gravityVector = {
            avgAccel[0] / norm,
            avgAccel[1] / norm,
            avgAccel[2] / norm
        };
        _isCalibrated = true;
        
        std::cout << "Calibration complete. Gravity vector: ["
                  << _gravityVector[0] << ", "
                  << _gravityVector[1] << ", "
                  << _gravityVector[2] << "]" << std::endl;
        return true;
    } else {
        std::cerr << "Calibration failed: gravity vector too weak" << std::endl;
        return false;
    }
}

GestureType GestureDetector::detectGesture() {
    // Check if we have enough data
    {
        std::lock_guard<std::mutex> lock(_bufferMutex);
        if (_accelBuffer.size() < _windowSamples / 2) {
            return GestureType::NONE;
        }
    }
    
    // Check debounce time
    if (!checkDebounceTime()) {
        return GestureType::NONE;
    }
    
    // Get data from buffer with lock
    std::vector<std::array<float, 3>> accelData;
    std::vector<std::array<float, 3>> gyroData;
    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> timestamps;
    
    {
        std::lock_guard<std::mutex> lock(_bufferMutex);
        accelData.assign(_accelBuffer.begin(), _accelBuffer.end());
        gyroData.assign(_gyroBuffer.begin(), _gyroBuffer.end());
        timestamps.assign(_timestampBuffer.begin(), _timestampBuffer.end());
    }
    
    // DETECTION LOGIC
    
    // Get the most recent samples (last 0.1s)
    size_t recentSamples = std::min(static_cast<size_t>(_sampleRate * 0.1f), accelData.size());
    size_t startIdx = accelData.size() - recentSamples;
    
    // Calculate mean accelerometer data (for tilt detection)
    std::array<float, 3> accelMean = {0.0f, 0.0f, 0.0f};
    for (size_t i = startIdx; i < accelData.size(); ++i) {
        accelMean[0] += accelData[i][0];
        accelMean[1] += accelData[i][1];
        accelMean[2] += accelData[i][2];
    }
    accelMean[0] /= recentSamples;
    accelMean[1] /= recentSamples;
    accelMean[2] /= recentSamples;
    
    // Find maximum absolute values of accelerometer and gyroscope data
    std::array<float, 3> accelMax = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyroMax = {0.0f, 0.0f, 0.0f};
    
    for (size_t i = startIdx; i < accelData.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            accelMax[j] = std::max(accelMax[j], std::abs(accelData[i][j]));
            gyroMax[j] = std::max(gyroMax[j], std::abs(gyroData[i][j]));
        }
    }
    
    // Calculate tilt relative to gravity
    float tiltNorm = magnitude(accelMean);
    GestureType detectedGesture = GestureType::NONE;
    
    if (tiltNorm > 0.5f) {  // Check for valid acceleration
        // Normalize the acceleration vector
        std::array<float, 3> tiltVector = normalize(accelMean);
        
        // Calculate angle between tilt vector and gravity
        float dotProd = dotProduct(tiltVector, _gravityVector);
        float tiltAngle = std::acos(std::clamp(dotProd, -1.0f, 1.0f));
        
        // Calculate direction of tilt (cross product with gravity)
        std::array<float, 3> tiltDirection = crossProduct(_gravityVector, tiltVector);
        
        // SHAKE DETECTION (high gyroscope values)
        if (gyroMax[0] > _thresholds["shake"]) {
            detectedGesture = GestureType::SHAKE_X;
        }
        else if (gyroMax[1] > _thresholds["shake"]) {
            detectedGesture = GestureType::SHAKE_Y;
        }
        else if (gyroMax[2] > _thresholds["shake"]) {
            detectedGesture = GestureType::SHAKE_Z;
        }
        
        // PUNCH DETECTION (rapid acceleration spike on z-axis)
        else if (accelMax[2] > _thresholds["punch"]) {
            // Check if it's a quick motion by looking at the difference
            if (accelData.size() >= 10) {
                std::vector<float> accelDiff;
                for (size_t i = accelData.size() - std::min<size_t>(10, accelData.size());
                     i < accelData.size() - 1; ++i) {
                    accelDiff.push_back(accelData[i+1][2] - accelData[i][2]);
                }
                
                float maxDiff = 0.0f;
                for (float diff : accelDiff) {
                    maxDiff = std::max(maxDiff, std::abs(diff));
                }
                
                if (maxDiff > 0.8f) {
                    detectedGesture = GestureType::PUNCH;
                }
            }
        }
        
        // TILT DETECTION
        else if (tiltAngle > 0.3f) {  // About 17 degrees tilt
            // Determine the primary direction of tilt
            if (std::abs(tiltDirection[0]) > std::abs(tiltDirection[1])) {
                // Tilting around X axis (forward/backward)
                if (tiltDirection[0] > _thresholds["tilt"]) {
                    detectedGesture = GestureType::TILT_FORWARD;
                }
                else if (tiltDirection[0] < -_thresholds["tilt"]) {
                    detectedGesture = GestureType::TILT_BACKWARD;
                }
            }
            else {
                // Tilting around Y axis (left/right)
                if (tiltDirection[1] > _thresholds["tilt"]) {
                    detectedGesture = GestureType::TILT_RIGHT;
                }
                else if (tiltDirection[1] < -_thresholds["tilt"]) {
                    detectedGesture = GestureType::TILT_LEFT;
                }
            }
        }
        
        // SWIPE DETECTION
        else if (accelData.size() >= 10 && timestamps.size() >= 10) {
            // Calculate velocity by integrating acceleration
            std::vector<float> velocity_x;
            for (size_t i = accelData.size() - 10; i < accelData.size() - 1; ++i) {
                auto dt = std::chrono::duration<float>(
                    timestamps[i+1] - timestamps[i]).count();
                velocity_x.push_back(accelData[i][0] * dt);
            }
            
            // Simple cumulative sum for integration
            std::vector<float> cum_velocity(velocity_x.size());
            std::partial_sum(velocity_x.begin(), velocity_x.end(), cum_velocity.begin());
            
            // Check for swipe thresholds
            float maxVel = *std::max_element(cum_velocity.begin(), cum_velocity.end());
            float minVel = *std::min_element(cum_velocity.begin(), cum_velocity.end());
            
            if (maxVel > _thresholds["swipe"]) {
                detectedGesture = GestureType::SWIPE_RIGHT;
            }
            else if (minVel < -_thresholds["swipe"]) {
                detectedGesture = GestureType::SWIPE_LEFT;
            }
        }
        
        // CIRCULAR MOTION DETECTION
        else if (gyroData.size() >= 20) {
            // Check for consistent rotation around z-axis
            std::vector<float> gyro_z;
            for (size_t i = gyroData.size() - 20; i < gyroData.size(); ++i) {
                gyro_z.push_back(gyroData[i][2]);
            }
            
            // Calculate mean and standard deviation
            float mean = std::accumulate(gyro_z.begin(), gyro_z.end(), 0.0f) / gyro_z.size();
            
            float variance = 0.0f;
            for (float val : gyro_z) {
                variance += (val - mean) * (val - mean);
            }
            variance /= gyro_z.size();
            float stdDev = std::sqrt(variance);
            
            if ((mean > 50.0f && stdDev < 30.0f) || 
                (mean < -50.0f && stdDev < 30.0f)) {
                detectedGesture = GestureType::CIRCULAR;
            }
        }
    }
    
    // Update last gesture and timestamp if a gesture was detected
    if (detectedGesture != GestureType::NONE) {
        _lastGesture = detectedGesture;
        _lastGestureTime = std::chrono::high_resolution_clock::now();
    }
    
    return detectedGesture;
}

GestureType GestureDetector::getLastGesture() const {
    return _lastGesture;
}

void GestureDetector::adjustThreshold(const std::string& thresholdName, float value) {
    if (_thresholds.find(thresholdName) != _thresholds.end()) {
        _thresholds[thresholdName] = value;
        std::cout << "Adjusted threshold '" << thresholdName 
                  << "' to " << value << std::endl;
    } else {
        std::cerr << "Unknown threshold: " << thresholdName << std::endl;
    }
}

float GestureDetector::getThreshold(const std::string& thresholdName) const {
    auto it = _thresholds.find(thresholdName);
    if (it != _thresholds.end()) {
        return it->second;
    }
    return -1.0f;  // Indicates not found
}

std::string GestureDetector::getAllThresholds() const {
    std::ostringstream oss;
    oss << "{";
    
    bool first = true;
    for (const auto& [name, value] : _thresholds) {
        if (!first) {
            oss << ", ";
        }
        oss << "\"" << name << "\": " << std::fixed << std::setprecision(2) << value;
        first = false;
    }
    
    oss << "}";
    return oss.str();
}

void GestureDetector::clearBuffer() {
    std::lock_guard<std::mutex> lock(_bufferMutex);
    _accelBuffer.clear();
    _gyroBuffer.clear();
    _timestampBuffer.clear();
}

void GestureDetector::setDebounceTime(float debounceTime) {
    adjustThreshold("debounce", debounceTime);
}

float GestureDetector::getDebounceTime() const {
    return getThreshold("debounce");
}

std::array<float, 3> GestureDetector::applyFilter(
    const std::array<float, 3>& data,
    const std::deque<std::array<float, 3>>& filterBuffer,
    float alpha) const {
    
    if (filterBuffer.empty()) {
        return data;
    }
    
    const auto& prev = filterBuffer.back();
    return {
        prev[0] * (1.0f - alpha) + data[0] * alpha,
        prev[1] * (1.0f - alpha) + data[1] * alpha,
        prev[2] * (1.0f - alpha) + data[2] * alpha
    };
}

float GestureDetector::magnitude(const std::array<float, 3>& vec) const {
    return std::sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

float GestureDetector::dotProduct(const std::array<float, 3>& a, 
                                 const std::array<float, 3>& b) const {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

std::array<float, 3> GestureDetector::crossProduct(
    const std::array<float, 3>& a, 
    const std::array<float, 3>& b) const {
    
    return {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

std::array<float, 3> GestureDetector::normalize(const std::array<float, 3>& vec) const {
    float mag = magnitude(vec);
    if (mag < 1e-6f) {
        return {0.0f, 0.0f, 0.0f};
    }
    
    return {
        vec[0] / mag,
        vec[1] / mag,
        vec[2] / mag
    };
}

bool GestureDetector::checkDebounceTime() const {
    auto currentTime = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(
        currentTime - _lastGestureTime).count();
    
    return elapsed >= _thresholds.at("debounce");
}

void GestureDetector::initializeThresholds() {
    _thresholds["tilt"] = 0.8f;        // Threshold for tilt detection
    _thresholds["shake"] = 150.0f;     // Threshold for shake detection (deg/s)
    _thresholds["punch"] = 1.5f;       // Threshold for punch detection (g)
    _thresholds["swipe"] = 1.2f;       // Threshold for swipe detection
    _thresholds["debounce"] = 0.3f;    // Debounce time (seconds)
}