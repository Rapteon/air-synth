/**
 * @file MPU6050.cpp
 * @brief Implementation of the MPU6050 sensor interface
 * @author Mohammed
 * @date Feb 25, 2025
 */

#include "MPU6050.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <algorithm>

// MPU6050 Register Addresses
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_SMPLRT_DIV     0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CONFIG    0x1B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_FIFO_EN        0x23
#define MPU6050_REG_INT_ENABLE     0x38
#define MPU6050_REG_ACCEL_XOUT_H   0x3B
#define MPU6050_REG_GYRO_XOUT_H    0x43
#define MPU6050_REG_TEMP_OUT_H     0x41
#define MPU6050_REG_FIFO_COUNTH    0x72
#define MPU6050_REG_FIFO_COUNTL    0x73
#define MPU6050_REG_FIFO_R_W       0x74
#define MPU6050_REG_WHO_AM_I       0x75

// Gyroscope Sensitivity Scale Factors
const float GYRO_SCALE_FACTOR[] = {
    131.0f,  // ±250 °/s
    65.5f,   // ±500 °/s
    32.8f,   // ±1000 °/s
    16.4f    // ±2000 °/s
};

// Accelerometer Sensitivity Scale Factors
const float ACCEL_SCALE_FACTOR[] = {
    16384.0f,  // ±2g
    8192.0f,   // ±4g
    4096.0f,   // ±8g
    2048.0f    // ±16g
};

// Buffer size for continuous sampling
const size_t MAX_BUFFER_SIZE = 1000;

MPU6050::MPU6050(const std::string& devicePath, uint8_t address)
    : _i2cFd(-1)
    , _devicePath(devicePath)
    , _address(address)
    , _gyroRange(GyroRange::RANGE_250_DEG)
    , _accelRange(AccelRange::RANGE_2G)
    , _filterBandwidth(FilterBandwidth::BAND_44_HZ)
    , _sampleRate(100)
    , _gyroOffset{0.0f, 0.0f, 0.0f}
    , _gyroScale(GYRO_SCALE_FACTOR[0])
    , _accelScale(ACCEL_SCALE_FACTOR[0])
    , _isSampling(false)
    , _sampleCount(0)
    , _effectiveSampleRate(0.0f) {
}

MPU6050::~MPU6050() {
    // Stop sampling thread if active
    stopSampling();
    
    // Close I2C device if open
    if (_i2cFd >= 0) {
        close(_i2cFd);
    }
}

bool MPU6050::initialize(GyroRange gyroRange, AccelRange accelRange,
                         FilterBandwidth filterBandwidth, uint16_t sampleRate) {
    // Open I2C device
    _i2cFd = open(_devicePath.c_str(), O_RDWR);
    if (_i2cFd < 0) {
        std::cerr << "Failed to open I2C device: " << _devicePath << std::endl;
        return false;
    }
    
    // Set I2C slave address
    if (ioctl(_i2cFd, I2C_SLAVE, _address) < 0) {
        std::cerr << "Failed to set I2C slave address" << std::endl;
        close(_i2cFd);
        _i2cFd = -1;
        return false;
    }
    
    // Check device ID
    uint8_t whoAmI;
    if (!readRegister(MPU6050_REG_WHO_AM_I, whoAmI) || whoAmI != 0x68) {
        std::cerr << "Failed to identify MPU6050 (expected 0x68, got 0x"
                  << std::hex << static_cast<int>(whoAmI) << ")" << std::endl;
        close(_i2cFd);
        _i2cFd = -1;
        return false;
    }
    
    // Wake up the sensor (clear sleep bit)
    if (!writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00)) {
        std::cerr << "Failed to wake up sensor" << std::endl;
        return false;
    }
    
    // Store the sample rate
    _sampleRate = std::min(std::max(sampleRate, static_cast<uint16_t>(10)),
                          static_cast<uint16_t>(1000));
    
    // Set sample rate divider
    // The sensor sample rate is 1kHz divided by this value
    uint8_t divider = std::max(std::min(static_cast<int>(1000 / _sampleRate - 1), 255), 0);
    if (!writeRegister(MPU6050_REG_SMPLRT_DIV, divider)) {
        std::cerr << "Failed to set sample rate divider" << std::endl;
        return false;
    }
    
    // Set digital low-pass filter
    _filterBandwidth = filterBandwidth;
    if (!writeRegister(MPU6050_REG_CONFIG, static_cast<uint8_t>(_filterBandwidth))) {
        std::cerr << "Failed to set filter bandwidth" << std::endl;
        return false;
    }
    
    // Set gyroscope range
    _gyroRange = gyroRange;
    if (!writeRegister(MPU6050_REG_GYRO_CONFIG,
                      static_cast<uint8_t>(_gyroRange) << 3)) {
        std::cerr << "Failed to set gyroscope range" << std::endl;
        return false;
    }
    
    // Set accelerometer range
    _accelRange = accelRange;
    if (!writeRegister(MPU6050_REG_ACCEL_CONFIG,
                      static_cast<uint8_t>(_accelRange) << 3)) {
        std::cerr << "Failed to set accelerometer range" << std::endl;
        return false;
    }
    
    // Update scale factors
    updateScaleFactors();
    
    // Small delay for the sensor to stabilize
    usleep(100000);  // 100ms
    
    std::cout << "MPU6050 initialized successfully" << std::endl;
    return true;
}

bool MPU6050::calibrateGyro(uint16_t samples, float maxDuration) {
    if (_i2cFd < 0) {
        std::cerr << "Device not initialized" << std::endl;
        return false;
    }
    
    std::cout << "Calibrating gyroscope. Keep the sensor still..." << std::endl;
    
    // Reset offsets
    _gyroOffset = {0.0f, 0.0f, 0.0f};
    
    // Collect samples
    std::vector<std::array<float, 3>> gyroReadings;
    gyroReadings.reserve(samples);
    
    auto startTime = std::chrono::high_resolution_clock::now();
    auto endTime = startTime + std::chrono::duration<float>(maxDuration);
    
    for (uint16_t i = 0; i < samples; ++i) {
        // Check for timeout
        auto currentTime = std::chrono::high_resolution_clock::now();
        if (currentTime > endTime) {
            std::cerr << "Calibration timeout after "
                      << gyroReadings.size() << " samples" << std::endl;
            break;
        }
        
        // Read raw data
        RawData raw = readRawData();
        
        // Store gyro readings
        gyroReadings.push_back({
            static_cast<float>(raw.gx),
            static_cast<float>(raw.gy),
            static_cast<float>(raw.gz)
        });
        
        // Small delay between readings
        usleep(1000);  // 1ms
    }
    
    // Calculate mean offsets
    if (gyroReadings.empty()) {
        std::cerr << "No samples collected for calibration" << std::endl;
        return false;
    }
    
    std::array<float, 3> sum = {0.0f, 0.0f, 0.0f};
    for (const auto& reading : gyroReadings) {
        sum[0] += reading[0];
        sum[1] += reading[1];
        sum[2] += reading[2];
    }
    
    _gyroOffset[0] = sum[0] / gyroReadings.size();
    _gyroOffset[1] = sum[1] / gyroReadings.size();
    _gyroOffset[2] = sum[2] / gyroReadings.size();
    
    std::cout << "Gyroscope calibration complete. Offsets: ["
              << _gyroOffset[0] << ", "
              << _gyroOffset[1] << ", "
              << _gyroOffset[2] << "]" << std::endl;
    
    return true;
}

MPU6050::RawData MPU6050::readRawData() {
    RawData data = {0};
    
    if (_i2cFd < 0) {
        std::cerr << "Device not initialized" << std::endl;
        return data;
    }
    
    // Read 14 bytes starting from ACCEL_XOUT_H register
    uint8_t buffer[14];
    if (!readRegisters(MPU6050_REG_ACCEL_XOUT_H, buffer, 14)) {
        std::cerr << "Failed to read sensor data" << std::endl;
        return data;
    }
    
    // Combine high and low bytes to form 16-bit values
    data.ax = (buffer[0] << 8) | buffer[1];
    data.ay = (buffer[2] << 8) | buffer[3];
    data.az = (buffer[4] << 8) | buffer[5];
    data.temp = (buffer[6] << 8) | buffer[7];
    data.gx = (buffer[8] << 8) | buffer[9];
    data.gy = (buffer[10] << 8) | buffer[11];
    data.gz = (buffer[12] << 8) | buffer[13];
    
    return data;
}

MPU6050::SensorData MPU6050::readSensorData() {
    SensorData data;
    data.timestamp = std::chrono::high_resolution_clock::now();
    
    RawData raw = readRawData();
    
    // Convert accelerometer values to g
    data.ax = static_cast<float>(raw.ax) / _accelScale;
    data.ay = static_cast<float>(raw.ay) / _accelScale;
    data.az = static_cast<float>(raw.az) / _accelScale;
    
    // Convert gyroscope values to degrees/sec and apply calibration offset
    data.gx = (static_cast<float>(raw.gx) - _gyroOffset[0]) / _gyroScale;
    data.gy = (static_cast<float>(raw.gy) - _gyroOffset[1]) / _gyroScale;
    data.gz = (static_cast<float>(raw.gz) - _gyroOffset[2]) / _gyroScale;
    
    // Convert temperature to Celsius
    // Formula: Temp_degC = (TEMP_OUT Register Value / 340) + 36.53
    data.temp = (static_cast<float>(raw.temp) / 340.0f) + 36.53f;
    
    return data;
}

void MPU6050::setGyroRange(GyroRange range) {
    if (_i2cFd < 0) {
        std::cerr << "Device not initialized" << std::endl;
        return;
    }
    
    _gyroRange = range;
    
    // Update the register with the new range (bits 3 and 4)
    uint8_t regValue;
    if (readRegister(MPU6050_REG_GYRO_CONFIG, regValue)) {
        regValue = (regValue & 0xE7) | (static_cast<uint8_t>(range) << 3);
        writeRegister(MPU6050_REG_GYRO_CONFIG, regValue);
        
        // Update the scale factor
        updateScaleFactors();
    }
}

void MPU6050::setAccelRange(AccelRange range) {
    if (_i2cFd < 0) {
        std::cerr << "Device not initialized" << std::endl;
        return;
    }
    
    _accelRange = range;
    
    // Update the register with the new range (bits 3 and 4)
    uint8_t regValue;
    if (readRegister(MPU6050_REG_ACCEL_CONFIG, regValue)) {
        regValue = (regValue & 0xE7) | (static_cast<uint8_t>(range) << 3);
        writeRegister(MPU6050_REG_ACCEL_CONFIG, regValue);
        
        // Update the scale factor
        updateScaleFactors();
    }
}

void MPU6050::setFilterBandwidth(FilterBandwidth bandwidth) {
    if (_i2cFd < 0) {
        std::cerr << "Device not initialized" << std::endl;
        return;
    }
    
    _filterBandwidth = bandwidth;
    writeRegister(MPU6050_REG_CONFIG, static_cast<uint8_t>(bandwidth));
}

MPU6050::GyroRange MPU6050::getGyroRange() const {
    return _gyroRange;
}

MPU6050::AccelRange MPU6050::getAccelRange() const {
    return _accelRange;
}

MPU6050::FilterBandwidth MPU6050::getFilterBandwidth() const {
    return _filterBandwidth;
}

bool MPU6050::startSampling() {
    if (_i2cFd < 0) {
        std::cerr << "Device not initialized" << std::endl;
        return false;
    }
    
    if (_isSampling.load()) {
        std::cerr << "Sampling already active" << std::endl;
        return false;
    }
    
    // Reset buffer and metrics
    {
        std::lock_guard<std::mutex> lock(_bufferMutex);
        _dataBuffer.clear();
    }
    
    _sampleCount = 0;
    _effectiveSampleRate = 0.0f;
    _samplingStartTime = std::chrono::high_resolution_clock::now();
    
    // Start sampling thread
    _isSampling.store(true);
    _samplingThread = std::make_unique<std::thread>(&MPU6050::samplingLoop, this);
    
    std::cout << "Started continuous sampling at " << _sampleRate << " Hz" << std::endl;
    return true;
}

void MPU6050::stopSampling() {
    if (!_isSampling.load()) {
        return;
    }
    
    // Stop sampling thread
    _isSampling.store(false);
    
    if (_samplingThread && _samplingThread->joinable()) {
        _samplingThread->join();
    }
    
    _samplingThread.reset();
    std::cout << "Stopped continuous sampling" << std::endl;
}

bool MPU6050::isSampling() const {
    return _isSampling.load();
}

std::vector<MPU6050::SensorData> MPU6050::getLatestData(size_t count) {
    std::vector<SensorData> result;
    
    if (count == 0) {
        return result;
    }
    
    // Lock the buffer and copy the requested number of samples
    std::lock_guard<std::mutex> lock(_bufferMutex);
    
    if (_dataBuffer.empty()) {
        return result;
    }
    
    size_t actualCount = std::min(count, _dataBuffer.size());
    result.reserve(actualCount);
    
    auto it = _dataBuffer.end() - actualCount;
    for (; it != _dataBuffer.end(); ++it) {
        result.push_back(*it);
    }
    
    return result;
}

float MPU6050::getEffectiveSampleRate() const {
    return _effectiveSampleRate.load();
}

std::string MPU6050::getSamplingStats() const {
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration<float>(currentTime - _samplingStartTime).count();
    
    std::ostringstream oss;
    oss << "{";
    oss << "\"sampleCount\": " << _sampleCount.load() << ", ";
    oss << "\"elapsedTime\": " << std::fixed << std::setprecision(3) << elapsedTime << ", ";
    oss << "\"effectiveSampleRate\": " << std::fixed << std::setprecision(1) << _effectiveSampleRate.load() << ", ";
    oss << "\"requestedSampleRate\": " << _sampleRate << ", ";
    oss << "\"bufferSize\": " << _dataBuffer.size() << ", ";
    oss << "\"active\": " << (_isSampling.load() ? "true" : "false");
    oss << "}";
    
    return oss.str();
}

bool MPU6050::writeRegister(uint8_t reg, uint8_t data) {
    if (_i2cFd < 0) {
        return false;
    }
    
    uint8_t buffer[2] = {reg, data};
    return write(_i2cFd, buffer, 2) == 2;
}

bool MPU6050::readRegister(uint8_t reg, uint8_t& data) {
    if (_i2cFd < 0) {
        return false;
    }
    
    // Set register address
    if (write(_i2cFd, &reg, 1) != 1) {
        return false;
    }
    
    // Read register value
    return read(_i2cFd, &data, 1) == 1;
}

bool MPU6050::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    if (_i2cFd < 0 || buffer == nullptr) {
        return false;
    }
    
    // Set register address
    if (write(_i2cFd, &reg, 1) != 1) {
        return false;
    }
    
    // Read register values
    return read(_i2cFd, buffer, length) == length;
}

void MPU6050::updateScaleFactors() {
    // Update the scale factors based on current range settings
    _gyroScale = GYRO_SCALE_FACTOR[static_cast<int>(_gyroRange)];
    _accelScale = ACCEL_SCALE_FACTOR[static_cast<int>(_accelRange)];
}

void MPU6050::samplingLoop() {
    // Calculate the target interval in microseconds
    const uint64_t targetIntervalUs = 1000000 / _sampleRate;
    uint64_t totalSamples = 0;
    auto startTime = std::chrono::high_resolution_clock::now();
    auto nextSampleTime = startTime;
    
    while (_isSampling.load()) {
        // Calculate sleep time until next sample
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto sleepDuration = nextSampleTime - currentTime;
        
        // Sleep until next sample time if we're ahead of schedule
        if (sleepDuration.count() > 0) {
            std::this_thread::sleep_for(sleepDuration);
        }
        
        // Read sensor data
        SensorData data = readSensorData();
        
        // Store data in buffer
        {
            std::lock_guard<std::mutex> lock(_bufferMutex);
            _dataBuffer.push_back(data);
            
            // Limit buffer size
            while (_dataBuffer.size() > MAX_BUFFER_SIZE) {
                _dataBuffer.pop_front();
            }
        }
        
        // Update sample count
        totalSamples++;
        _sampleCount.store(totalSamples);
        
        // Update effective sample rate
        auto elapsed = std::chrono::duration<float>(
            std::chrono::high_resolution_clock::now() - startTime).count();
        if (elapsed > 0.0f) {
            _effectiveSampleRate.store(totalSamples / elapsed);
        }
        
        // Calculate next sample time
        nextSampleTime += std::chrono::microseconds(targetIntervalUs);
    }
}