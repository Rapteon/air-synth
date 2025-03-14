/**
 * @file MPU6050.hpp
 * @brief Interface for the MPU6050 accelerometer and gyroscope sensor
 * @author Mohammed
 * @date Feb 25, 2025
 */


#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <array>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include <cstdint>
#include <chrono>
#include <memory>

/**
 * @class MPU6050
 * @brief Class for interfacing with the MPU6050 sensor using I2C on Raspberry Pi
 */
class MPU6050 {
public:
    /**
     * @brief Structure to hold raw sensor readings
     */
    struct RawData {
        int16_t ax, ay, az;    // Raw accelerometer values
        int16_t gx, gy, gz;    // Raw gyroscope values
        int16_t temp;          // Raw temperature value
    };
    
    /**
     * @brief Structure to hold calibrated sensor readings in standard units
     */
    struct SensorData {
        float ax, ay, az;      // Acceleration in g
        float gx, gy, gz;      // Angular velocity in degrees/sec
        float temp;            // Temperature in °C
        std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
    };
    
    /**
     * @brief Gyroscope range settings
     */
    enum class GyroRange : uint8_t {
        RANGE_250_DEG  = 0x00,  // ±250 °/s
        RANGE_500_DEG  = 0x01,  // ±500 °/s
        RANGE_1000_DEG = 0x02,  // ±1000 °/s
        RANGE_2000_DEG = 0x03   // ±2000 °/s
    };
    
    /**
     * @brief Accelerometer range settings
     */
    enum class AccelRange : uint8_t {
        RANGE_2G  = 0x00,  // ±2g
        RANGE_4G  = 0x01,  // ±4g
        RANGE_8G  = 0x02,  // ±8g
        RANGE_16G = 0x03   // ±16g
    };
    
    /**
     * @brief Digital low-pass filter settings
     */
    enum class FilterBandwidth : uint8_t {
        BAND_260_HZ = 0x00,  // 260 Hz
        BAND_184_HZ = 0x01,  // 184 Hz
        BAND_94_HZ  = 0x02,  // 94 Hz
        BAND_44_HZ  = 0x03,  // 44 Hz
        BAND_21_HZ  = 0x04,  // 21 Hz
        BAND_10_HZ  = 0x05,  // 10 Hz
        BAND_5_HZ   = 0x06   // 5 Hz
    };
    
    /**
     * @brief Constructor
     * @param devicePath I2C device path (default: "/dev/i2c-1")
     * @param address I2C address of the MPU6050 (default: 0x68)
     */
    MPU6050(const std::string& devicePath = "/dev/i2c-1", uint8_t address = 0x68);
    
    /**
     * @brief Destructor
     */
    ~MPU6050();
    
    /**
     * @brief Initialize the MPU6050 sensor
     * @param gyroRange Gyroscope range setting
     * @param accelRange Accelerometer range setting
     * @param filterBandwidth Digital low-pass filter setting
     * @param sampleRate Sensor sample rate in Hz (10-1000 Hz)
     * @return True if initialization successful, false otherwise
     */
    bool initialize(GyroRange gyroRange = GyroRange::RANGE_250_DEG,
                   AccelRange accelRange = AccelRange::RANGE_2G,
                   FilterBandwidth filterBandwidth = FilterBandwidth::BAND_44_HZ,
                   uint16_t sampleRate = 100);
    
    /**
     * @brief Calibrate the gyroscope by finding the zero offset
     * @param samples Number of samples to take for calibration
     * @param maxDuration Maximum duration for calibration in seconds
     * @return True if calibration successful, false otherwise
     */
    bool calibrateGyro(uint16_t samples = 1000, float maxDuration = 5.0);
    
    /**
     * @brief Read raw sensor values
     * @return RawData structure containing raw sensor readings
     */
    RawData readRawData();
    
    /**
     * @brief Read calibrated sensor values in standard units
     * @return SensorData structure containing calibrated sensor readings
     */
    SensorData readSensorData();
    
    /**
     * @brief Set the gyroscope range
     * @param range The desired range
     */
    void setGyroRange(GyroRange range);
    
    /**
     * @brief Set the accelerometer range
     * @param range The desired range
     */
    void setAccelRange(AccelRange range);
    
    /**
     * @brief Set the digital low-pass filter bandwidth
     * @param bandwidth The desired bandwidth
     */
    void setFilterBandwidth(FilterBandwidth bandwidth);
    
    /**
     * @brief Get the current gyroscope range
     * @return The current gyroscope range
     */
    GyroRange getGyroRange() const;
    
    /**
     * @brief Get the current accelerometer range
     * @return The current accelerometer range
     */
    AccelRange getAccelRange() const;
    
    /**
     * @brief Get the current filter bandwidth
     * @return The current filter bandwidth
     */
    FilterBandwidth getFilterBandwidth() const;
    
    /**
     * @brief Start continuous sampling in a background thread
     * @return True if sampling started successfully
     */
    bool startSampling();
    
    /**
     * @brief Stop the continuous sampling thread
     */
    void stopSampling();
    
    /**
     * @brief Check if continuous sampling is running
     * @return True if sampling is active
     */
    bool isSampling() const;
    
    /**
     * @brief Get the latest sensor readings from the buffer
     * @param count Number of readings to retrieve
     * @return Vector of the latest sensor readings
     */
    std::vector<SensorData> getLatestData(size_t count = 1);
    
    /**
     * @brief Get the current sampling rate in Hz
     * @return The effective sampling rate
     */
    float getEffectiveSampleRate() const;
    
    /**
     * @brief Get statistics about the sampling performance
     * @return JSON-formatted string with sampling statistics
     */
    std::string getSamplingStats() const;

private:
    // I2C communication
    int _i2cFd;                      // I2C file descriptor
    std::string _devicePath;         // I2C device path
    uint8_t _address;                // I2C address of the MPU6050
    
    // Configuration
    GyroRange _gyroRange;            // Current gyroscope range
    AccelRange _accelRange;          // Current accelerometer range
    FilterBandwidth _filterBandwidth; // Current filter bandwidth
    uint16_t _sampleRate;            // Desired sample rate in Hz
    
    // Calibration
    std::array<float, 3> _gyroOffset; // Gyroscope calibration offsets
    
    // Scale factors
    float _gyroScale;                // Current gyroscope scale factor
    float _accelScale;               // Current accelerometer scale factor
    
    // Sampling thread
    std::unique_ptr<std::thread> _samplingThread;
    std::atomic<bool> _isSampling;
    std::deque<SensorData> _dataBuffer;
    std::mutex _bufferMutex;
    
    // Performance metrics
    std::atomic<uint64_t> _sampleCount;
    std::atomic<float> _effectiveSampleRate;
    std::chrono::time_point<std::chrono::high_resolution_clock> _samplingStartTime;
    
    /**
     * @brief Write a byte to the MPU6050 register
     * @param reg Register address
     * @param data Data byte to write
     * @return True if write successful
     */
    bool writeRegister(uint8_t reg, uint8_t data);
    
    /**
     * @brief Read a byte from the MPU6050 register
     * @param reg Register address
     * @param data Reference to store the read data
     * @return True if read successful
     */
    bool readRegister(uint8_t reg, uint8_t& data);
    
    /**
     * @brief Read multiple bytes from the MPU6050 registers
     * @param reg Starting register address
     * @param buffer Buffer to store the read data
     * @param length Number of bytes to read
     * @return True if read successful
     */
    bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
    
    /**
     * @brief Update the scale factors based on range settings
     */
    void updateScaleFactors();
    
    /**
     * @brief Sampling thread function
     */
    void samplingLoop();
};

#endif // MPU6050_HPP