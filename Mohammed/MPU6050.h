/**
 * @file MPU6050.h
 * @brief Interface for the MPU6050 accelerometer and gyroscope sensor
 * @author Mohammed
 * @date Feb 25, 2025
 * 
 * This file provides a clean, well-documented interface for interacting with
 * the MPU6050 6-axis accelerometer and gyroscope sensor using I2C.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

// MPU6050 I2C address (AD0 pin low)
#define MPU6050_ADDR 0x68

// MPU6050 register addresses
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43

// Gyroscope sensitivity scale factors
#define GYRO_SCALE_250DPS   131.0f
#define GYRO_SCALE_500DPS   65.5f
#define GYRO_SCALE_1000DPS  32.8f
#define GYRO_SCALE_2000DPS  16.4f

// Accelerometer sensitivity scale factors
#define ACCEL_SCALE_2G      16384.0f
#define ACCEL_SCALE_4G      8192.0f
#define ACCEL_SCALE_8G      4096.0f
#define ACCEL_SCALE_16G     2048.0f

// Configuration options
enum GyroRange {
    GYRO_RANGE_250DPS = 0,
    GYRO_RANGE_500DPS = 1,
    GYRO_RANGE_1000DPS = 2,
    GYRO_RANGE_2000DPS = 3
};

enum AccelRange {
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G = 1,
    ACCEL_RANGE_8G = 2,
    ACCEL_RANGE_16G = 3
};

/**
 * @class MPU6050
 * @brief Class for interfacing with the MPU6050 sensor
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
    };

    /**
     * @brief Constructor
     * @param wire Reference to the TwoWire instance (default: Wire)
     */
    MPU6050(TwoWire &wire = Wire);
    
    /**
     * @brief Initialize the MPU6050 sensor
     * @param gyroRange Gyroscope sensitivity range
     * @param accelRange Accelerometer sensitivity range
     * @return True if initialization successful, false otherwise
     */
    bool begin(GyroRange gyroRange = GYRO_RANGE_250DPS, 
               AccelRange accelRange = ACCEL_RANGE_2G);
               
    /**
     * @brief Calibrate the gyroscope by finding the zero offset
     * @param samples Number of samples to take for calibration
     */
    void calibrateGyro(uint16_t samples = 1000);
    
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
     * @brief Get the current gyroscope range
     * @return The current gyroscope range
     */
    GyroRange getGyroRange() const;
    
    /**
     * @brief Get the current accelerometer range
     * @return The current accelerometer range
     */
    AccelRange getAccelRange() const;

private:
    TwoWire &_wire;                    // I2C interface
    GyroRange _gyroRange;              // Current gyroscope range
    AccelRange _accelRange;            // Current accelerometer range
    float _gyroScale;                  // Current gyroscope scale factor
    float _accelScale;                 // Current accelerometer scale factor
    float _gyroOffset[3] = {0};        // Gyroscope calibration offsets
    
    /**
     * @brief Write a byte to the MPU6050 register
     * @param reg Register address
     * @param data Data byte to write
     */
    void writeRegister(uint8_t reg, uint8_t data);
    
    /**
     * @brief Read a byte from the MPU6050 register
     * @param reg Register address
     * @return Value read from the register
     */
    uint8_t readRegister(uint8_t reg);
    
    /**
     * @brief Read multiple bytes from the MPU6050 registers
     * @param reg Starting register address
     * @param buffer Buffer to store the read data
     * @param length Number of bytes to read
     */
    void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
    
    /**
     * @brief Update the scale factors based on range settings
     */
    void updateScaleFactors();
};

#endif // MPU6050_H