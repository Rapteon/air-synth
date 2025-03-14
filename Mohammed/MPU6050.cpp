/**
 * @file MPU6050.cpp
 * @brief Implementation of the MPU6050 sensor interface
 * @author Mohammed
 * @date Feb 25, 2025
 */

#include "MPU6050.h"

MPU6050::MPU6050(TwoWire &wire) : 
    _wire(wire),
    _gyroRange(GYRO_RANGE_250DPS),
    _accelRange(ACCEL_RANGE_2G),
    _gyroScale(GYRO_SCALE_250DPS),
    _accelScale(ACCEL_SCALE_2G) {
}

bool MPU6050::begin(GyroRange gyroRange, AccelRange accelRange) {
    // Initialize I2C communication
    _wire.begin();
    
    // Check if the MPU6050 is responding
    _wire.beginTransmission(MPU6050_ADDR);
    if (_wire.endTransmission() != 0) {
        return false;  // Device not found
    }
    
    // Wake up the MPU6050 (clear sleep bit)
    writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00);
    
    // Set gyroscope and accelerometer ranges
    setGyroRange(gyroRange);
    setAccelRange(accelRange);
    
    // Small delay for the sensor to stabilize
    delay(100);
    
    return true;
}

void MPU6050::calibrateGyro(uint16_t samples) {
    float sumX = 0, sumY = 0, sumZ = 0;
    
    // Read multiple samples for averaging
    for (uint16_t i = 0; i < samples; i++) {
        RawData raw = readRawData();
        sumX += raw.gx;
        sumY += raw.gy;
        sumZ += raw.gz;
        delay(1);
    }
    
    // Calculate average offsets
    _gyroOffset[0] = sumX / samples;
    _gyroOffset[1] = sumY / samples;
    _gyroOffset[2] = sumZ / samples;
}

MPU6050::RawData MPU6050::readRawData() {
    RawData data;
    uint8_t buffer[14];
    
    // Read 14 bytes starting from ACCEL_XOUT_H register
    readRegisters(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    
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
    RawData raw = readRawData();
    SensorData data;
    
    // Convert accelerometer values to g
    data.ax = raw.ax / _accelScale;
    data.ay = raw.ay / _accelScale;
    data.az = raw.az / _accelScale;
    
    // Convert gyroscope values to degrees/sec and apply calibration offset
    data.gx = (raw.gx - _gyroOffset[0]) / _gyroScale;
    data.gy = (raw.gy - _gyroOffset[1]) / _gyroScale;
    data.gz = (raw.gz - _gyroOffset[2]) / _gyroScale;
    
    // Convert temperature to Celsius
    // Formula: Temp_degC = (TEMP_OUT Register Value / 340) + 36.53
    data.temp = (raw.temp / 340.0) + 36.53;
    
    return data;
}

void MPU6050::setGyroRange(GyroRange range) {
    _gyroRange = range;
    
    // Update the register with the new range (bits 3 and 4)
    uint8_t regValue = readRegister(MPU6050_REG_GYRO_CONFIG);
    regValue = (regValue & 0xE7) | (range << 3);
    writeRegister(MPU6050_REG_GYRO_CONFIG, regValue);
    
    // Update the scale factor
    updateScaleFactors();
}

void MPU6050::setAccelRange(AccelRange range) {
    _accelRange = range;
    
    // Update the register with the new range (bits 3 and 4)
    uint8_t regValue = readRegister(MPU6050_REG_ACCEL_CONFIG);
    regValue = (regValue & 0xE7) | (range << 3);
    writeRegister(MPU6050_REG_ACCEL_CONFIG, regValue);
    
    // Update the scale factor
    updateScaleFactors();
}

GyroRange MPU6050::getGyroRange() const {
    return _gyroRange;
}

AccelRange MPU6050::getAccelRange() const {
    return _accelRange;
}

void MPU6050::writeRegister(uint8_t reg, uint8_t data) {
    _wire.beginTransmission(MPU6050_ADDR);
    _wire.write(reg);
    _wire.write(data);
    _wire.endTransmission();
}

uint8_t MPU6050::readRegister(uint8_t reg) {
    uint8_t data = 0;
    
    _wire.beginTransmission(MPU6050_ADDR);
    _wire.write(reg);
    _wire.endTransmission(false);  // Don't release the bus
    
    _wire.requestFrom(MPU6050_ADDR, 1);
    if (_wire.available()) {
        data = _wire.read();
    }
    
    return data;
}

void MPU6050::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    _wire.beginTransmission(MPU6050_ADDR);
    _wire.write(reg);
    _wire.endTransmission(false);  // Don't release the bus
    
    _wire.requestFrom(MPU6050_ADDR, length);
    for (uint8_t i = 0; i < length && _wire.available(); i++) {
        buffer[i] = _wire.read();
    }
}

void MPU6050::updateScaleFactors() {
    // Update the scale factors based on current range settings
    switch (_gyroRange) {
        case GYRO_RANGE_250DPS:
            _gyroScale = GYRO_SCALE_250DPS;
            break;
        case GYRO_RANGE_500DPS:
            _gyroScale = GYRO_SCALE_500DPS;
            break;
        case GYRO_RANGE_1000DPS:
            _gyroScale = GYRO_SCALE_1000DPS;
            break;
        case GYRO_RANGE_2000DPS:
            _gyroScale = GYRO_SCALE_2000DPS;
            break;
    }
    
    switch (_accelRange) {
        case ACCEL_RANGE_2G:
            _accelScale = ACCEL_SCALE_2G;
            break;
        case ACCEL_RANGE_4G:
            _accelScale = ACCEL_SCALE_4G;
            break;
        case ACCEL_RANGE_8G:
            _accelScale = ACCEL_SCALE_8G;
            break;
        case ACCEL_RANGE_16G:
            _accelScale = ACCEL_SCALE_16G;
            break;
    }
}