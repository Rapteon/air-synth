#ifndef CONTROLLEREVENT_H
#define CONTROLLEREVENT_H

/**
 * @class ControllerEvent
 * @brief Represents a motion event from the MPU6050 sensor with acceleration values.
 * 
 * This class encapsulates the three-dimensional acceleration data from the MPU6050 sensor.
 * It provides immutable access to the acceleration values through const member functions.
 */
class ControllerEvent {
public:
    /**
     * @brief Constructs a new ControllerEvent with the given acceleration values.
     * @param x X-axis acceleration in m/s²
     * @param y Y-axis acceleration in m/s²
     * @param z Z-axis acceleration in m/s²
     */
    ControllerEvent(double x, double y, double z);
    
    /**
     * @brief Gets the X-axis acceleration value.
     * @return X-axis acceleration in m/s²
     */
    double getX() const noexcept;
    
    /**
     * @brief Gets the Y-axis acceleration value.
     * @return Y-axis acceleration in m/s²
     */
    double getY() const noexcept;
    
    /**
     * @brief Gets the Z-axis acceleration value.
     * @return Z-axis acceleration in m/s²
     */
    double getZ() const noexcept;

private:
    const double x;  // X-axis acceleration
    const double y;  // Y-axis acceleration
    const double z;  // Z-axis acceleration
};

#endif