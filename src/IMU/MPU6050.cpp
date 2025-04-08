#include "IMU/MPU6050.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <csignal>
#include <gpiod.h>


MPU6050::~MPU6050()
{
    close(file);
    std::cerr << "I2C Closed for address " << address << "\n";
}
// TODO: Pass I2CBus as a reference to the constructor. The Bus should be a member
// within this class which must be used to configure this class with the MPUConfigReader's
// configure method. So the actual calls to the Bus's methods should be done
// within the configure method of the MPUConfigReader class.
MPU6050::MPU6050(int addr) : address(addr), running(true), active(false), counter_x(0), counter_y(0), counter_z(0),
                             threshold_crossed_x(false), threshold_crossed_y(false), threshold_crossed_z(false), bus(nullptr)
{
    // This 'if' can be removed, because this is done in the I2CBus constructor.
    if ((file = open(I2C_BUS, O_RDWR)) < 0)
    {
        std::cerr << "Failed to open I2C bus" << std::endl;
    }
    
    // This is in a method which can be called by MPUConfigReader's configure method.
    if (ioctl(file, I2C_SLAVE, address) < 0)
    {
        std::cerr << "Failed to initialize MPU6050 at address " << address << std::endl;
    }
    // Same as above if for this one.
    uint8_t config[2] = {PWR_MGMT_1, 0x00};
    if (write(file, config, 2) != 2)
    {
        std::cerr << "Failed to initialize MPU6050 at address " << address << std::endl;
    }
    bus = nullptr;
    calibrate_z_axis();
}

MPU6050::MPU6050(I2CBus &bus): bus(bus) {
    calibrate_z_axis();
}

// Should be a private method, and public facing methods should internally
// decide what register to write. The public facing API should not expose
// registers.
int16_t MPU6050::read_word(int8_t reg)
{
    uint8_t buffer[2];
    write(file, &reg, 1);
    read(file, buffer, 2);
    return (buffer[0] << 8) | buffer[1];
}

void MPU6050::calibrate_z_axis()
{
    constexpr int CALIBRATION_SAMPLES = 100;
    double sum_az = 0.0;

    std::cerr << "Calibrating Z-axis... Please keep the sensor stationary. Device Address: " << address << std::endl;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        sum_az += (read_word(ACCEL_ZOUT_H) / ACCEL_SCALE) * G;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    z_offset = sum_az / CALIBRATION_SAMPLES;
    std::cerr << "Z-axis calibration completed. Offset: " << z_offset << " m/s²" << std::endl;
}

void MPU6050::worker()
{
    while (running)
    {
        if (!active)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        // Convert these ACCEL_XOUT_H, ACCEL_YOUT_H, and ACCEL_ZOUT_H to
        // variables within this class, and set them in the configure method
        // of the MPUConfigReader class.
        int16_t raw_x = read_word(ACCEL_XOUT_H);
        int16_t raw_y = read_word(ACCEL_YOUT_H);
        int16_t raw_z = read_word(ACCEL_ZOUT_H);
        double ax = (raw_x / ACCEL_SCALE) * G;
        double ay = (raw_y / ACCEL_SCALE) * G;
        double az = ((raw_z / ACCEL_SCALE) * G) - z_offset;
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (ax > ACCEL_THRESHOLD_POS && !threshold_crossed_x)
                counter_x += INCREMENT_VALUE, threshold_crossed_x = true;
            else if (ax < ACCEL_THRESHOLD_NEG && !threshold_crossed_x)
                counter_x -= INCREMENT_VALUE, threshold_crossed_x = true;
            else if (ax > ACCEL_THRESHOLD_NEG && ax < ACCEL_THRESHOLD_POS)
                threshold_crossed_x = false;

            if (ay > ACCEL_THRESHOLD_POS && !threshold_crossed_y)
                counter_y += INCREMENT_VALUE, threshold_crossed_y = true;
            else if (ay < ACCEL_THRESHOLD_NEG && !threshold_crossed_y)
                counter_y -= INCREMENT_VALUE, threshold_crossed_y = true;
            else if (ay > ACCEL_THRESHOLD_NEG && ay < ACCEL_THRESHOLD_POS)
                threshold_crossed_y = false;

            if (az > ACCEL_THRESHOLD_POS_Z && !threshold_crossed_z)
                counter_z += INCREMENT_VALUE, threshold_crossed_z = true;
            else if (az < ACCEL_THRESHOLD_NEG_Z && !threshold_crossed_z)
                counter_z -= INCREMENT_VALUE, threshold_crossed_z = true;
            else if (az > ACCEL_THRESHOLD_NEG_Z && az < ACCEL_THRESHOLD_POS_Z)
                threshold_crossed_z = false;

            std::cerr << "[MPU " << address << "] Counter_X = " << counter_x << " || Counter_Y = " << counter_y << " || Counter_Z = " << counter_z << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MPU6050::start()
{
    std::thread(&MPU6050::worker, this).detach();
}


I2CBus MPU6050::getBus() {
    return bus;
}

void MPU6050::setAddress(int address) {
    this->address = address;
}

int MPU6050::getAddress() {
    return address;
}