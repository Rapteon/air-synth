#include "IMU/MPU6050.h"
#include "Synth/Synth.h"
#include "ControllerEvent/ControllerEvent.h"
#include <stdexcept>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <csignal>
#include <gpiod.h>

MPU6050::~MPU6050()
{
    close(file);
    std::cerr << "I2C Closed for address " << address << "\n";
    
    // Clean up synth if attached
    if (synth) {
        synth->stop();
    }
}

MPU6050::MPU6050(int addr) : address(addr), running(true), active(false), counter_x(0), counter_y(0), counter_z(0),
                             threshold_crossed_x(false), threshold_crossed_y(false), threshold_crossed_z(false)
{
    if ((file = open(I2C_BUS, O_RDWR)) < 0)
    {
        std::cerr << "Failed to open I2C bus" << std::endl;
    }
    if (ioctl(file, I2C_SLAVE, address) < 0)
    {
        std::cerr << "Failed to initialize MPU6050 at address " << address << std::endl;
    }
    uint8_t config[2] = {PWR_MGMT_1, 0x00};
    if (write(file, config, 2) != 2)
    {
        std::cerr << "Failed to initialize MPU6050 at address " << address << std::endl;
    }

    calibrate_z_axis();
}

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

    std::cout << "Calibrating Z-axis... Please keep the sensor stationary. Device Address: " << address << std::endl;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        try {
            sum_az += (read_word(ACCEL_ZOUT_H) / ACCEL_SCALE) * G;
        } catch (const std::runtime_error& e) {
            std::cerr << "Calibration error: " << e.what() << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    z_offset = sum_az / CALIBRATION_SAMPLES;
    std::cout << "Z-axis calibration completed. Offset: " << z_offset << " m/s²" << std::endl;
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

            ControllerEvent ce {ax, ay, az};
            onEvent(ce);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MPU6050::start()
{
    // Create a new thread for the worker function
    std::thread(&MPU6050::worker, this).detach();
    std::cout << "MPU6050 started for address " << address << std::endl;
}

void MPU6050::set_active(bool state)
{
    active = state;
    
    if (state && synth) {
        synth->start();
        std::cout << "Synth activated for MPU6050 at address " << address << std::endl;
    } else if (!state && synth) {
        synth->stop();
        std::cout << "Synth deactivated for MPU6050 at address " << address << std::endl;
    }
}

void MPU6050::hasEvent(ControllerEvent& event)
{
    // This method is called when this MPU6050 acts as a receiver of events
    // Just forward to Synth if it's attached
    if (synth && active) {
        synth->addEvent(event);
    }
}

void MPU6050::onEvent(ControllerEvent& event)
{
    // Notify all registered callbacks about the event
    for (auto& cb : registeredCallbacks) {
        cb->hasEvent(event);
    }
}

void MPU6050::registerCallback(ControllerCallbackInterface* ci) {
    registeredCallbacks.push_back(ci);
}

void MPU6050::attach_synth()
{
    // Create the synthesizer with this sensor as input
    synth = std::make_unique<MPUSynth>(this, 44100);
    
    // Configure the synthesizer
    synth->setScale(ScaleType::MAJOR, currentRootNote);
    synth->setInstrument(InstrumentType::PIANO);
    synth->setVolume(0.8f);
    synth->setNoteDuration(150.0f);
    synth->setMappingMode(true, true, true);
    synth->setAxisNoteSpreads(12, 12, 12);
    
    // Higher sensitivity values make smaller movements have bigger effects
    float xSens = 15.0f / ACCEL_THRESHOLD_POS;
    float ySens = 15.0f / ACCEL_THRESHOLD_POS;
    float zSens = 15.0f / ACCEL_THRESHOLD_POS_Z;
    synth->setAxisVelocitySensitivity(xSens, ySens, zSens);
    
    // Register the synthesizer as a callback
    registerCallback(synth.get());
    
    // Start the synthesizer
    synth->start();
}