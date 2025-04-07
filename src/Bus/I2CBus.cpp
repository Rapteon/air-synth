#include "Bus/I2CBus.h"
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

I2CBus::I2CBus(const char *device_file) {
  if ((file = open(device_file, O_RDWR)) < 0) {
    throw std::runtime_error("Failed to open I2C bus");
  }
}

I2CBus::~I2CBus() {
  close(file);
  std::cerr << "I2C bus closed\n";
}

void I2CBus::register_slave(int address) {
  if (ioctl(file, I2C_SLAVE, address) < 0) {
    throw std::runtime_error("Failed to initialize I2C slave");
  }
  std::cerr << "I2C slave registered at address " << address << "\n";
}

/**
 * @brief Write data specified by reg to the I2C bus.
 */
void I2CBus::sendData(int address, uint8_t *buffer, int byte_count) {
  write(file, buffer, byte_count);
}

/**
 * @brief Read data from the I2C bus.
 * @param address The I2C address of the slave device.
 * @param buffer The buffer to store the received data.
 * @param byte_count The number of bytes to read.
 * @return The number of bytes read.
 */
long I2CBus::receiveData(int address, uint8_t *buffer, int byte_count) {
  return read(file, buffer, byte_count);
}