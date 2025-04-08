#include "ConfigReader/MPUConfigReader.h"
#include "Bus/I2CBus.h"
#include "IMU/MPU6050.h"
#include <list>
#include <sstream>
#include <stdexcept>

MPUConfigReader::MPUConfigReader(const char *file_path,
                                 const char *schema_file_path)
    : ConfigReader(file_path), schema_reader(schema_file_path) {
  isValidConfig();
}

void MPUConfigReader::configure(MPU6050 &mpu) {
  /**
1. Register MPU to Bus by reading address from JSON file.
  2. Configure power management settings
  3. Set callibration samples variable.
  4. Set ACCEL_SCALE variable
  5. Set ACCEL_ZOUT_H variable
  5. Set Gravity G.
  5. Set ACCEL_THRESHOLD_POS variable, and for a-axis
  6. Set ACCEL_THRESHOLD_NEG variable, and for z-axis.
  7. Set INCREMENT_VALUE variable
   */
  auto config = getConfig();
  configureAddress(config["address"], mpu);
  configurePowerManagement(config["power-management"], mpu);
}

void MPUConfigReader::throwMissingKeyError(const std::string &key) {
  throw std::runtime_error("Missing '" + key + "' in config");
}

void MPUConfigReader::checkConfigKey() {
  if (!data.contains("config")) {
    throwMissingKeyError("Missing 'config' in JSON");
    return;
  }
}

void MPUConfigReader::checkRequiredKeys() {
  const std::list<std::string> VALID_CONFIG_KEYS{"interface", "address",
                                                 "power-management", "accel"};
  auto config = getConfig();

  //   Check if any required keys are missing.
  for (auto i : VALID_CONFIG_KEYS) {
    if (!config.contains(i)) {
      throwMissingKeyError(i);
      break;
    }
  }
}

void MPUConfigReader::checkAddressValue() {
  auto config = getConfig();

  // The Raspberry Pi I2C bus supports addresses from 0 to 127.
  if (config["address"] < 0 || config["address"] > 127) {
    throw std::runtime_error("Invalid address value. Must lie with 0 and 127.");
  }
}

void MPUConfigReader::checkPowerManagement() {
  auto config = getConfig();
  auto power_management = config["power-management"];

  if (!power_management.contains("mode")) {
    throw std::runtime_error("Missing 'mode' in power-management settings");
  }

  auto mode = power_management["mode"];

  if (VALID_MODES.find(mode) == VALID_MODES.end()) {
    throw std::runtime_error("Invalid mode value. Must be one of: " +
                             setToString(VALID_MODES));
  }

  if (mode == "cycle") {
    // TODO implement check for cycle-frequency
  }
}

void MPUConfigReader::checkAccel() {
  auto config = getConfig();
  auto accel = config["accel"];

  // Checks for "range" config.
  if (!accel.contains("range")) {
    throw std::runtime_error("Missing 'range' in accel settings");
  }

  if (VALID_ACCEL_RANGES.find(accel["range"]) == VALID_ACCEL_RANGES.end()) {
    throw std::runtime_error("Invalid range value. Must be one of: " +
                             setToString(VALID_ACCEL_RANGES));
  }

  // Checks for "scaling" config.
  if (!accel.contains("scaling")) {
    // TODO implement schema reader to get dedfault value for accel scaling.
    throw std::runtime_error("Missing 'scale' in accel settings");
  }

  if (accel["scaling"] <= 0) {
    throw std::runtime_error("Invalid scaling value. Must not be less than 0");
  }
}

std::string
MPUConfigReader::setToString(const std::unordered_set<std::string> &set) {
  std::ostringstream oss;
  for (auto iter = set.begin(); iter != set.end(); ++iter) {
    oss << *iter;
    if (std::next(iter) != set.end()) {
      oss << ", ";
    }
  }
  return oss.str();
}

json MPUConfigReader::getConfig() { return data["config"]; }

void MPUConfigReader::configureAddress(const json &address, MPU6050 &mpu) {
  mpu.setAddress(address);
  I2CBus bus{mpu.getBus()};
  bus.register_slave(address);
  std::cerr << "Address configured." << '\n';
}

void MPUConfigReader::configurePowerManagement(const json &power_management,
                                               MPU6050 &mpu) {
  uint8_t PWR_MGMT_REG_VALUE{};
  if (power_management == "reset") {
    PWR_MGMT_REG_VALUE = 0;
  } else if (power_management == "sleep") {
    // TODO: implement
  } else if (power_management == "cycle") {
    // TODO: implement
  } else {
    throw std::runtime_error(
        "Invalid power-management value when configuring.");
  }
  const uint8_t PWR_MGMT_REG_ADDR{107};
  constexpr size_t BUFFER_SIZE{2};
  uint8_t config_buffer[BUFFER_SIZE] = {PWR_MGMT_REG_ADDR, PWR_MGMT_REG_VALUE};
  I2CBus bus{mpu.getBus()};
  int address = mpu.getAddress();
  bus.sendData(address, config_buffer, BUFFER_SIZE);
  std::cerr << "Power management configured." << '\n';
}