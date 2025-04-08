#ifndef MPUCONFIGREADER_H
#define MPUCONFIGREADER_H
#include "ConfigReader/ConfigReader.h"
#include "ConfigReader/MPUSchemaConfigReader.h"
#include "Bus/I2CBus.h"
#include "IMU/MPU6050.h"
#include <unordered_set>

class MPUConfigReader : public ConfigReader {
public:
  MPUConfigReader(const char *file_path, const char *schema_file_path);

  // Add methods required for configuring MPU.
  // MPU will read and configure itself accordingly.

  void configure(MPU6050 &mpu);
  json getConfig();

private:
  const std::unordered_set<std::string> VALID_INTERFACES{"i2c", "spi"};
  const std::unordered_set<std::string> VALID_MODES{"sleep", "cycle", "reset"};
  const std::unordered_set<std::string> VALID_ACCEL_RANGES{"2g", "4g", "8g",
                                                           "16g"};
  MPUSchemaConfigReader schema_reader;

  virtual void isValidConfig() override {
    checkConfigKey();
    checkRequiredKeys();
    checkAddressValue();
    checkPowerManagement();
    checkAccel();
  }

  void checkConfigKey();
  void checkRequiredKeys();
  void checkInterfaceValue();
  void checkAddressValue();
  void checkPowerManagement();
  void checkAccel();

  std::string setToString(const std::unordered_set<std::string> &set);

  void throwMissingKeyError(const std::string &key);

  void configureAddress(const json &address, MPU6050 &mpu);
  void configurePowerManagement(const json &power_management, MPU6050 &mpu);
};
#endif