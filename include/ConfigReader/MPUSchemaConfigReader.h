#ifndef MPUSCHEMACONFIGREADER_H
#define MPUSCHEMACONFIGREADER_H
#include "ConfigReader/ConfigReader.h"
#include <string>

class MPUSchemaConfigReader : public ConfigReader {
public:
  MPUSchemaConfigReader(const char *file_path);

  std::string getDefaultInterface();
  int getDefaultAddress();
  int getDefaultCycleFreq();
  std::string getDefaultAccelRange();
  double getDefaultAccelScaling();
  int getDefaultZSamples();
  double getDefaultGravity();
  void isValidConfig() override {
    checkPropertiesKey();
    checkRequiredKeys();
    checkInterface();
    checkPowerManagement();
    checkAccel();
  }

private:
  void checkPropertiesKey();
  void checkRequiredKeys();
  void checkInterface();
  void checkPowerManagement();
  void checkAccel();
};

#endif