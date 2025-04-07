#ifndef CONFIGREADER_H
#define CONFIGREADER_H

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

using json = nlohmann::json;

class ConfigReader {
public:
  ConfigReader(const char *file_path);

  ~ConfigReader();

protected:
  std::ifstream input_file;
  json data{};

  json getData();

  /**
   * @brief Validates the configuration data.
   *
   * This method checks if the configuration data is valid.
   * It must be implemented by derived classes. Preferably call this in
   * the constructor.
   *
   * @throws std::runtime_error if the configuration is incorrect.
   */
  virtual void isValidConfig() = 0;
};
#endif