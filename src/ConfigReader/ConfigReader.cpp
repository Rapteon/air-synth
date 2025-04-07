#include "ConfigReader/ConfigReader.h"
#include <fstream>

ConfigReader::ConfigReader(const char *file_path) {
  input_file.open(file_path);
  if (!input_file.is_open()) {
    throw std::runtime_error("Failed to open config file");
  }
  try {
    data = json::parse(input_file);
  } catch (const json::parse_error &e) {
    throw std::runtime_error("Failed to parse JSON: " + std::string(e.what()));
  }
}

ConfigReader::~ConfigReader() {
  // Destructor implementation if needed
  if (input_file.is_open())
    input_file.close();
}

json ConfigReader::getData() { return data; }
