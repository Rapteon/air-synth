#include "ConfigReader/MPUSchemaConfigReader.h"
#include <list>
#include <stdexcept>

MPUSchemaConfigReader::MPUSchemaConfigReader(const char *file_path)
    : ConfigReader(file_path) {
  isValidConfig();
}

void MPUSchemaConfigReader::checkPropertiesKey() {
  if (!data.contains("properties")) {
    throw std::runtime_error("Missing 'properties' in JSON");
  }
}

void MPUSchemaConfigReader::checkRequiredKeys() {
  const std::list<std::string> VALID_PROPERTIES{"interface", "address",
                                                "power-management", "accel"};
  auto properties = data["properties"];

  // Check if any required keys are missing.
  for (auto i : VALID_PROPERTIES) {
    if (!properties.contains(i)) {
      throw std::runtime_error("Missing '" + i + "' in JSON");
      break;
    }
  }
}
void MPUSchemaConfigReader::checkInterface() {
  auto properties = data["properties"];
  if (!properties.contains("interface")) {
    throw std::runtime_error("Missing 'interface' in properties");
  }

  auto interface = properties["interface"];
  if (!interface.contains("default")) {
    throw std::runtime_error("Missing 'default' in interface");
  }
}

void MPUSchemaConfigReader::checkPowerManagement() {
  auto properties = data["properties"];
  if (!properties.contains("power-management")) {
    throw std::runtime_error("Missing 'power-management' in properties");
  }

  auto power_management = properties["power-management"];
  if (!power_management.contains("properties")) {
    throw std::runtime_error("Missing 'properties' in power-management");
  }

  //   Check if default value for cycle-frequency is present.
  properties = power_management["properties"];
  if (!properties.contains("cycle-frequency")) {
    throw std::runtime_error(
        "Missing 'cycle-frequency' in power-management properties");
  }
  auto cycle_frequency = properties["cycle-frequency"];
  if (!cycle_frequency.contains("default")) {
    throw std::runtime_error("Missing 'default' in cycle-frequency");
  }
}

void MPUSchemaConfigReader::checkAccel() {
  auto properties = data["properties"];
  if (!properties.contains("accel")) {
    throw std::runtime_error("Missing 'accel' in properties");
  }

  auto accel = properties["accel"];
  if (!accel.contains("properties")) {
    throw std::runtime_error("Missing 'properties' in accel");
  }

  // Check properties of 'accel'.
  auto accel_properties = accel["properties"];

  // Check if 'range' is present.
  if (!accel_properties.contains("range")) {
    throw std::runtime_error("Missing 'range' in accel properties");
  }
  // Check if 'default' key is present in 'range'.
  auto range = accel_properties["range"];
  if (!range.contains("default")) {
    throw std::runtime_error("Missing 'default' in range");
  }

  // Check if 'scaling' is present.
  if (!accel_properties.contains("scaling")) {
    throw std::runtime_error("Missing 'scaling' in accel properties");
  }
  // Check if 'default' key is present in 'scaling'.
  auto scaling = accel_properties["scaling"];
  if (!scaling.contains("default")) {
    throw std::runtime_error("Missing 'default' in scaling");
  }

  // Check if 'z' is present in 'accel'.
  if (!accel_properties.contains("z")) {
    throw std::runtime_error("Missing 'z' in accel properties");
  }

  // Check if 'properties' key is present in 'z'.
  auto z = accel_properties["z"];
  if (!z.contains("properties")) {
    throw std::runtime_error("Missing 'properties' in z");
  }

  auto z_properties = z["properties"];
  // Check if 'samples' is present in 'z_properties'.
  if (!z_properties.contains("samples")) {
    throw std::runtime_error("Missing 'samples' in z_properties");
  }
  // Check if 'default' key is present in 'samples'.
  auto samples = z_properties["samples"];
  if (!samples.contains("default")) {
    throw std::runtime_error("Missing 'default' in samples");
  }

  // Check if 'gravity' is present in 'accel'.
  if (!accel_properties.contains("gravity")) {
    throw std::runtime_error("Missing 'gravity' in accel properties");
  }
  // Check if 'default' key is present in 'gravity'.
  auto gravity = accel_properties["gravity"];
  if (!gravity.contains("default")) {
    throw std::runtime_error("Missing 'default' in gravity");
  }
}

std::string MPUSchemaConfigReader::getDefaultInterface() {
  auto data = getData();
  auto properties = data["properties"];
  auto interface = properties["interface"];
  return interface["default"];
}

int MPUSchemaConfigReader::getDefaultAddress() {
  auto data = getData();
  auto properties = data["properties"];
  auto address = properties["address"];
  return address["default"];
}

int MPUSchemaConfigReader::getDefaultCycleFreq() {
  auto data = getData();
  auto properties = data["properties"];
  auto power_management = properties["power-management"];
  auto power_management_properties = power_management["properties"];
  auto cycle_frequency = power_management_properties["cycle-frequency"];
  return cycle_frequency["default"];
}

std::string MPUSchemaConfigReader::getDefaultAccelRange() {
  auto data = getData();
  auto properties = data["properties"];
  auto accel = properties["accel"];
  auto accel_properties = accel["properties"];
  auto range = accel_properties["range"];
  return range["default"];
}

double MPUSchemaConfigReader::getDefaultAccelScaling() {
  auto data = getData();
  auto properties = data["properties"];
  auto accel = properties["accel"];
  auto accel_properties = accel["properties"];
  auto scaling = accel_properties["scaling"];
  return scaling["default"];
}

int MPUSchemaConfigReader::getDefaultZSamples() {
  auto data = getData();
  auto properties = data["properties"];
  auto accel = properties["accel"];
  auto accel_properties = accel["properties"];
  auto z = accel_properties["z"];
  auto z_properties = z["properties"];
  auto samples = z_properties["samples"];
  return samples["default"];
}

double MPUSchemaConfigReader::getDefaultGravity() {
  auto data = getData();
  auto properties = data["properties"];
  auto accel = properties["accel"];
  auto accel_properties = accel["properties"];
  auto gravity = accel_properties["gravity"];
  return gravity["default"];
}