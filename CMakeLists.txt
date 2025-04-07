cmake_minimum_required(VERSION 3.25)

#[=[
The version string format is-> MajorVersion:MinorVersion:PatchVersion

Please update:
* PatchVersion when any bug is fixed.
* MinorVersion when a new feature is introduced.
* MajorVersion when a newly introduced feature breaks backward compatibility.
For more details about versioning, see https://semver.org

]=]#

project(AirSynth VERSION 0.0.1
        DESCRIPTION "A gesture-based music synthesizer."
        LANGUAGES CXX)

enable_testing()

# Use C++17
set(CMAKE_CXX_STANDARD 17)

add_library(Button STATIC src/Button/Button.cpp)
target_include_directories(Button PRIVATE include/Button)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

add_library(ConfigReader src/ConfigReader/ConfigReader.cpp)
target_include_directories(ConfigReader PRIVATE include/ConfigReader)
target_link_libraries(ConfigReader nlohmann_json::nlohmann_json)

add_library(MPUSchemaConfigReader src/ConfigReader/MPUSchemaConfigReader.cpp)
target_include_directories(MPUSchemaConfigReader PRIVATE include/ConfigReader)
target_link_libraries(MPUSchemaConfigReader ConfigReader)


add_library(MPUConfigReader src/ConfigReader/MPUConfigReader.cpp)
target_include_directories(MPUConfigReader PRIVATE include/ConfigReader)
target_link_libraries(MPUConfigReader ConfigReader)
target_link_libraries(MPUConfigReader MPUSchemaConfigReader)

add_executable(AirSynth src/main.cpp)
add_library(IMU STATIC src/IMU/MPU6050.cpp)
target_include_directories(IMU PRIVATE include/IMU)
target_link_libraries(IMU gpiod)
target_link_libraries(IMU Button)

include_directories(AirSynth include)
target_link_libraries(AirSynth gpiod)
target_link_libraries(AirSynth IMU)
target_link_libraries(AirSynth Button)
target_link_libraries(AirSynth nlohmann_json::nlohmann_json)
target_link_libraries(AirSynth MPUConfigReader)

# Set the path to the JSON configuration files.
set(LEFT_MPU_CONFIG_FILEPATH "${CMAKE_SOURCE_DIR}/config/IMU/left-mpu.json")
set(RIGHT_MPU_CONFIG_FILEPATH "${CMAKE_SOURCE_DIR}/config/IMU/right-mpu.json")
set(MPU_SCHEMA_CONFIG_FILEPATH "${CMAKE_SOURCE_DIR}/config/IMU/mpu.schema.json")

configure_file(${CMAKE_SOURCE_DIR}/include/Config.h.in
               ${CMAKE_BINARY_DIR}/generated/Config.h
               @ONLY)
target_include_directories(AirSynth PRIVATE ${CMAKE_BINARY_DIR}/generated)