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

# Add include directories
include_directories(${CMAKE_SOURCE_DIR})

# Button component
add_library(Button STATIC Button/Button.cpp)
target_include_directories(Button PRIVATE ${CMAKE_SOURCE_DIR})

# ControllerEvent component
add_library(ControllerEvent STATIC ControllerEvent/ControllerEvent.cpp)
target_include_directories(ControllerEvent PRIVATE ${CMAKE_SOURCE_DIR})

# IMU component
add_library(IMU STATIC IMU/MPU6050.cpp)
target_include_directories(IMU PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(IMU gpiod)
target_link_libraries(IMU Button)

# Synth component
add_library(Synth STATIC Synth/Synth.cpp)
target_include_directories(Synth PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(Synth stk)
target_link_libraries(Synth rtaudio)

# Main executable
add_executable(AirSynth main.cpp)
target_link_libraries(AirSynth gpiod)
target_link_libraries(AirSynth IMU)
target_link_libraries(AirSynth Button)
target_link_libraries(AirSynth ControllerEvent)
target_link_libraries(AirSynth Synth)