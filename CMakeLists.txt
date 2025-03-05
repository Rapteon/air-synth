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

add_executable(AirSynth src/mpu6050.cpp)

target_link_libraries(AirSynth gpiod)
