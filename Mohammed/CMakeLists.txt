cmake_minimum_required(VERSION 3.10)
project(AirSynth VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Set compilation flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Add real-time scheduling flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lrt")

# Find required packages
find_package(Threads REQUIRED)
find_library(ALSA_LIBRARY asound)
find_library(PIGPIO_LIBRARY pigpio)

# Check if running on Raspberry Pi
execute_process(
    COMMAND grep -q "Raspberry Pi" /proc/cpuinfo
    RESULT_VARIABLE NOT_RPI
    OUTPUT_QUIET
    ERROR_QUIET
)

if(NOT_RPI EQUAL 0)
    message(STATUS "Detected Raspberry Pi platform")
    add_definitions(-DRASPBERRY_PI)
else()
    message(STATUS "Building for generic platform")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Define source files
set(SOURCES
    src/MPU6050.cpp
    src/GestureDetector.cpp
    src/Synthesizer.cpp
    src/AirSynth.cpp
    src/main.cpp
)

# Define headers
set(HEADERS
    include/MPU6050.hpp
    include/GestureDetector.hpp
    include/Synthesizer.hpp
    include/AirSynth.hpp
)

# Create executable
add_executable(air-synth ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(air-synth
    PRIVATE
    Threads::Threads
    ${ALSA_LIBRARY}
    ${PIGPIO_LIBRARY}
    m
    rt
)

# Installation rules
install(TARGETS air-synth DESTINATION bin)

# Add tests subdirectory if tests are enabled
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# Print configuration information
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")

# Add a target for generating documentation
add_custom_target(doc
    COMMAND doxygen Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
)