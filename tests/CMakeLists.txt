cmake_minimum_required(VERSION 3.10)
project(MPU6050_Test)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable testing
enable_testing()

# Comprehensive test executable only
add_executable(mpu6050_test mpu6050_tests.cpp)

# Add threading support
find_package(Threads REQUIRED)
target_link_libraries(mpu6050_test PRIVATE Threads::Threads)

# Add test case
add_test(NAME MPU6050Test COMMAND mpu6050_test)

# Add custom test command for easier running
add_custom_target(run_test
    COMMAND mpu6050_test
    DEPENDS mpu6050_test
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)