# Create build directory if it doesn't exist
mkdir -p build
cd build

# Configure and build with CMake
cmake ..
cmake --build .

# Run the test
./mpu6050_test

cd ../
rm -r build