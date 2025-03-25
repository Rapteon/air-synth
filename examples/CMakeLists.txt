add_executable(IMUInterruptExample main.cpp mpu.cpp gpioevent.cpp)
target_link_libraries(IMUInterruptExample gpiod)