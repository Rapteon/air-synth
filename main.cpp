#include "MPU6050.h"

#define MPU6050_ADDR1 0x68
#define MPU6050_ADDR2 0x69
#define BUTTON_GPIO1 20
#define BUTTON_GPIO2 21

int main() {
    MPU6050 sensor1(MPU6050_ADDR1), sensor2(MPU6050_ADDR2);
    sensor1.calibrate_z_axis();
    sensor2.calibrate_z_axis();
    sensor1.start_monitoring();
    sensor2.start_monitoring();

    thread(button_interrupt, ref(sensor1), BUTTON_GPIO1).detach();
    thread(button_interrupt, ref(sensor2), BUTTON_GPIO2).detach();

    while (true) this_thread::sleep_for(chrono::milliseconds(1000));
    return 0;
}