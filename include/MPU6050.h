#include <cstdint>
#include <mutex>
#include <atomic>

#define MPU6050_ADDR1 0x68
#define MPU6050_ADDR2 0x69
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F

#define I2C_BUS "/dev/i2c-1"
#define ACCEL_SCALE 16384.0
#define G 9.81
#define ACCEL_THRESHOLD_POS 5.0
#define ACCEL_THRESHOLD_NEG -5.0
#define ACCEL_THRESHOLD_POS_Z 3.0
#define ACCEL_THRESHOLD_NEG_Z -3.0
#define INCREMENT_VALUE 5
#define DECREMENT_VALUE 5

#define GPIO_CHIP "gpiochip0"
#define BUTTON_GPIO1 20
#define BUTTON_GPIO2 21
class MPU6050
{
public:
    MPU6050(int addr);

    ~MPU6050();

    int16_t read_word(int8_t reg);

    void calibrate_z_axis();

    void monitor_threshold();

    void start_monitoring();

    void set_active(bool state);

private:
    int file;
    int address;
    std::mutex mtx;
    std::atomic<bool> running;
    std::atomic<bool> active;
    int counter_x, counter_y, counter_z;
    bool threshold_crossed_x, threshold_crossed_y, threshold_crossed_z;
    double z_offset;
};