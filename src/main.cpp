#include "IMU/MPU6050.h"
#include "Config.h"
#include "ConfigReader/MPUConfigReader.h"

int main()
{
    const char *GPIO_CHIP{"gpiochip0"};
    constexpr int LEFT_BTN_OFFSET{20};
    constexpr int RIGHT_BTN_OFFSET{21};
    constexpr int MPU6050_ADDR1{0x68};
    constexpr int MPU6050_ADDR2{0x69};

    // TODO: Move I2C_BUS to a config file and add a cmake variable for it.
    // Also rename it to device_file.
    I2CBus bus {I2C_BUS};
    
    MPUConfigReader leftConfig {LEFT_MPU_CONFIG_FILEPATH, MPU_SCHEMA_CONFIG_FILEPATH};
    MPUConfigReader rightConfig {RIGHT_MPU_CONFIG_FILEPATH, MPU_SCHEMA_CONFIG_FILEPATH};

    MPU6050 leftSensor{bus};
    leftConfig.configure(leftSensor);

    Button leftButton{};
    leftButton.registerCallback(&leftSensor);
    leftSensor.start();
    leftButton.start(GPIO_CHIP, LEFT_BTN_OFFSET);

    // MPU6050 rightSensor{MPU6050_ADDR2};
    // Button rightButton{};
    // rightButton.registerCallback(&rightSensor);
    // rightSensor.start();
    // rightButton.start(GPIO_CHIP, RIGHT_BTN_OFFSET);

    fprintf(stdout, "Press any key to stop\n");
    getchar();
    leftButton.stop();
    // rightButton.stop();
    return 0;
}