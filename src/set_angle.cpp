#include <iostream>
#include <string>

#include "i2c_linux.hpp"
#include "motor_driver.hpp"
#include "servo_controller.hpp"

int main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " pin_number angle_radians" << std::endl;
    std::cout << "Example: " << argv[0] << " 0 0.785" << std::endl;
    return 1;
  }

  std::string i2c_name = "/dev/i2c-1";
  I2CLinuxAPI i2c_dev(i2c_name);
  i2c_dev.begin();
  uint8_t num_servos = 16;
  ServoBoardConfig servo_config = ServoBoardConfig(num_servos);
  Adafruit_PWMServoDriver_Wrapper motor_driver(PCA9685_I2C_ADDRESS, &i2c_dev);
  ServoController servo_controller = ServoController(&servo_config,
                                 &motor_driver);
  
  const float angle = std::stof(argv[2]);
  const uint8_t pin_num = std::stoul(argv[1]);
  servo_controller.set_servo_angle(pin_num, angle);

  i2c_dev.close();

  std::cout << "Set pin " << pin_num << " to " << angle
            << std::endl;
  return 0;
}