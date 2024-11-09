#include <iostream>
#include <string>

#include "i2c_linux.hpp"
#include "motor_driver.hpp"
#include "servo_controller.hpp"

int main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " pin_number pwm" << std::endl;
    std::cout << "Example: " << argv[0] << " 0 2000" << std::endl;
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
  
  const uint16_t pwm = std::stoul(argv[2]);
  const uint8_t pin_num = std::stoul(argv[1]);
  servo_controller.setPWM(pin_num, 0, pwm);

  i2c_dev.close();

  std::cout << "Set pin " << std::to_string(pin_num) << " to " << std::to_string(pwm)
            << std::endl;
  return 0;
}