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
  float32_t default_lower_angle_limit = -90;
  float32_t default_upper_angle_limit = 90;
  float32_t default_zero_position = -55;
  bool default_invert_servo_position = false;
  float angular_range = 270;
  uint16_t min_microseconds_to_command = 500;
  uint16_t max_microseconds_to_command = 2500;
  uint16_t min_pulsewidth_to_command = 100;
  uint16_t max_pulsewidth_to_command = 1500;
  float32_t servo_pwm_frequency = 50;
  uint32_t pca9685_oscillator_freq = 25000000;

  ServoBoardConfig servo_config = ServoBoardConfig(num_servos, default_lower_angle_limit, default_upper_angle_limit
                                                   default_zero_position, default_invert_servo_position,
                                                   angular_range, min_microseconds_to_command, max_microseconds_to_command,
                                                   min_pulsewidth_to_command, max_pulsewidth_to_command, servo_pwm_frequency
                                                   pca9685_oscillator_freq);
  std::string log_str = servo_config.to_string();
  std::cout << "servo board config str:" << std::endl << log_str << std::endl;
  Adafruit_PWMServoDriver_Wrapper motor_driver(PCA9685_I2C_ADDRESS, &i2c_dev);
  ServoController servo_controller = ServoController(&servo_config,
                                 &motor_driver);
  
  const uint16_t pwm = std::stoul(argv[2]);
  const uint8_t pin_num = std::stoul(argv[1]);

  servo_controller.setPWM(pin_num, 0, pwm);
  uint16_t cur_pwm = servo_controller.getPWM(pin_num, true);
  float32_t cur_freq = servo_controller.readPwmFreq();

  i2c_dev.close();

  std::cout << "Set pin " << std::to_string(pin_num) << " to " << std::to_string(pwm)
            << std::endl;
  std::cout << "Pwm was set to " << std::to_string(cur_pwm) << std::endl;
  std::cout << "Pwm frequency is " << std::to_string(cur_freq) << std::endl;
  return 0;
}