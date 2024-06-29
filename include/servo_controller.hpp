#ifndef __SERVO_CONTROLLER__
#define __SERVO_CONTROLLER__

#include <cmath>
#include <cstdint>
#include <memory>

#include "i2c.hpp"
#include "motor_driver.hpp"
#include "utils.hpp"



/*
Example Servo Specs
Model
P1S
Operating Voltage	6.0V ~ 8.4V
Idle Current	200mA
Stall Current	1500mA
Stall Torque	3kg*cm
Control Type	Digital PWM
Signal range:	500~2500µs
Dead Band:	≤2µs
Operating Travel	270°
Operating Speed	0.07 sec/60°
Sensor	Potentiometer
Size	30 x 24 x 12mm
Weight	14g
Ball-bearing	1 bearing
Gear Material	Metal
Motor	Coreless
Connector wire	7mm/17mm
Spline count	25

*/

class ServoBoardConfig {
public:
  ServoBoardConfig(uint8_t num_servos,
                   float32_t default_lower_angle_limit = -M_PI / 2,
                   float32_t default_upper_angle_limit = M_PI / 2,
                   float32_t default_zero_position = 0,
                   bool default_invert_servo_position = false,
                   float32_t min_angle_to_command = -M_PI/2,
                   float32_t max_angle_to_command = M_PI/2,
                   uint16_t min_pulsewidth_to_command = 500,
                   uint16_t max_pulsewidth_to_command = 2500);
  bool set_min_angle(const uint8_t &servo_num,
                     const float32_t &lower_angle_limit);
  bool get_min_angle(const uint8_t &servo_num, float32_t &min_angle);
  bool set_max_angle(const uint8_t &servo_num,
                     const float32_t &upper_angle_limit);
  bool get_max_angle(const uint8_t &servo_num, float32_t &max_angle);
  bool get_zero_position(const uint8_t &servo_num, float32_t &zero_pos);
  bool set_zero_position(const uint8_t &servo_num, const float32_t &zero_pos);
  bool get_invert_servo_flag(const uint8_t &servo_num, bool &invert_servo_pos);
  bool set_invert_servo_flag(const uint8_t &servo_num, const bool &invert_servo_pos);
  bool is_servo_num_valid(const uint8_t &servo_num);
  void servo_angle_to_pulsewidth(const float32_t& angle, uint16_t& pwm_out);
private:
  uint8_t num_servos_;
  float32_t cur_angle_helper_;
  float32_t min_angle_to_command_;
  float32_t max_angle_to_command_;
  uint16_t min_pulsewidth_to_command_;
  uint16_t max_pulsewidth_to_command_;
  float32_t angle_to_pulsewidth_slope_;
  std::shared_ptr<float32_t> min_angles_;
  std::shared_ptr<float32_t> max_angles_;
  std::shared_ptr<float32_t> zero_positions_;
  std::shared_ptr<bool> invert_servo_positions_;
};


class ServoController {
  public:
    ServoController(ServoBoardConfig* servo_config, MotorDriver* motor_driver);
    bool set_servo_angle(const uint8_t &servo_num,
                         const float32_t &servo_angle);
    void setPWM(uint8_t num, uint16_t on, uint16_t off);

  private:
    void angle_to_pwm(const uint8_t &servo_num, const float32_t& angle, uint16_t& pwm);
    float32_t cur_angle_helper_;
    ServoBoardConfig* servo_config_;
    MotorDriver* motor_driver_;
    float32_t cur_servo_zero_pos_;
    bool is_cur_servo_inverted_;
    uint16_t cur_pwm_;

};

#endif
