#ifndef __SERVO_CONTROLLER__
#define __SERVO_CONTROLLER__

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "i2c.hpp"
#include "motor_driver.hpp"
#include "utils.hpp"



uint8_t calc_pca9685_prescaler(float32_t servo_pwm_freq, uint32_t pca9685_oscillator_freq);
float32_t calc_microseconds_to_pulse(uint16_t microseconds, uint8_t pca9685_prescaler,  uint32_t pca9685_oscillator_freq);
float32_t calc_angle_to_pulsewidth_slope(uint16_t min_microseconds_to_command, uint16_t max_microseconds_to_command,
                                         float32_t angular_range, float32_t microseconds_to_pulse);

class ServoBoardConfig {
public:
  ServoBoardConfig(uint8_t num_servos,
                   float32_t default_lower_angle_limit = -90.0,
                   float32_t default_upper_angle_limit = 90.0,
                   float32_t default_zero_position = 0,
                   bool default_invert_servo_position = false,
                   float32_t angular_range = 270,
                   uint16_t min_microseconds_to_command = 500,
                   uint16_t max_microseconds_to_command = 2500,
                   uint16_t min_pulsewidth_to_command = 150,
                   uint16_t max_pulsewidth_to_command = 600,
                   float32_t servo_pwm_frequency = 60,
                   uint32_t pca9685_oscillator_freq = 25000000);
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
  bool servo_angle_to_adj_angle(const uint8_t & servo_num, const float32_t& angle, float32_t& angle_out);
  bool servo_angle_to_pulsewidth(const uint8_t & servo_num, const float32_t& angle, uint16_t& pwm_out);
  float32_t get_servo_pwm_freq();
  bool get_servo_pwm_pin_num(const uint8_t &servo_num, uint8_t& pwm_pin);
  bool set_servo_pwm_pin_num(const uint8_t &servo_num, const uint8_t &pwm_pin);
  bool get_servo_angular_range(const uint8_t &servo_num, float32_t& angular_range);
  bool set_servo_angular_range(const uint8_t &servo_num, const float32_t& angular_range);
  uint8_t get_num_servos();
  float32_t get_microseconds_to_pulse();
  uint8_t get_pca9685_prescaler_value();
  std::string to_string();

private:


  uint8_t num_servos_;
  uint16_t min_microseconds_to_command_;
  uint16_t max_microseconds_to_command_;
  uint16_t min_pulsewidth_to_command_;
  uint16_t max_pulsewidth_to_command_;
  uint16_t pulsewidth_offset_;
  float32_t servo_pwm_freq_;
  uint32_t pca9685_oscillator_freq_;
  float32_t microseconds_to_pulse_;
  uint8_t pca9685_prescaler_value_;
  std::vector<float32_t> min_angles_;
  std::vector<float32_t> max_angles_;
  std::vector<float32_t> angular_ranges_;
  std::vector<float32_t> angle_to_pulsewidth_slopes_;
  std::vector<float32_t> zero_positions_;
  std::vector<bool> invert_servo_positions_;
  std::vector<uint8_t> servo_pwm_pin_num_;

};


class ServoController {
  public:
    ServoController(ServoBoardConfig* servo_config, MotorDriver* motor_driver,
                                 bool init_to_zero = false);
    bool set_servo_angle(const uint8_t &servo_num,
                         const float32_t &servo_angle);
    void setPWM(uint8_t num, uint16_t on, uint16_t off);
    uint16_t getPWM(uint8_t num, bool off = true);
    float32_t readPwmFreq();

  private:
    void angle_to_pwm(const uint8_t &servo_num, const float32_t& angle, uint16_t& pwm);
    ServoBoardConfig* servo_config_;
    MotorDriver* motor_driver_;
    std::vector<float32_t> cur_angles_;
    std::vector<float32_t> cur_angles_adj_;
    std::vector<uint16_t> cur_pwm_;

};

#endif
