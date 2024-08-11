#include "servo_controller.hpp"

bool ServoBoardConfig::is_servo_num_valid(const uint8_t &servo_num) {
  return servo_num < num_servos_;
}

ServoBoardConfig::ServoBoardConfig(
    uint8_t num_servos, float32_t default_lower_angle_limit,
    float32_t default_upper_angle_limit, float32_t default_zero_position,
    bool default_invert_servo_position, float32_t angular_range,
    uint16_t min_microseconds_to_command,
    uint16_t max_microseconds_to_command,
    uint16_t min_pulsewidth_to_command, uint16_t max_pulsewidth_to_command,
    float32_t servo_pwm_frequency, 
    uint32_t pca9685_oscillator_freq) {
  num_servos_ = num_servos;

  servo_pwm_freq_ = servo_pwm_frequency;
  angular_ranges_ = std::vector<float32_t>(num_servos_, angular_range);

  min_microseconds_to_command_ = min_microseconds_to_command;
  max_microseconds_to_command_ = max_microseconds_to_command;
  min_pulsewidth_to_command_ = min_pulsewidth_to_command;
  max_pulsewidth_to_command_ = max_pulsewidth_to_command;
  pca9685_oscillator_freq_ = pca9685_oscillator_freq;
  pca9685_prescaler_value_ = calc_pca9685_prescaler(servo_pwm_freq_, pca9685_oscillator_freq_);
  microseconds_to_pulse_ = calc_microseconds_to_pulse(1, pca9685_prescaler_value_,  pca9685_oscillator_freq_);
  float32_t angle_to_pulsewidth_slope = calc_angle_to_pulsewidth_slope(min_microseconds_to_command_, max_microseconds_to_command_,
                      angular_range, microseconds_to_pulse_);
  angle_to_pulsewidth_slopes_ = std::vector<float32_t>(num_servos_, angle_to_pulsewidth_slope);
  // servo_freq / default_freq * min_pulsewidth
  pulsewidth_offset_ = servo_pwm_freq_ / 50 * min_pulsewidth_to_command_;
  min_angles_ = std::vector<float32_t>(num_servos_);
  max_angles_ = std::vector<float32_t>(num_servos_);
  zero_positions_ = std::vector<float32_t>(num_servos_);
  servo_pwm_pin_num_ = std::vector<uint8_t>(num_servos_, 0);
  invert_servo_positions_ = std::vector<bool>(num_servos_);
  for (uintmax_t i = 0; i < num_servos_; i++) {
    min_angles_[i] = default_lower_angle_limit;
    max_angles_[i] = default_upper_angle_limit;
    zero_positions_[i] = default_zero_position;
    invert_servo_positions_[i] = default_invert_servo_position;
    servo_pwm_pin_num_[i] = i;
  }
}

bool ServoBoardConfig::servo_angle_to_adj_angle(const uint8_t &servo_num,
                                                const float32_t &angle,
                                                float32_t &angle_out) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }

  // some servos we want to flip the angular convention in software, given the
  // servo's installation position can't be flipped
  angle_out = angle;
  if (invert_servo_positions_[servo_num]) {
    angle_out *= -1;
  }

  // in some robot setups (like the Bittle robot) the middle of the pulse width
  // range (1500) does not correspond to zero degrees, first we account for
  // that. Then we account for max/min angles.
  angle_out = angle_out + zero_positions_[servo_num];
  angle_out = max(angle_out, min_angles_[servo_num]);
  angle_out = min(angle_out, max_angles_[servo_num]);
  return true;
}

bool ServoBoardConfig::servo_angle_to_pulsewidth(const uint8_t &servo_num,
                                                 const float32_t &angle,
                                                 uint16_t &pwm_out) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  const float32_t& angle_to_pulsewidth_slope = angle_to_pulsewidth_slopes_[servo_num];
  pwm_out = round(angle_to_pulsewidth_slope / 1000.0 * angle + pulsewidth_offset_);
  if (pwm_out > max_microseconds_to_command_)
  {
    return false;
  }
  if (pwm_out < min_microseconds_to_command_)
  {
    return false;
  }
  return true;
}

bool ServoBoardConfig::set_min_angle(const uint8_t &servo_num,
                                     const float32_t &lower_angle_limit) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  min_angles_[servo_num] = lower_angle_limit;
  return true;
}

bool ServoBoardConfig::set_max_angle(const uint8_t &servo_num,
                                     const float32_t &upper_angle_limit) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  max_angles_[servo_num] = upper_angle_limit;
  return true;
}

bool ServoBoardConfig::get_min_angle(const uint8_t &servo_num,
                                     float32_t &min_angle) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  min_angle = min_angles_[servo_num];
  return true;
}

bool ServoBoardConfig::get_max_angle(const uint8_t &servo_num,
                                     float32_t &max_angle) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  max_angle = max_angles_[servo_num];
  return true;
}

bool ServoBoardConfig::get_zero_position(const uint8_t &servo_num,
                                         float32_t &zero_pos) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }

  zero_pos = zero_positions_[servo_num];

  return true;
}
bool ServoBoardConfig::set_zero_position(const uint8_t &servo_num,
                                         const float32_t &zero_pos) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  zero_positions_[servo_num] = zero_pos;
  return true;
}

bool ServoBoardConfig::get_invert_servo_flag(const uint8_t &servo_num,
                                             bool &invert_servo_pos) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  invert_servo_pos = invert_servo_positions_[servo_num];
  return true;
}

bool ServoBoardConfig::set_invert_servo_flag(const uint8_t &servo_num,
                                             const bool &invert_servo_pos) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  invert_servo_positions_[servo_num] = invert_servo_pos;
  return true;
}

float32_t ServoBoardConfig::get_servo_pwm_freq() { return servo_pwm_freq_; }

bool ServoBoardConfig::get_servo_pwm_pin_num(const uint8_t &servo_num,
                                             uint8_t &pwm_pin) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  pwm_pin = servo_pwm_pin_num_[servo_num];
  return true;
}
bool ServoBoardConfig::set_servo_pwm_pin_num(const uint8_t &servo_num,
                                             const uint8_t &pwm_pin) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  servo_pwm_pin_num_[servo_num] = pwm_pin;
  return true;
}

uint8_t ServoBoardConfig::get_num_servos() { return num_servos_; }

float32_t ServoBoardConfig::get_microseconds_to_pulse()
{
  return microseconds_to_pulse_;
}

uint8_t ServoBoardConfig::get_pca9685_prescaler_value() 
{
  return pca9685_prescaler_value_;
}

bool ServoBoardConfig::get_servo_angular_range(const uint8_t &servo_num, float32_t& angular_range)
{
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  angular_range = angular_ranges_[servo_num];
  return true;
}
bool ServoBoardConfig::set_servo_angular_range(const uint8_t &servo_num, const float32_t& angular_range)
{

  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  angular_ranges_[servo_num] = angular_range;
  angle_to_pulsewidth_slopes_[servo_num] = calc_angle_to_pulsewidth_slope(min_pulsewidth_to_command_,
                                                                          max_microseconds_to_command_,
                                      angular_range, microseconds_to_pulse_);

  return true;


}

uint8_t calc_pca9685_prescaler(float32_t servo_pwm_freq, uint32_t pca9685_oscillator_freq)
{
  // calculate prescaler based on Equation 1 from datasheet section 7.3.5
  float32_t denominator = static_cast<float32_t>(4096 * servo_pwm_freq);
  float32_t prescaler_pre_rounding = static_cast<float32_t>(pca9685_oscillator_freq) / denominator;
  uint8_t prescaler_value = round(prescaler_pre_rounding) - 1;
  return prescaler_value;
}


float32_t calc_microseconds_to_pulse(uint16_t microseconds, uint8_t pca9685_prescaler,  uint32_t pca9685_oscillator_freq)
{
  // Calculate the pulse for PWM based on Equation 1 from the datasheet section 7.3.5
  float32_t pulse = microseconds;
  float32_t pulse_length = 1000000;  // 1,000,000 us per second
  pca9685_prescaler += 1;
  pulse_length *= pca9685_prescaler;
  pulse_length /= pca9685_oscillator_freq;
  pulse /= pulse_length;
  return pulse;
}

float32_t calc_angle_to_pulsewidth_slope(uint16_t min_microseconds_to_command, uint16_t max_microseconds_to_command,
                                    float32_t angular_range, float32_t microseconds_to_pulse)
{
  float32_t angle_to_pulsewidth_slope = 1.0 * (max_microseconds_to_command - min_microseconds_to_command) /
      (angular_range);
  angle_to_pulsewidth_slope *= microseconds_to_pulse;
  return angle_to_pulsewidth_slope;
}

ServoController::ServoController(ServoBoardConfig *servo_config,
                                 MotorDriver *motor_driver,
                                 bool init_to_zero) {
  servo_config_ = servo_config;
  motor_driver_ = motor_driver;
  uint8_t servo_num = servo_config_->get_num_servos();
  cur_angles_ = std::vector<float32_t>(servo_num, 0);
  cur_angles_adj_ = std::vector<float32_t>(servo_num, 0);
  cur_pwm_ = std::vector<uint16_t>(servo_num, 0);

  // set freq of pwm
  motor_driver_->setPwmFreq(servo_config_->get_servo_pwm_freq());
  if (init_to_zero)
  {
    for (uint8_t i = 0; i < servo_num; i++) {
      set_servo_angle(i, 0);
    }
  }
}

bool ServoController::set_servo_angle(const uint8_t &servo_num,
                                      const float32_t &servo_angle) {
  if (!servo_config_->is_servo_num_valid(servo_num)) {
    log_msg("invalid servo number");
    return false;
  }
  cur_angles_[servo_num] = servo_angle;
  bool success = servo_config_->servo_angle_to_adj_angle(
      servo_num, cur_angles_[servo_num], cur_angles_adj_[servo_num]);
  success &= servo_config_->servo_angle_to_pulsewidth(
      servo_num, cur_angles_adj_[servo_num], cur_pwm_[servo_num]);
  log_msg("Would have set servo " + std::to_string(servo_num) + std::to_string(cur_pwm_[servo_num]) );
  //motor_driver_->setPWM(servo_num, 0, cur_pwm_[servo_num]);
  if (!success) {
    log_msg("could not adjust angle and calc pulsewidth");
    return false;
  }

  return true;
}

void ServoController::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  motor_driver_->setPWM(num, on, off);
}
