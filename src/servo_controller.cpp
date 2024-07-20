#include "servo_controller.hpp"

bool ServoBoardConfig::is_servo_num_valid(const uint8_t &servo_num) {
  return servo_num < num_servos_;
}

ServoBoardConfig::ServoBoardConfig(
    uint8_t num_servos, float32_t default_lower_angle_limit,
    float32_t default_upper_angle_limit, float32_t default_zero_position,
    bool default_invert_servo_position, float32_t angular_range,
    uint16_t min_pulsewidth_to_command, uint16_t max_pulsewidth_to_command,
    float32_t servo_pwm_frequency, 
    uint32_t pca9685_oscillator_freq) {
  num_servos_ = num_servos;

  servo_pwm_freq_ = servo_pwm_frequency;
  angular_range_ = angular_range;
  min_pulsewidth_to_command_ = min_pulsewidth_to_command;
  max_pulsewidth_to_command_ = max_pulsewidth_to_command;
  angle_to_pulsewidth_slope_ =
      1.0 * (max_pulsewidth_to_command_ - min_pulsewidth_to_command_) /
      (angular_range_);
  pca9685_oscillator_freq_ = pca9685_oscillator_freq;
  min_angles_ = std::vector<float32_t>(num_servos_);
  max_angles_ = std::vector<float32_t>(num_servos_);
  max_angles_ = std::vector<float32_t>(num_servos_);
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

  pwm_out =
      min_pulsewidth_to_command_ + round(angle_to_pulsewidth_slope_ * angle);
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

void ServoBoardConfig::set_servo_pwm_freq(const float32_t &freq) {
  servo_pwm_freq_ = freq;
}

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

ServoController::ServoController(ServoBoardConfig *servo_config,
                                 MotorDriver *motor_driver) {
  servo_config_ = servo_config;
  motor_driver_ = motor_driver;
  uint8_t servo_num = servo_config_->get_num_servos();
  cur_angles_ = std::vector<float32_t>(servo_num, 0);
  cur_angles_adj_ = std::vector<float32_t>(servo_num, 0);
  cur_pwm_ = std::vector<uint16_t>(servo_num, 0);

  // set freq of pwm
  motor_driver_->setPwmFreq(servo_config_->get_servo_pwm_freq());
  // set all servos to zero pos
  for (uint8_t i = 0; i < servo_num; i++) {
    set_servo_angle(i, 0);
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
  motor_driver_->setPWM(servo_num, 0, cur_pwm_[servo_num]);
  if (!success) {
    log_msg("could not adjust angle and calc pulsewidth");
    return false;
  }

  return true;
}

void ServoController::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  motor_driver_->setPWM(num, on, off);
}

// void calibratedPWM(byte i, float angle, float speedRatio = 0) {
//   /*float angle = max(-SERVO_ANG_RANGE/2, min(SERVO_ANG_RANGE/2, angle));
//     if (i > 3 && i < 8)
//     angle = max(-5, angle);*/
//   angle = max(float(loadAngleLimit(i, 0)), min(float(loadAngleLimit(i, 1)),
//   angle)); int duty0 = EEPROMReadInt(CALIBRATED_ZERO_POSITIONS + i * 2) +
//   currentAng[i] * eeprom(ROTATION_DIRECTION, i); currentAng[i] = angle;

//   int duty = EEPROMReadInt(CALIBRATED_ZERO_POSITIONS + i * 2) + angle *
//   eeprom(ROTATION_DIRECTION, i); int steps = speedRatio > 0 ?
//   int(round(abs(duty - duty0) / 1.0 /*degreeStep*/ / speedRatio)) : 0;
//   //if default speed is 0, no interpolation will be used
//   //otherwise the speed ratio is compared to 1 degree per second.
//   for (int s = 0; s <= steps; s++) {
//     pwm.writeAngle(i, duty + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) /
//     2 * (duty0 - duty)));
//   }
// }

// void Adafruit_PWMServoDriver::writeAngle(uint8_t num, float32_t angle) {
//   int duty = EEPROMReadInt(ZERO_POSITIONS + target[0] * 2) +
//   float(servoCalib[target[0]]) * eeprom(ROTATION_DIRECTION, target[0]);
//   pwm.writeAngle(target[0], duty);
//   int clipped = min(max(EEPROMReadInt(ANGLE2PULSE_FACTOR + servoNum * 2) /
//   1000.0 * angle + (byte)eeprom(B_OFFSET) * 10, 500), 2500);
//   setPWM(eeprom(PWM_PIN, servoNum), 0, clipped);
// }