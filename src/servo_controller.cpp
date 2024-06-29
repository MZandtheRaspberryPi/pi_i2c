#include "servo_controller.hpp"

bool ServoBoardConfig::is_servo_num_valid(const uint8_t &servo_num) {
  return servo_num < num_servos_;
}

ServoBoardConfig::ServoBoardConfig(uint8_t num_servos,
                                   float32_t default_lower_angle_limit,
                                   float32_t default_upper_angle_limit,
                                   float32_t default_zero_position,
                                   bool default_invert_servo_position,
                                   float32_t min_angle_to_command,
                                   float32_t max_angle_to_command,
                                   uint16_t min_pulsewidth_to_command,
                                   uint16_t max_pulsewidth_to_command) {
  num_servos_ = num_servos;

  min_angle_to_command_ = min_angle_to_command;
  max_angle_to_command_ = max_angle_to_command;
  min_pulsewidth_to_command_ = min_pulsewidth_to_command;
  max_pulsewidth_to_command_ = max_pulsewidth_to_command;
  angle_to_pulsewidth_slope_ = 1.0 * (max_pulsewidth_to_command_ - min_pulsewidth_to_command_) / (max_angle_to_command_ - min_angle_to_command_);

  min_angles_ = std::shared_ptr<float32_t>(new float32_t[num_servos]);
  max_angles_ = std::shared_ptr<float32_t>(new float32_t[num_servos]);
  zero_positions_ = std::shared_ptr<float32_t>(new float32_t[num_servos]);
  invert_servo_positions_ = std::shared_ptr<bool>(new bool[num_servos]);
  for (uintmax_t i = 0; i < num_servos_; i++) {
    min_angles_.get()[i] = default_lower_angle_limit;
    max_angles_.get()[i] = default_upper_angle_limit;
    zero_positions_.get()[i] = default_zero_position;
    invert_servo_positions_.get()[i] = default_invert_servo_position;
  }
  cur_angle_helper_ = 0;
}

void ServoBoardConfig::servo_angle_to_pulsewidth(const float32_t& angle, uint16_t& pwm_out)
{
  pwm_out = min_pulsewidth_to_command_ + round(angle_to_pulsewidth_slope_ * (angle - min_angle_to_command_));
}

bool ServoBoardConfig::set_min_angle(const uint8_t &servo_num,
                                     const float32_t &lower_angle_limit) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  min_angles_.get()[servo_num] = lower_angle_limit;
  return true;
}

bool ServoBoardConfig::set_max_angle(const uint8_t &servo_num,
                                     const float32_t &upper_angle_limit) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  max_angles_.get()[servo_num] = upper_angle_limit;
  return true;
}

bool ServoBoardConfig::get_min_angle(const uint8_t &servo_num,
                                     float32_t &min_angle) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  min_angle = min_angles_.get()[servo_num];
  return true;
}

bool ServoBoardConfig::get_max_angle(const uint8_t &servo_num,
                                     float32_t &max_angle) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  max_angle = max_angles_.get()[servo_num];
  return true;
}

bool ServoBoardConfig::get_zero_position(const uint8_t &servo_num,
                                         float32_t &zero_pos) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }

  zero_pos = zero_positions_.get()[servo_num];

  return true;
}
bool ServoBoardConfig::set_zero_position(const uint8_t &servo_num,
                                         const float32_t &zero_pos) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  zero_positions_.get()[servo_num] = zero_pos;
  return true;
}

bool ServoBoardConfig::get_invert_servo_flag(const uint8_t &servo_num,
                                             bool &invert_servo_pos) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  invert_servo_pos = invert_servo_positions_.get()[servo_num];
  return true;
}

bool ServoBoardConfig::set_invert_servo_flag(const uint8_t &servo_num,
                                             const bool &invert_servo_pos) {
  if (!is_servo_num_valid(servo_num)) {
    return false;
  }
  invert_servo_positions_.get()[servo_num] = invert_servo_pos;
  return true;
}

ServoController::ServoController(ServoBoardConfig *servo_config,
                                 MotorDriver *motor_driver) {
  servo_config_ = servo_config;
  motor_driver_ = motor_driver;
  cur_servo_zero_pos_ = 0;
  is_cur_servo_inverted_ = false;
  cur_angle_helper_ = 0;
  cur_pwm_ = 0;
}

bool ServoController::set_servo_angle(const uint8_t &servo_num,
                                      const float32_t &servo_angle) {
  if (!servo_config_->is_servo_num_valid(servo_num)) {
    return false;
  }
  servo_config_->get_min_angle(servo_num, cur_angle_helper_);
  if (servo_angle < cur_angle_helper_) {
    return false;
  }
  servo_config_->get_max_angle(servo_num, cur_angle_helper_);
  if (servo_angle > cur_angle_helper_) {
    return false;
  }

  servo_config_->get_zero_position(servo_num, cur_servo_zero_pos_);
  servo_config_->get_invert_servo_flag(servo_num, is_cur_servo_inverted_);
  cur_angle_helper_ = (servo_angle + cur_servo_zero_pos_);
  if (is_cur_servo_inverted_) {
    cur_angle_helper_ *= -1;
  }

  servo_config_->servo_angle_to_pulsewidth(cur_angle_helper_, cur_pwm_);
  motor_driver_->setPWM(servo_num, 0, cur_pwm_);

  //   int clipped = min(max(EEPROMReadInt(ANGLE2PULSE_FACTOR + servoNum * 2) /
  //   1000.0 * angle + (byte)eeprom(B_OFFSET) * 10, 500), 2500);
  //   setPWM(eeprom(PWM_PIN, servoNum), 0, clipped);

  return true;
}

void ServoController::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
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