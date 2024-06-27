#ifndef __GENERIC_MOTOR_DRIVER__
#define __GENERIC_MOTOR_DRIVER__
#include <Adafruit_PWMServoDriver.h>

#include <cstdint>

class MotorDriver {
public:
  MotorDriver() {}
  virtual void setPWM(uint8_t num, uint16_t on, uint16_t off) = 0;
};

class Adafruit_PWMServoDriver_Wrapper : public MotorDriver {
public:
  Adafruit_PWMServoDriver_Wrapper(const uint8_t addr, I2C_Handler *i2c) : driver_(addr, i2c) {
    driver_.begin();
  }
  void setPWM(uint8_t num, uint16_t on, uint16_t off) {
    driver_.setPWM(num, on, off);
  }

private:
  Adafruit_PWMServoDriver driver_;
};

#endif