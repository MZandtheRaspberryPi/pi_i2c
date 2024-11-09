#ifndef __GENERIC_MOTOR_DRIVER__
#define __GENERIC_MOTOR_DRIVER__
#include <Adafruit_PWMServoDriver.h>

#include <cstdint>

class MotorDriver {
public:
  MotorDriver() {}
  virtual void setPWM(uint8_t num, uint16_t on, uint16_t off) = 0;
  virtual uint16_t getPWM(uint8_t num, bool off = true) = 0;
  virtual void setPwmFreq(float32_t freq) = 0;
  virtual float32_t readPwmFreq() = 0;
};

class Adafruit_PWMServoDriver_Wrapper : public MotorDriver {
public:
  Adafruit_PWMServoDriver_Wrapper(const uint8_t addr, I2C_Handler *i2c) : driver_(addr, i2c) {
    driver_.begin();
  }
  void setPWM(uint8_t num, uint16_t on, uint16_t off) {
    driver_.setPWM(num, on, off);
  }
  // If true, returns PWM OFF value, otherwise PWM ON
  uint16_t getPWM(uint8_t num, bool off = true)
  {
    return driver_.getPWM(num, off);
  }
  void setPwmFreq(float32_t freq) {
    driver_.setPWMFreq(freq);
  }

  float32_t readPwmFreq() {
    uint8_t prescale = driver_.readPrescale();
    uint32_t oscillator_frequency = driver_.getOscillatorFrequency();
    // prescaler is calculated for pca9685 as per datasheet:
    // float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
    // so we solve for freq
    // pwm_freq = (oscillator_freq / (prescaleval + 1 - 0.5) )/ 4096
    float32_t pwm_freq = (float32_t)oscillator_frequency / (4096.0 * ((float32_t)prescale + 1 - 0.5));
    return pwm_freq;
  }

private:
  Adafruit_PWMServoDriver driver_;
};

#endif