#ifndef __PI_EEPROM__
#define __PI_EEPROM__
#include <cstdint>

#include "i2c.hpp"

const uint8_t EEPROM_BUFFER_SIZE = 16;

class EEPROM {
public:
  EEPROM(const uint8_t& i2c_addr, I2C_Handler *i2c);
  uint16_t read_uint_16(const uint16_t& address);
private:
  I2C_Handler* i2c_dev_;
  uint8_t i2c_addr_;
  uint8_t buffer_[EEPROM_BUFFER_SIZE];
};

#endif