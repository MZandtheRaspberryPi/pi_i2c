#include "eeprom.h"

EEPROM::EEPROM(const uint8_t& i2c_addr, I2C_Handler *i2c)
{
    i2c_dev_ = i2c;
    i2c_addr_ = i2c_addr;
}

uint16_t EEPROM::read_uint_16(const uint16_t& address)
{
    i2c_dev_->readBytesExtendedReg(i2c_addr_, address, 2, buffer_);
    uint16_t res = ((buffer_[0] << 0) & 0xFF) + ((buffer_[1] << 8) & 0xFF00);
    return res;
}