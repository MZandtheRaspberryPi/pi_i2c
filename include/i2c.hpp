#ifndef __GENERIC_I2C__
#define __GENERIC_I2C__

#include <cstdint>
#include <string>

class I2C_Handler {
public:
  I2C_Handler(const std::string &i2c_interface_name)
      : i2c_interface_name_(i2c_interface_name){};
  virtual bool write(const uint8_t &device_address, const uint8_t *buffer,
                     size_t len) = 0;
  virtual bool write_then_read(const uint8_t &device_address,
                               const uint8_t *write_buffer, size_t write_len,
                               uint8_t *read_buffer, size_t read_len) = 0;
  virtual bool begin() = 0;
  virtual void close() = 0;

  // functions for compatibility with MPU6050 library
  virtual int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                         uint8_t *data) = 0;
  virtual int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                          uint8_t length, uint8_t *data) = 0;
  virtual bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                         uint8_t length, uint8_t data) = 0;
  virtual bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                        uint8_t data) = 0;
  virtual bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) = 0;
  virtual int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) = 0;
  virtual int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) = 0;

protected:
  std::string i2c_interface_name_;
};

#endif
