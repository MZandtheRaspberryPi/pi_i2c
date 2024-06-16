#ifndef __PI_I2C__
#define __PI_I2C__

#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <thread>
#include <sys/ioctl.h>
#include <string>
#include <unistd.h>

#include "i2c.hpp"

void delay(int64_t sleep_ms);

void log_msg(const std::string &msg);

uint64_t millis();

class I2CBusRaspberryPi : public I2C_Handler
{
public:
    I2CBusRaspberryPi(const std::string &i2c_interface_name);
    bool write(const uint8_t &device_address, const uint8_t *buffer, size_t len);
    bool write_then_read(const uint8_t &device_address, const uint8_t *write_buffer, size_t write_len,
                         uint8_t *read_buffer, size_t read_len);
    bool begin();
    void close();

    // functions for compatibility with MPU6050 library
    int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
    bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
    int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);

private:
    int16_t file_descriptor_;
    uint8_t sendBuf_[256];
    uint8_t recvBuf_[256];
};

#endif