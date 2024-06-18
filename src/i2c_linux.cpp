#include "i2c_linux.hpp"

I2CLinuxAPI::I2CLinuxAPI(const std::string &i2c_interface_name)
    : I2C_Handler(i2c_interface_name) {}

bool I2CLinuxAPI::write(const uint8_t &device_address,
                              const uint8_t *buffer, size_t len) {
  i2c_msg messages[1] = {{device_address, 0, (typeof(i2c_msg().len))len,
                          (typeof(i2c_msg().buf))buffer}};
  i2c_rdwr_ioctl_data ioctl_data = {messages, 1};

  int result = ioctl(file_descriptor_, I2C_RDWR, &ioctl_data);

  if (result != 1) {
    log_msg("Failed to write to " + std::to_string(device_address));
    return false;
  }
  return true;
}
bool I2CLinuxAPI::write_then_read(const uint8_t &device_address,
                                        const uint8_t *write_buffer,
                                        size_t write_len, uint8_t *read_buffer,
                                        size_t read_len) {
  i2c_msg messages[2] = {
      {device_address, 0, (typeof(i2c_msg().len))write_len,
       (typeof(i2c_msg().buf))write_buffer},
      {device_address, I2C_M_RD, (typeof(i2c_msg().len))read_len,
       (typeof(i2c_msg().buf))read_buffer},
  };
  i2c_rdwr_ioctl_data ioctl_data = {messages, 2};

  int result = ioctl(file_descriptor_, I2C_RDWR, &ioctl_data);

  if (result != 2) {
    log_msg("Failed to write then read to " + std::to_string(device_address));
    return false;
  }
  return true;
}

bool I2CLinuxAPI::begin() {
  close();
  file_descriptor_ = ::open(i2c_interface_name_.c_str(), O_RDWR);
  if (file_descriptor_ == -1) {
    log_msg("Failed to open I2C device " + i2c_interface_name_);
    return false;
  }
  return true;
}

void I2CLinuxAPI::close() {
  if (file_descriptor_ != -1) {
    ::close(file_descriptor_);
    file_descriptor_ = -1;
  }
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
int8_t I2CLinuxAPI::readBit(uint8_t devAddr, uint8_t regAddr,
                                  uint8_t bitNum, uint8_t *data) {
  sendBuf_[0] = regAddr;
  bool success = write_then_read(devAddr, sendBuf_, 1, recvBuf_, 1);
  if (success) {
    *data = recvBuf_[1] & (1 << bitNum);
    return 1;
  } else {
    return 0;
  }
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2CLinuxAPI::writeBits(uint8_t devAddr, uint8_t regAddr,
                                  uint8_t bitStart, uint8_t length,
                                  uint8_t data) {
  sendBuf_[0] = regAddr;
  bool success = write_then_read(devAddr, sendBuf_, 1, recvBuf_, 1);
  if (success) {
    uint8_t b = recvBuf_[0];
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask;                     // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data;    // combine data with existing byte
    sendBuf_[1] = b;
    success &= write(devAddr, sendBuf_, 2);
  } 
  return success;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2CLinuxAPI::writeBit(uint8_t devAddr, uint8_t regAddr,
                                 uint8_t bitNum, uint8_t data) {
  sendBuf_[0] = regAddr;
  bool success = write_then_read(devAddr, sendBuf_, 1, recvBuf_, 1);
  if (success) {
    uint8_t b = recvBuf_[0] ;
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    sendBuf_[1] = b ;
    success &= write(devAddr, sendBuf_, 2);
  } 
  return success;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (true = success)
 */
int8_t I2CLinuxAPI::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
  // 01101001 read byte
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  //    010   masked
  //   -> 010 shifted
  sendBuf_[0] = regAddr;
  bool success = write_then_read(devAddr, sendBuf_, 1, recvBuf_, 1);
  if (success) {
    uint8_t b = recvBuf_[0] ;
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
  } 
  return success;
}


int8_t I2CLinuxAPI::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
  sendBuf_[0] = regAddr;
  bool success = write_then_read(devAddr, sendBuf_, 1, recvBuf_, length);
  int i ;
  for (i = 0; i < length ; i++) {
    data[i] = (uint8_t) recvBuf_[i];
  }
  return success;
}

int8_t I2CLinuxAPI::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
  return readBytes(devAddr, regAddr, 1, data);
}


bool I2CLinuxAPI::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBits(devAddr, regAddr, 0, 8, data);
}