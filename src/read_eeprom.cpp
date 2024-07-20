#include <iostream>
#include <string>

#include "i2c_linux.hpp"
#include "eeprom.h"

int main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " device_addr register_addr" << std::endl;
    std::cout << "Example: " << argv[0] << " 84 275" << std::endl;
    return 1;
  }

  std::string i2c_name = "/dev/i2c-1";
  I2CLinuxAPI i2c_dev(i2c_name);
  i2c_dev.begin();

  const uint8_t eeprom_addr = std::stoul(argv[1]);
  const uint16_t register_addr = std::stoul(argv[2]);
  std::cout << "reading from eeprom at: " << static_cast<uint16_T>(eeprom_addr) << " and register: " << register_addr << std::endl;


  EEPROM eeprom(eeprom_addr, &i2c_dev);
  uint16_t reg_val = eeprom.read_uint_16(register_addr);
  i2c_dev.close();

  std::cout << "register_value: " << reg_val << std::endl;
  return 0;
}