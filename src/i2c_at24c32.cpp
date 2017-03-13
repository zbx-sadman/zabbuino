#include "i2c_bus.h"
#include "i2c_at24c32.h"


int8_t initAT24C32(SoftwareWire* _softTWI, const uint8_t _i2cAddress) { 
  return isI2CDeviceReady(_softTWI, _i2cAddress);
}

int8_t saveAT24C32TZOffset(SoftwareWire* _softTWI, const uint8_t _i2cAddress, int16_t _tzOffset) {
  uint8_t rc = false, 
          value[3] = {0, 0, 0};  // first two byte - EEPROM address 
  // write first byte to addr: 0000
  value[2] = _tzOffset >> 8;
  if (0x00 != writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, value, sizeof(value))) { goto finish; } 

  // write second byte to addr: 0001
  value[1]++;
  value[2] = _tzOffset & 0x00FF;
  if (0x00 != writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, value, sizeof(value))) { goto finish; } 

  rc = true; 
  finish:
  return rc;
}

int8_t readAT24C32TZOffset(SoftwareWire* _softTWI, const uint8_t _i2cAddress, int16_t* _tzOffset) {
  uint8_t rc = false, 
          value[2] = {0, 0}; // first two byte - EEPROM address 

  // write to controller address from which will be readed data
  if (0x00 != writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, value, sizeof(value))) { goto finish; } 

  // read two byte (uint16_t)
  if (0x00 != readBytesFromi2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, value, 2) ) { goto finish; } 

  *_tzOffset= (value[0] << 8) | value[1];
  rc = true;

  finish:
  return rc;
}

