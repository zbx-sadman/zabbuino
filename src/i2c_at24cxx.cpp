// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_at24cxx.h"

// No cell boundary control realized
int8_t AT24CXXWrite(SoftwareTWI* _softTWI, const uint8_t _i2cAddress, uint16_t _cellAddress, uint16_t _lenght, uint8_t* _src) {
  uint8_t rc = false, 
          value[0x03] = {0x00, 0x00, 0x00};  // first two byte - EEPROM address 

  while (_lenght) {
    // Send the write & next data byte address to AT24C
    value[0x00] = highByte(_cellAddress);
    value[0x01] = lowByte(_cellAddress);
    value[0x02] = *_src;
    if (sizeof(value) != writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, value, sizeof(value))) { goto finish; } 
    delay(5); // t write is 5ms max
//    DEBUG_PORT.print("Written: "); DEBUG_PORT.println(value[2], HEX);
    _cellAddress++;
    _src++;
    _lenght--;
  }

  rc = true; 
finish:
  return rc;
}

// No cell boundary control realized
int8_t AT24CXXRead(SoftwareTWI* _softTWI, const uint8_t _i2cAddress, const uint16_t _cellAddress, const uint16_t _lenght, uint8_t* _dst) {
  uint8_t rc = false, 
          value[0x02];
  // Send the start cell address to AT24C
  value[0x00] = highByte(_cellAddress);
  value[0x01] = lowByte(_cellAddress);
  if (sizeof(value) != writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, value, sizeof(value))) { goto finish; } 
  // Sequentally read _lenght bytes to _dst
  if (_lenght != readBytesFromI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, _dst, _lenght) ) { goto finish; } 
  rc = true;

finish:
  return rc;
}

