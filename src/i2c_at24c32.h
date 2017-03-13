#ifndef _ZABBUINO_I2C_AT24C32_H_
#define _ZABBUINO_I2C_AT24C32_H_

#include <time.h>


int8_t initAT24C32(SoftwareWire* _softTWI, const uint8_t _i2cAddress);
int8_t saveAT24C32TZOffset(SoftwareWire* _softTWI, const uint8_t _i2cAddress, int16_t _tzOffset);
int8_t readAT24C32TZOffset(SoftwareWire* _softTWI, const uint8_t _i2cAddress, int16_t* _tzOffset);

#endif // #ifndef _ZABBUINO_I2C_AT24C32_H_