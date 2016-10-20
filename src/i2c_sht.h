#ifndef ZabbuinoI2C_SHT_h
#define ZabbuinoI2C_SHT_h

#include "i2c_bus.h"

#define SHT2X_I2C_ADDRESS                                       0x40
#define SHT2X_CMD_GETTEMP_HOLD                                  0xE3
#define SHT2X_CMD_GETHUMD_HOLD                                  0xE5

uint16_t getRawDataFromSHT2X(const uint8_t _i2cAddress, const uint8_t _command);
int8_t getSHT2XMetric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, const uint8_t _metric, char* _dst);

#endif