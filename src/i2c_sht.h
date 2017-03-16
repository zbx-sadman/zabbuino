#ifndef _ZABBUINO_I2C_SHT_H_
#define _ZABBUINO_I2C_SHT_H_

#include "i2c_bus.h"

#define SHT2X_I2C_ADDRESS                                       0x40
#define SHT2X_CMD_GETTEMP_HOLD                                  0xE3
#define SHT2X_CMD_GETHUMD_HOLD                                  0xE5

/*****************************************************************************************************************************
*
*   Get raw data from the SHT2X sensor 
*
*   Returns: 
*     - 0 on error
*     - 16-bit raw data on success
*
*****************************************************************************************************************************/
uint16_t getRawDataFromSHT2X(SoftwareWire*, const uint8_t, const uint8_t);

/*****************************************************************************************************************************
*
*   Read specified metric's value of the SHT2X sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if sensor do not ready to work
*
*****************************************************************************************************************************/
int8_t getSHT2XMetric(SoftwareWire*, uint8_t, const uint8_t, uint32_t*);
int8_t getSHT2XMetric(SoftwareWire*, uint8_t, const uint8_t, char*);

int8_t getSHT2XMetric(SoftwareWire*, uint8_t, const uint8_t, char*, uint32_t*, const uint8_t _wantsNumber = false);

#endif // #ifndef _ZABBUINO_I2C_SHT_H_