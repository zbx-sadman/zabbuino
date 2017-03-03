#ifndef _ZABBUINO_I2C_BUS_H_
#define _ZABBUINO_I2C_BUS_H_

// Wire lib for I2C sensors
#include <Wire.h>
#include "../basic.h"
#include "tune.h"
#include "service.h"
#include "system.h"
#include "network.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                      COMMON I2C SECTION

 -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

#define WireToU8(_source)  ((uint8_t) _source[0])
#define WireToS8(_source)  ((int8_t) _source[0])

#define WireToU16(_source)  ((uint16_t) ( ((uint16_t) _source[0] << 8)| (uint16_t) _source[1]))
#define WireToS16(_source)  ((int16_t) ( ((uint16_t) _source[0] << 8)| (uint16_t) _source[1]))

#define WireToU16LE(_source)  ((uint16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))
#define WireToS16LE(_source)  ((int16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))

#define WireToU24(_source)  ((uint32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))
#define WireToS24(_source)  ((int32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))

/*****************************************************************************************************************************
*
*   Scan I2C bus and print to ethernet client addresses of all detected devices 
*
*   Returns: 
*     - RESULT_IS_PRINTED on success
*     - RESULT_IS_FAIL of no devices found 
*
*****************************************************************************************************************************/
int8_t scanI2C(NetworkClass*);

/*****************************************************************************************************************************
*
*   Send one byte to writeBytesToI2C() subroutine
*
*   Returns: 
*     - writeBytesToI2C()'s result code
*
*****************************************************************************************************************************/
uint8_t writeByteToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _src);

/*****************************************************************************************************************************
*
*   Write incoming bytes to I2C device register (if specified) or just to device
*
*   Returns: 
*     - Wire.endTransmission result code
*       0 - success
*       1 - data too long to fit in transmit buffer
*       2 - received NACK on transmit of address
*       3 - received NACK on transmit of data
*       4 - other error
*
*****************************************************************************************************************************/
uint8_t writeBytesToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t *_src, uint8_t _len);

/*****************************************************************************************************************************
*
*   Reads bytes from device's register (or not) over I2C.
*
*   Returns: 
*     - Wire.endTransmission result code
*       0 - success
*       1 - data too long to fit in transmit buffer
*       2 - received NACK on transmit of address
*       3 - received NACK on transmit of data
*       4 - other error
*
*
*****************************************************************************************************************************/
uint8_t readBytesFromi2C(const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t *_dst, const uint8_t _len);

/*****************************************************************************************************************************
*
*   Ping I2C slave device
*
*   Returns: 
*     - Wire.endTransmission result code
*       0 - success
*       1 - data too long to fit in transmit buffer
*       2 - received NACK on transmit of address
*       3 - received NACK on transmit of data
*       4 - other error
*
*
*****************************************************************************************************************************/
uint8_t inline isI2CDeviceReady(uint8_t _i2cAddress)
{
  //
  Wire.beginTransmission(_i2cAddress);
  return (0 == Wire.endTransmission(true));
}

#endif // #ifndef _ZABBUINO_I2C_BUS_H_