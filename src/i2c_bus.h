#pragma once

#if defined(ARDUINO_ARCH_AVR)
#include "SoftwareWire/SoftwareWire.h"
typedef SoftwareWire SoftwareTWI;

#elif defined(ARDUINO_ARCH_ESP8266)  
#include <Wire.h>
typedef TwoWire SoftwareTWI;

#endif

#define I2C_NO_REG_SPECIFIED                                     (-0x01) //
#define I2C_NO_ADDR_SPECIFIED                                    (0x00) // General Call Address

#define SOFTWARETWI_NO_ERROR                                     (0x00)
#define SOFTWARETWI_BUFFER_FULL                                  (0x01)
#define SOFTWARETWI_ADDRESS_NACK                                 (0x02)
#define SOFTWARETWI_DATA_NACK                                    (0x03)
#define SOFTWARETWI_OTHER                                        (0x04)

//#define WireToU8(_src)     ((uint8_t) _src[0])
//#define WireToS8(_src)     ((int8_t)  _src[0])

//#define WireToU16(_src)  ((uint16_t) ( ((uint16_t) _src[0] << 8)| (uint16_t) _src[1]))
//#define WireToU16(_src)    ((uint16_t) ( ((uint16_t) _src[0] << 8)| (uint8_t) _src[1]))
//#define WireToS16(_src)    ((int16_t)  ( ((uint16_t) _src[0] << 8)| (uint8_t) _src[1]))

//#define WireToU16LE(_src)  ((uint16_t) ( ((uint16_t) _src[1] << 8)| (uint8_t) _src[0]))
//#define WireToS16LE(_src)  ((int16_t)  ( ((uint16_t) _src[1] << 8)| (uint8_t) _src[0]))

//#define WireToU24(_src)    ((uint32_t) ( ((uint32_t) _src[0] << 16) | ((uint16_t) _src[1] << 8) | (uint8_t) _src[2]))
//#define WireToS24(_src)    ((int32_t)  ( ((uint32_t) _src[0] << 16) | ((uint16_t) _src[1] << 8) | (uint8_t) _src[2]))


void U16ToWire(uint8_t*, uint16_t);
void U16LToWire(uint8_t*, uint16_t);

inline uint8_t WireToU8(uint8_t* _src)     { return ( (uint8_t)   _src[0x00] ); }
inline int8_t  WireToS8(uint8_t* _src)     { return ( (int8_t)    _src[0x00] ); }

inline uint16_t WireToU16(uint8_t* _src)   { return ( ((uint16_t) _src[0x00] << 8)  | _src[0x01] ); }
inline int16_t  WireToS16(uint8_t* _src)   { return ( ((int16_t)  _src[0x00] << 8)  | _src[0x01] ); }

inline uint16_t WireToU16LE(uint8_t* _src) { return ( ((uint16_t) _src[0x01] << 8)  | _src[0x00] ); }
inline int16_t  WireToS16LE(uint8_t* _src) { return ( ((int16_t)  _src[0x01] << 8)  | _src[0x00] ); }

inline uint32_t WireToU24(uint8_t* _src)   { return ( ((uint32_t) _src[0x00] << 16) | ((uint16_t) _src[0x01] << 8) | _src[0x02] ); }
inline int32_t  WireToS24(uint8_t* _src)   { return ( ((int32_t)  _src[0x00] << 16) | ((uint16_t) _src[0x01] << 8) | _src[0x02] ); }


/*****************************************************************************************************************************
*
*   Send one byte to writeBytesToI2C() subroutine
*
*   Returns: 
*     - writeBytesToI2C()'s result code
*
*****************************************************************************************************************************/
uint8_t writeByteToI2C(SoftwareTWI* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _src);

/*****************************************************************************************************************************
*
*   Write incoming bytes to I2C device register (if specified) or just to device
*
*   Returns: 
*     - number of bytes written to I2C device
*     - 0 on any error detected on I2C bus
*
*****************************************************************************************************************************/
uint8_t writeBytesToI2C(SoftwareTWI* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t *_src, const uint8_t _len); 

/*****************************************************************************************************************************
*
*   Reads bytes from device's register (or not) over I2C.
*
*   Returns: 
*     - number of bytes written to I2C device
*     - 0 on any error detected on I2C bus
*
*****************************************************************************************************************************/
uint8_t readBytesFromI2C(SoftwareTWI* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t *_dst, const uint8_t _len);

/*****************************************************************************************************************************
*
*   Reads numeric value from device's register (or not) over I2C.
*
*   Returns: 
*     - RESULT_IS_SIGNED_VALUE on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t readValueFromI2C(SoftwareTWI*, const uint8_t, const int16_t, uint32_t*, uint8_t, uint8_t _numberOfReadings = 0x00);

/*****************************************************************************************************************************
*
*   Write numeric value to device's register (or not) over I2C.
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t writeValueToI2C(SoftwareTWI*, const uint8_t, const int16_t, uint32_t, const uint8_t);

/*****************************************************************************************************************************
*
*   Write bit value (set bit) of byte in device's register (or not).
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t bitWriteToI2C(SoftwareTWI*, const uint8_t, const int16_t, const uint8_t, const uint8_t);

/*****************************************************************************************************************************
*
*   Read bit value (get bit) of byte in device's register (or not).
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t bitReadFromI2C(SoftwareTWI*, const uint8_t, const int16_t, const uint8_t, uint8_t*);

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
uint8_t isI2CDeviceReady(SoftwareTWI* _softTWI, uint8_t _i2cAddress);
                              
