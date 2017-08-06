#ifndef _ZABBUINO_I2C_BUS_H_
#define _ZABBUINO_I2C_BUS_H_

#define I2C_NO_REG_SPECIFIED                                    (-0x01) //

#define WireToU8(_source)  ((uint8_t) _source[0])
#define WireToS8(_source)  ((int8_t) _source[0])

//#define WireToU16(_source)  ((uint16_t) ( ((uint16_t) _source[0] << 8)| (uint16_t) _source[1]))
#define WireToU16(_source)  ((uint16_t) ( ((uint8_t) _source[0] << 8)| (uint8_t) _source[1]))
#define WireToS16(_source)  ((int16_t)  ( ((uint8_t) _source[0] << 8)| (uint8_t) _source[1]))

#define WireToU16LE(_source)  ((uint16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))
#define WireToS16LE(_source)  ((int16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))

#define WireToU24(_source)  ((uint32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))
#define WireToS24(_source)  ((int32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))

/*****************************************************************************************************************************
*
*   Send one byte to writeBytesToI2C() subroutine
*
*   Returns: 
*     - writeBytesToI2C()'s result code
*
*****************************************************************************************************************************/
uint8_t writeByteToI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _src);

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
uint8_t writeBytesToI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t *_src, uint8_t _len);

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
uint8_t readBytesFromI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t *_dst, const uint8_t _len);

/*****************************************************************************************************************************
*
*   Reads numeric value from device's register (or not) over I2C.
*
*   Returns: 
*     - RESULT_IS_SIGNED_VALUE on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t readValueFromI2C(SoftwareWire*, const uint8_t, const int16_t, uint32_t*, uint8_t, uint8_t _numberOfReadings = 0x00);

/*****************************************************************************************************************************
*
*   Write numeric value to device's register (or not) over I2C.
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t writeValueToI2C(SoftwareWire*, const uint8_t, const int16_t, uint32_t, uint8_t);

/*****************************************************************************************************************************
*
*   Write bit value (set bit) of byte in device's register (or not).
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t bitWriteToI2C(SoftwareWire*, const uint8_t, const int16_t, const uint8_t, const uint8_t);

/*****************************************************************************************************************************
*
*   Read bit value (get bit) of byte in device's register (or not).
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t bitReadFromI2C(SoftwareWire*, const uint8_t, const int16_t, const uint8_t, uint8_t*);

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
uint8_t isI2CDeviceReady(SoftwareWire* _softTWI, uint8_t _i2cAddress);

#endif // _ZABBUINO_I2C_BUS_H_