// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"

/*****************************************************************************************************************************
*
*   Send one byte to writeBytesToI2C() subroutine
*
*   Returns: 
*     - writeBytesToI2C()'s result code
*
*****************************************************************************************************************************/
//#define writeByteToI2C((_i2cAddress), (_registerAddress), (_data)) 
uint8_t writeByteToI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _src)
{
  return writeBytesToI2C(_softTWI, _i2cAddress, _registerAddress, &_src, 1);
}


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
uint8_t writeBytesToI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t *_src, uint8_t _len) 
{

  _softTWI->beginTransmission(_i2cAddress); // start transmission to device 
  // registerAddress is 0x00 and above ?
  if (I2C_NO_REG_SPECIFIED < _registerAddress) {
     _softTWI->write((uint8_t) _registerAddress); // sends register address to be written
  }
  while (_len) {
    _softTWI->write(*_src);  // write data
    _src++;
    _len--;
  }
  return _softTWI->endTransmission(true); // end transmission

}

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
*****************************************************************************************************************************/
uint8_t readBytesFromI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t *_dst, uint8_t _len)
{
  if (!_len) { return false; }

  _softTWI->beginTransmission(_i2cAddress);   
  if (I2C_NO_REG_SPECIFIED < _registerAddress) {
     _softTWI->write((uint8_t) _registerAddress);
     // No Stop Condition, for repeated Talk
     _softTWI->endTransmission(false);     
  }

  uint32_t lastTimeCheck = millis();         
  _softTWI->requestFrom(_i2cAddress, _len);  
  while(_len && (_softTWI->available() > 0)) {
    // 100 => 0.1 sec timeout
    // SoftwareWire have protection timeout for 1sec, but error sign is not returned :(
    if ((millis() - lastTimeCheck) > 300UL) {
       _softTWI->endTransmission(true);
       return false;
    }
    *_dst = (uint8_t) _softTWI->read();
    _dst++;
    _len--;
  }
  // Stop Condition
  return _softTWI->endTransmission(true);    
}

/*****************************************************************************************************************************
*
*   Reads numeric value from device's register (or not) over I2C.
*
*   Returns: 
*     - RESULT_IS_SIGNED_VALUE on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t readValueFromI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, uint32_t* _value, uint8_t _len, uint8_t _numberOfReadings)
{
  int8_t rc = RESULT_IS_FAIL;
  uint8_t i, dropResult, readNumber, bytes[4];
  int32_t accResult, tmpResult;
   

  _len = constrain(_len, 1, 4);
  // If numberOfReadings is 0 (not specified in Zabbuino command) - do once reading only 
  // Otherwise make several reading for re-run sensor conversion and drop first result to "flush" old data.

  if (0x00 != _numberOfReadings) {
     readNumber = _numberOfReadings + 1;
     dropResult = true;
  } else {
     readNumber = _numberOfReadings = 1;
     dropResult = false;
  }

  while (readNumber) {
    readNumber--;
    delayMicroseconds(constAdcStabilizationDelay);
    if (0 != readBytesFromI2C(_softTWI, _i2cAddress, _registerAddress, bytes, _len)) {
      goto finish;
    }
    if (dropResult) {
       // Just discard first readed data
       dropResult = false;
    } else {
       // make int32 from i2C's bytes
       tmpResult = 0;
       for (i = 0; i < _len; i++) {
         tmpResult <<= 8;
         tmpResult |= bytes[i];
       }
       accResult += tmpResult;
    }
  }

  *_value = (accResult / _numberOfReadings);
  rc = RESULT_IS_SIGNED_VALUE;

  finish:
  return rc;
 
}

/*****************************************************************************************************************************
*
*   Write numeric value to device's register (or not) over I2C.
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t writeValueToI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, uint32_t _value, uint8_t _len)
{
  uint8_t i, bytes[4];

  _len = constrain(_len, 1, 4);
  i = _len;
  while (i) {
    i--;
    // take last byte and shift bits right to make able get next byte
    bytes[i] = _value & 0xFF;
    _value = _value >> 8;
  }
  return (0x00 == writeBytesToI2C(_softTWI, _i2cAddress, _registerAddress, bytes, _len)) ? RESULT_IS_OK : RESULT_IS_FAIL;
}

/*****************************************************************************************************************************
*
*   Write bit value (set bit) of byte in device's register (or not).
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t bitWriteToI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _bitNumber, const uint8_t _value)
{
  int8_t rc = RESULT_IS_FAIL;
  uint8_t bytes[1];

  if (0 > _bitNumber || 7 < _bitNumber) { goto finish; }

  if (0x00 != readBytesFromI2C(_softTWI, _i2cAddress, _registerAddress, bytes, 1)) { goto finish; }

  // "!!" convert value 0100 to 1.
  bitWrite (bytes[0], _bitNumber, !!_value);

  // Write one byte to I2C
  rc = (0x00 == writeBytesToI2C(_softTWI, _i2cAddress, _registerAddress, bytes, 1)) ? RESULT_IS_OK : RESULT_IS_FAIL;

  finish:
  return rc;

}

/*****************************************************************************************************************************
*
*   Read bit value (get bit) of byte in device's register (or not).
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on fail
*
*****************************************************************************************************************************/
int8_t bitReadFromI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _bitNumber, uint8_t* _value)
{
  int8_t rc = RESULT_IS_FAIL;
  uint8_t bytes[1];

  if (0 > _bitNumber || 7 < _bitNumber) { goto finish; }

  if (0x00 != readBytesFromI2C(_softTWI, _i2cAddress, _registerAddress, bytes, 1)) { goto finish; }

  *_value = bitRead(bytes[0], _bitNumber);

  rc = RESULT_IS_UNSIGNED_VALUE;

  finish:
  return rc;

}

uint8_t isI2CDeviceReady(SoftwareWire* _softTWI, uint8_t _i2cAddress)
{
  _softTWI->beginTransmission(_i2cAddress);
  return (0 == _softTWI->endTransmission(true));
}
