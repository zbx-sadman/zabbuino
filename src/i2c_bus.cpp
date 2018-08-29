// Config & common included files
#include "sys_includes.h"


#include "SoftwareWire/SoftwareWire.h"

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
*     - number of bytes written to I2C device
*     - 0 on any error detected on I2C bus
*
*****************************************************************************************************************************/
uint8_t writeBytesToI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t *_src, const uint8_t _len) 
{
  uint8_t writtenBytes = 0x00;

  // Do nothing if no lenght
  if (!_len) { goto finish; }
//  Serial.println(" -- 1 --");

  _softTWI->beginTransmission(_i2cAddress); // start transmission to device 
  // registerAddress is 0x00 and above ?
  if (I2C_NO_REG_SPECIFIED < _registerAddress) {
//  Serial.println(" -- 2 --");
     _softTWI->write((uint8_t) _registerAddress); // sends register address to be written
  }

  // Make bulk write to device
  writtenBytes = _softTWI->write(_src, _len);
//  Serial.print("writtenBytes: "); Serial.println(writtenBytes);

  // on any error return Zero as written bytes count
  if (SOFTWAREWIRE_NO_ERROR != _softTWI->endTransmission(true)) {
//     Serial.println(" -- 3 --");
     writtenBytes = 0x00;
  } 

//  Serial.println(" -- 4 --");

  finish:
  return writtenBytes;
}

/*****************************************************************************************************************************
*
*   Reads bytes from device's register (or not) over I2C.
*
*   Returns: 
*     - number of bytes written to I2C device
*     - 0 on any error detected on I2C bus
*
*****************************************************************************************************************************/
uint8_t readBytesFromI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t *_dst, const uint8_t _len)
{
  uint8_t recievedBytes = 0x00;
  
  if (!_len) { goto finish; }

  if (I2C_NO_REG_SPECIFIED < _registerAddress) {
     _softTWI->beginTransmission(_i2cAddress);   
     _softTWI->write((uint8_t) _registerAddress);
     // No Stop Condition, for repeated Talk
     _softTWI->endTransmission(false);     
  }

  // Make request with Stop Condition
  recievedBytes = _softTWI->requestFrom(_i2cAddress, _len, true);
  if (_len != recievedBytes) {
     recievedBytes = 0x00;
     goto finish;
  }

  if (recievedBytes == _len) {
    _softTWI->readBytes(_dst, _len);
  }

  finish:
  return recievedBytes;
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
  int32_t accResult = 0x00, tmpResult;
   

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
//    delayMicroseconds(constAdcStabilizationDelay);
    if (_len != readBytesFromI2C(_softTWI, _i2cAddress, _registerAddress, bytes, _len)) {
      goto finish;
    }
    if (dropResult) {
       // Just discard first readed data
       dropResult = false;
    } else {
       // make int32 from i2C's bytes
       tmpResult = 0x00;
       for (i = 0x00; i < _len; i++) {
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
int8_t writeValueToI2C(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, uint32_t _value, const uint8_t _len)
{
  uint8_t bytes[sizeof(uint32_t)];
  uint8_t i, nBytes;
  nBytes = constrain(_len, 1, sizeof(uint32_t));

  i = nBytes;
  while (i) {
    i--;
    // take last byte and shift bits right to make able get next byte
    bytes[i] = _value & 0xFF;
    _value = _value >> 8;
  }
  
  return ((nBytes == writeBytesToI2C(_softTWI, _i2cAddress, _registerAddress, bytes, nBytes)) ? RESULT_IS_OK : RESULT_IS_FAIL);
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

  //if (0x00 > _bitNumber || 7 < _bitNumber) { goto finish; }
  // _bitNumber is unsigned and can't be less than zero
  if (7 < _bitNumber) { goto finish; }

  if (sizeof(bytes) != readBytesFromI2C(_softTWI, _i2cAddress, _registerAddress, bytes, sizeof(bytes))) { goto finish; }

  bitWrite (bytes[0], _bitNumber, !!_value);

  // Write one byte to I2C
  rc = (sizeof(bytes) == writeBytesToI2C(_softTWI, _i2cAddress, _registerAddress, bytes, sizeof(bytes))) ? RESULT_IS_OK : RESULT_IS_FAIL;

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

  //if (0 > _bitNumber || 7 < _bitNumber) { goto finish; }
  // _bitNumber is unsigned and can't be less than zero
  if (7 < _bitNumber) { goto finish; }

  if (sizeof(bytes) != readBytesFromI2C(_softTWI, _i2cAddress, _registerAddress, bytes, sizeof(bytes))) { goto finish; }

  *_value = bitRead(bytes[0], _bitNumber);

  rc = RESULT_IS_UNSIGNED_VALUE;

  finish:
  return rc;

}

uint8_t isI2CDeviceReady(SoftwareWire* _softTWI, uint8_t _i2cAddress)
{
  _softTWI->beginTransmission(_i2cAddress);
  return (SOFTWAREWIRE_NO_ERROR == _softTWI->endTransmission(true));
}
