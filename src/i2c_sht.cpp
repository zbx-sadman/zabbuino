#include "i2c_sht.h"

static uint16_t getRawDataFromSHT2X(SoftwareWire*, const uint8_t, const uint8_t);

int8_t getSHT2XMetric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint8_t _metric, uint32_t* _value)
{
  char stubBuffer;
  return getSHT2XMetric(_softTWI, _i2cAddress, _metric, &stubBuffer, _value, true);

}

int8_t getSHT2XMetric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint8_t _metric, char* _dst)
{
  uint32_t stubValue;
  return getSHT2XMetric(_softTWI, _i2cAddress, _metric, _dst, &stubValue, false);
}

/*****************************************************************************************************************************
*
*   Get raw data from the SHT2X sensor 
*
*   Returns: 
*     - 0 on error
*     - 16-bit raw data on success
*
*****************************************************************************************************************************/
uint16_t getRawDataFromSHT2X(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _command)
{
    uint16_t result;

    _softTWI->beginTransmission(_i2cAddress);
    _softTWI->write(_command);
    delay(100);
    _softTWI->endTransmission();

    _softTWI->requestFrom((uint8_t)_i2cAddress, (uint8_t) 3);
    // Don't hang here for more than 300ms
    uint32_t readTimeOut = millis() + 300UL;
    //uint32_t timeout = millis() + 300;
    while (_softTWI->available() < 3) {
      if (millis() > readTimeOut) { return 0; }
    }
    
    //Store the result
    result = _softTWI->read() << 8;
    result += _softTWI->read();

    result &= ~0x0003;   // clear two low bits (status bits)
    //Clear the final byte from the buffer
    _softTWI->read();
    return result;
}

/*****************************************************************************************************************************
*
*   Read specified metric's value of the SHT2X sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if sensor do not ready to work
*
*****************************************************************************************************************************/
int8_t getSHT2XMetric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint8_t _metric, char *_dst, uint32_t* _value, const uint8_t _wantsNumber) 
{
//  int32_t result = 0;
  uint16_t rawData; 
  int8_t rc = RESULT_IS_FAIL;  
  
  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { return DEVICE_ERROR_CONNECT; }

  switch (_metric) {
    case SENS_READ_TEMP:
      rawData = getRawDataFromSHT2X(_softTWI, _i2cAddress, SHT2X_CMD_GETTEMP_HOLD);
      // Default humidity  resolution - 14 bit => 0.01C
      // all part of equation must be multiplied by 100 to get 2-digit fract part by ltoaf() 
      // H_real = 100 * -6 + (100 * 125 * H_raw) / (2^16)
      *_value = (((uint32_t) rawData * 17572) >> 16) - 4685;
      rc = RESULT_IN_BUFFER;
      break;
      
    case SENS_READ_HUMD:
      rawData = getRawDataFromSHT2X(_softTWI, _i2cAddress, SHT2X_CMD_GETHUMD_HOLD);
      // Default humidity  resolution - 12 bit => 0.04%RH
      // all part of equation must be multiplied by 100 to get 2-digit fract part by ltoaf() 
      // H_real = 100 * -6 + (100 * 125 * H_raw) / (2^16)
      if (0 < rawData) { 
         *_value = (((uint32_t) rawData * 100 * 125) >> 16) - 600; 
         rc = RESULT_IN_BUFFER;
      }
      else {
         rc = DEVICE_ERROR_WRONG_ANSWER; 
      } 
      break;
  }
  // result /=100
  if (!_wantsNumber) {
     ltoaf(*_value, _dst, 2);
  }

  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}


