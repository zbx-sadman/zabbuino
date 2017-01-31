#include "i2c_sht.h"

/*****************************************************************************************************************************
*
*   Get raw data from the SHT2X sensor 
*
*   Returns: 
*     - 0 on error
*     - 16-bit raw data on success
*
*****************************************************************************************************************************/
uint16_t getRawDataFromSHT2X(const uint8_t _i2cAddress, const uint8_t _command)
{
    uint16_t result;

    Wire.beginTransmission(_i2cAddress);
    Wire.write(_command);
    delay(100);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)_i2cAddress, (uint8_t) 3);
    // Don't hang here for more than 300ms
    uint32_t readTimeOut = millis() + 300UL;
    //uint32_t timeout = millis() + 300;
    while (Wire.available() < 3) {
      if (millis() > readTimeOut) { return 0; }
    }
    
    //Store the result
    result = Wire.read() << 8;
    result += Wire.read();

    result &= ~0x0003;   // clear two low bits (status bits)
    //Clear the final byte from the buffer
    Wire.read();
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
int8_t getSHT2XMetric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, const uint8_t _metric, char *_dst) 
{
  int32_t result = 0;
  uint16_t rawData; 
  
  switch (_i2cAddress) {
    case SHT2X_I2C_ADDRESS:
      break;
    default:  
       _i2cAddress = SHT2X_I2C_ADDRESS;
  }

  if (!isI2CDeviceReady(_i2cAddress)) { return DEVICE_ERROR_CONNECT; }
  
  switch (_metric) {
    case SENS_READ_TEMP:
      rawData = getRawDataFromSHT2X(_i2cAddress, SHT2X_CMD_GETTEMP_HOLD);
      // Default humidity  resolution - 14 bit => 0.01C
      // all part of equation must be multiplied by 100 to get 2-digit fract part by ltoaf() 
      // H_real = 100 * -6 + (100 * 125 * H_raw) / (2^16)
      result = (((uint32_t) rawData * 17572) >> 16) - 4685;
      break;
      
    case SENS_READ_HUMD:
      rawData = getRawDataFromSHT2X(_i2cAddress, SHT2X_CMD_GETHUMD_HOLD);
      // Default humidity  resolution - 12 bit => 0.04%RH
      // all part of equation must be multiplied by 100 to get 2-digit fract part by ltoaf() 
      // H_real = 100 * -6 + (100 * 125 * H_raw) / (2^16)
      if (0 < rawData) { result = (((uint32_t) rawData * 100 * 125) >> 16) - 600; }
  }
  // result /=100
  ltoaf(result, _dst, 2);
  gatherSystemMetrics(); // Measure memory consumption
  return RESULT_IN_BUFFER;  

}


