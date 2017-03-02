#include "i2c_bus.h"
#include "i2c_bh1750.h"


/*****************************************************************************************************************************
*
*   Read specified metric's value of the BH1750 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t getBH1750Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _metric, char *_dst)
{
  uint8_t readingNum;
  int16_t result, convertTime;

  if (!isI2CDeviceReady(_i2cAddress)) {return DEVICE_ERROR_CONNECT; }


  switch (_mode) {
    case BH1750_ONETIME_HIGHRES: 
    case BH1750_ONETIME_HIGHRES_2:
      readingNum = 3;
      convertTime = 180; // 180ms - max time to complete High-resolution measurement
      break;
    case BH1750_ONETIME_LOWRES:
       readingNum = 3;
       convertTime = 24; // 24ms - max time to complete Low-resolution measurement
       break;
    case BH1750_CONTINUOUS_HIGHRES:
    case BH1750_CONTINUOUS_HIGHRES_2:
      readingNum = 1;
      convertTime = 180*3; // 3 round of 180ms convertation (180ms - max time to complete High-resolution measurement)
      break;
    case BH1750_CONTINUOUS_LOWRES:
    default:  
      _mode = BH1750_CONTINUOUS_LOWRES;
      readingNum = 1;
      convertTime = 24*3; // 3 round of 24ms convertation (24ms - max time to complete Low-resolution measurement)
  }

  // Make some readings - 1 or 3 and get latest result
  while (readingNum) {
    readingNum--;
    // Wake up, sensor!
    writeByteToI2C(_i2cAddress, I2C_NO_REG_SPECIFIED, BH1750_CMD_POWERON);
    delay(10);
    // Start convertation
    writeByteToI2C(_i2cAddress, I2C_NO_REG_SPECIFIED, _mode);
    // Wait to complete covertation round
    delay(convertTime);
    // Read data
    readBytesFromi2C(_i2cAddress, I2C_NO_REG_SPECIFIED, (uint8_t*) _dst, 2);
  }

  result = WireToU16(_dst);


  if (SENS_READ_RAW == _metric) {
    ltoa(result, _dst, 10);
  } else {
    // Prepare result's value to using in ltoaf() subroutine
    // level = level/1.2; // convert to lux
    // 5 / 1.2 => 4,16
    // (5 * 1000) / 12 => 416 ==> ltoaf (..., ..., 2) ==> 4.16
    // max raw level = 65535 => 65535 * 1000 => long int
    result = (result * 1000) / 12;    
    ltoaf(result, _dst, 2);
  }

  return RESULT_IN_BUFFER;
}

