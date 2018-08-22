// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_bh1750.h"

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getBH1750Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _metric, uint32_t* _value)
{
  char stubBuffer;
  return getBH1750Metric(_softTWI, _i2cAddress, _mode, _metric, &stubBuffer, _value, true);
//  return getBH1750Metric(_softTWI, _i2cAddress, _mode, _metric, 0x00, _value, true);

}

int8_t getBH1750Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _metric, char* _dst)
{
  uint32_t stubValue;
  return getBH1750Metric(_softTWI, _i2cAddress, _mode, _metric, _dst, &stubValue, false);
  //  pointer to fake 'value' variable can be used, because called subroutine never write by this pointer
  //  return getBH1750Metric(_softTWI, _i2cAddress, _mode, _metric, _dst, (uint32_t*) _dst, false);
}

/*****************************************************************************************************************************
*
*   Read specified metric's value of the BH1750 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on test connection error
*     - RESULT_IS_FAIL - on other fails
*
*****************************************************************************************************************************/
int8_t getBH1750Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _metric, char *_dst, uint32_t* _value, const uint8_t _wantsNumber)
{
  int8_t rc = RESULT_IS_FAIL;
  uint8_t readingNum, 
          value[2];   // do not use _dst array instead value array, due strange behavior detected - sometime _dst[n] is not uint8_t
                      // println(_dst[n], BIN) can show 1111111111101010 for example. Need to make some cast experiments
  int16_t convertTime;
  // result variable must be 32bit because readed from sensor 16bit value will be scaled to avoid float calculation

  //  int32_t ;
  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }


  switch (_mode) {
    case BH1750_ONETIME_HIGHRES: 
    case BH1750_ONETIME_HIGHRES_2:
      readingNum = 2;
      convertTime = 180; // 180ms - max time to complete High-resolution measurement
      break;
    case BH1750_ONETIME_LOWRES:
       readingNum = 2;
       convertTime = 24; // 24ms - max time to complete Low-resolution measurement
       break;
    case BH1750_CONTINUOUS_HIGHRES:
    case BH1750_CONTINUOUS_HIGHRES_2:
      readingNum = 1;
      convertTime = 180; // 2 round of 180ms convertation (180ms - max time to complete High-resolution measurement)
      break;
    case BH1750_CONTINUOUS_LOWRES:
    default:  
      _mode = BH1750_CONTINUOUS_LOWRES;
      readingNum = 1;
      convertTime = 24; // 2 round of 24ms convertation (24ms - max time to complete Low-resolution measurement)
  }

  // Make some readings - 1 or 3 and get latest result
  while (readingNum) {
    --readingNum;
    // Wake up, sensor!
    // It going sleep after One-Time measurement 
    if (0x00 != writeByteToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, BH1750_CMD_POWERON)) { goto finish; }
    //delay(10);
    // Start convertation
    if (0x00 != writeByteToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, _mode)) { goto finish; }
    // Wait to complete covertation round
    delay(convertTime);
    // Read data
    if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, value, 0x02)) { goto finish; }
  }

  *_value = (((int32_t) value[0]) << 8) | value[1];

  if (SENS_READ_RAW == _metric) {
    ltoa(*_value, _dst, 10);
  } else {
    // Prepare result's value to using in ltoaf() subroutine
    // level = level/1.2; // convert to lux
    // 5 / 1.2 => 4,16
    // (5 * 1000) / 12 => 416 ==> ltoaf (..., ..., 2) ==> 4.16
    // max raw level = 65535 => 65535 * 1000 => long int
    *_value = (*_value * 1000) / 12;    
    // If required - put to variable '_value' scaled value or it whole part only.
    if (!_wantsNumber) {
       ltoaf(*_value, _dst, 2);
    }
  }
  rc = RESULT_IS_BUFFERED;
  finish:
  return rc;
}

