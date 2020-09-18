// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_bh1750.h"

/*****************************************************************************************************************************
*
*  Read specified metric's value of the BH1750 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success when RAW metric specified
*    - RESULT_IS_FLOAT_02_DIGIT    on success when LUX metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getBH1750Metric(SoftwareTWI* _softTWI, uint8_t _i2cAddress, const uint8_t _metric, int32_t* _value) {
  int8_t   rc                = DEVICE_ERROR_TIMEOUT;
  uint8_t  readingNum        = 0x02, 
           rawData[0x02]     = {0x00},       	
           measurementMode   = BH1750_ONETIME_HIGHRES;
  uint16_t rawValue          = 0x00; 
  uint32_t maxConversionTime = BH1750_HIGHRES_CONVERSION_TIME_MS;

  if (!_i2cAddress) { _i2cAddress = BH1750_I2C_ADDRESS; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  while (readingNum) {
    readingNum--;

    if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, BH1750_CMD_POWERON)) { goto finish; }

    if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, measurementMode)) { goto finish; }
    delay(maxConversionTime);

    // Read data
    if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, rawData, 0x02)) { goto finish; }

    rawValue = WireToU16(rawData);

    // On the dark - switch to the 0.11 lx resolution
    if (rawValue <= BH1750_HIGHRES_2_TRESHOLD) {
       measurementMode = BH1750_ONETIME_HIGHRES_2;
    } else {
       readingNum = 0x00;
    }
  }

  *_value = rawValue;

  rc = RESULT_IS_UNSIGNED_VALUE;

  if (SENS_READ_LUX == _metric) {
    // Prepare result's value to using in ltoaf() subroutine
    // level = level/1.2; // convert to lux
    // 5 / 1.2 => 4,16
    // (5 * 1000) / 12 => 416 ==> ltoaf (..., ..., 2) ==> 4.16
    // max raw level = 65535 => 65535 * 1000 => long int
    *_value = (*_value * 1000L) / 12;    
    rc = RESULT_IS_FLOAT_02_DIGIT;
  }

finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;
}

