// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_max44009.h"

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getMAX44009Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _integration_time, const uint8_t _metric, uint32_t* _value)
{
  char stubBuffer;
  return getMAX44009Metric(_softTWI, _i2cAddress, _mode, _integration_time, _metric, &stubBuffer, _value, true);
}

int8_t getMAX44009Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _integration_time, const uint8_t _metric, char* _dst)
{
  uint32_t stubValue;
  return getMAX44009Metric(_softTWI, _i2cAddress, _mode, _integration_time, _metric, _dst, &stubValue, false);
}

/*****************************************************************************************************************************
*
*   Read specified metric's value of the MAX44009 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on test connection error
*     - RESULT_IS_FAIL - on other fails
*
*   Note: sensor can return wrong values if given integration time so short to make measurement
*
*
*****************************************************************************************************************************/
int8_t getMAX44009Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _integration_time, const uint8_t _metric, char *_dst, uint32_t* _value, const uint8_t _wantsNumber)
{
  int8_t rc = RESULT_IS_FAIL;
  uint8_t autoTIM = false,
          luxExponent,
          value[2];
  uint16_t integrationTime[] = {800, 400, 200, 100, 50, 25, 13, 7};
  uint16_t waitTime = 0;
 

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  // _mode must be 0x80 or 0x00
  // invalid _integration_time cause switching sensor to auto measure mode
  if (MAX44009_CONTINUOUS_MODE == _mode) {
     if (MAX44009_INTEGRATION_TIME_100MS < _integration_time) { autoTIM = true; }
  } else {
     _mode = MAX44009_800MS_CYCLE_MODE;
     // MAX44009_INTEGRATION_TIME_6MS is latest mode (idx = 0x07)
     if (MAX44009_INTEGRATION_TIME_6MS < _integration_time) { autoTIM = true; }
  }

  // Manual mode choosed
  if (! autoTIM) {
/*     switch (_integration_time) {
       case MAX44009_INTEGRATION_TIME_800MS:
         waitTime = 800;
         break;
       case MAX44009_INTEGRATION_TIME_400MS:
         waitTime = 400;
         break;
       case MAX44009_INTEGRATION_TIME_200MS:
         waitTime = 200;
         break;
       case MAX44009_INTEGRATION_TIME_100MS:
         waitTime = 100;
         break;
       case MAX44009_INTEGRATION_TIME_50MS:
         waitTime = 50;
         break;
       case MAX44009_INTEGRATION_TIME_25MS:
         waitTime = 25;
         break;
       case MAX44009_INTEGRATION_TIME_12MS:
         waitTime = 13; // Must be 12.5, but i do not use float
         break;
       case MAX44009_INTEGRATION_TIME_6MS:
         waitTime = 7;  // Must be 6.25, but i do not use float
         break;
     }
*/
     waitTime = integrationTime[_integration_time];

     // "Manual" mode, bit 6 must be 1
     _mode |= 0x40;
     // Set integration time 
     // CDR byte is dropped, current not divided. All of the photodiode current goes to the ADC.
     _mode |= _integration_time;
     // Seems that need wait max integration time (800ms) on first reading after configuration byte changing to avoid get wrong values
     if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, MAX44009_REG_CONFIGURATION, value, 0x01)) { goto finish; }
     if (value[0] != _mode) { waitTime = 800; }
  } else {
    // wait 800ms, because no way to get end of conversion flag
    waitTime = 800;
  }


  if (0x00 != writeByteToI2C(_softTWI, _i2cAddress, MAX44009_REG_CONFIGURATION, _mode)) { goto finish; }
  
  delay(waitTime);
  if (0x00 != readBytesFromI2C(_softTWI, _i2cAddress, MAX44009_REG_LUXREADING, value, 2)) { goto finish; }


  luxExponent = ((value[0] >> 4) & 0x0F);
  value[0]    = ((value[0] << 4) & 0xF0);
  value[1]   &= 0x0F;

  *_value = 45L * (value[0] | value[1]) * (1<< luxExponent);
  if (!_wantsNumber) {
       ltoaf(*_value, _dst, 3);
  }

  rc = RESULT_IS_BUFFERED;
  finish:
  return rc;
}

