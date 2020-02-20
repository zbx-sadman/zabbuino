// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_max44009.h"

/*****************************************************************************************************************************
*
*  Read specified metric's value of the MAX44009 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_FLOAT_03_DIGIT    on success when LUX metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getMAX44009Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint8_t _mode, const uint8_t _newTIM, const uint8_t _metric, int32_t* _value) {
  __SUPPRESS_WARNING_UNUSED(_metric);

  int8_t   rc                      = DEVICE_ERROR_TIMEOUT;
  const uint16_t integrationTime[] = {800, 400, 200, 100, 50, 25, 13, 7};
  uint8_t  rawData[0x02]           = {0x00},
           configReg               = 0x00,
           luxExponent;
           
  uint16_t conversionTime;

  if (!_i2cAddress) { _i2cAddress = MAX44009_I2C_ADDRESS; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  switch (_mode){
    case MAX44009_800MS_CYCLE_MODE: {
      // wait 800ms, because no way to get end of conversion flag
      configReg = _mode;
      conversionTime = 800;
      break;
    }

    case MAX44009_CONTINUOUS_MODE: {
      if (arraySize(integrationTime) > _newTIM) {
         // We need wait some time because no any 'conversion finished' signal exists
         // Seems that MAX44009 read TIM, make conversion, read TIM again, and make conversion again. 
         // If TIM will be changed on the half way of second conversion round, raw value will be wrong (so high or so low)
         //
         // So, if we want to get correct results when TIM changed, then we must:
         // 1) Wait while previous conversion will be done - add old TIM interval to waiting time;
         // 2) Wait for new conversion finish - add new interval to waiting time;
         if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, MAX44009_REG_CONFIGURATION, rawData, 0x01)) { goto finish; }
         uint8_t oldTIM = rawData[0] & MAX44009_TIM_MASK;
         conversionTime = integrationTime[_newTIM];
         if (oldTIM != _newTIM) { conversionTime += integrationTime[oldTIM]; } 
         // "Manual" mode, bit 6 must be 0x01 + Set integration time 
         // CDR byte is dropped, current not divided. All of the photodiode current goes to the ADC.
         configReg |= MAX44009_MANUAL_TIM_MODE | _newTIM;
         break;
      }

    // No break operator here its OK

    }

    default: {
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish;
    }
  }


  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, MAX44009_REG_CONFIGURATION, configReg)) { goto finish; }
  
  delay(conversionTime);

  if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, MAX44009_REG_LUXREADING, rawData, 0x02)) { goto finish; }

  luxExponent = ((rawData[0] >> 4) & 0x0F);
  rawData[0]  = ((rawData[0] << 4) & 0xF0);
  rawData[1] &= 0x0F;

  *_value = 45L * (rawData[0] | rawData[1]) * (1 << luxExponent);

  rc = RESULT_IS_FLOAT_03_DIGIT;

finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}

