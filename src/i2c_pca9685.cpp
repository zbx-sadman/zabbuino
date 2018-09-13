// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_pca9685.h"

/*****************************************************************************************************************************
*
*   Set sate of the PCA9685 outputs. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on detect of connection error
*     - DEVICE_ERROR_NOT_SUPPORTED on wrong parameters set
*     - RESULT_IS_FAIL on other fails
*
*****************************************************************************************************************************/
int8_t writePCA9685(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int8_t _outputIdx, const uint16_t _onTime, const uint16_t _offTime)
{
  int8_t rc;
  uint8_t value[4];
  uint8_t i2cReg;

  rc = DEVICE_ERROR_NOT_SUPPORTED; 

  // FULL_ON and FULL_OFF states can't be turned on at the same time
  if (PCA9685_CHANNEL_FULL_STATE == _onTime && PCA9685_CHANNEL_FULL_STATE == _offTime) { goto finish; }

  // LED_ON and LED_OFF values can't be more than 4096 
  // if (PCA9685_CHANNEL_FULL_STATE < _onTime || 0x00 > _onTime || PCA9685_CHANNEL_FULL_STATE < _offTime || 0x00 > _offTime) { goto finish; }
  if (PCA9685_CHANNEL_FULL_STATE < _onTime || PCA9685_CHANNEL_FULL_STATE < _offTime ) { goto finish; }

  // Channel Idx can't be equal or more than PCA9685_CHANNEL_COUNT or less than PCA9685_CHANNEL_ALL_LEDS (All leds Idx)
  if (PCA9685_CHANNEL_COUNT <= _outputIdx || PCA9685_CHANNEL_LEDS_ALL > _outputIdx ) { goto finish; }


  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  i2cReg = (PCA9685_CHANNEL_LEDS_ALL == _outputIdx) ? PCA9685_LEDS_ALL_REG : (PCA9685_LEDS_START_REG + (_outputIdx * 0x04));

  U16LToWire(value, _onTime);
  U16LToWire(&value[2], _offTime);

  if (0x04 != writeBytesToI2C(_softTWI, _i2cAddress, i2cReg, value, 0x04)) { rc = RESULT_IS_FAIL; goto finish; }

  rc = RESULT_IS_OK; 

  finish:
//  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}

