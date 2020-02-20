// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_pca9685.h"
#include "i2c_bus.h"

/*****************************************************************************************************************************
*
*  Set PCA9685's outputs to the specified state.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success 
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t writePCA9685(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int8_t _outputIdx, const uint16_t _onTime, const uint16_t _offTime, const uint8_t _outputMode) {

  __SUPPRESS_WARNING_UNUSED(_outputMode);

  int8_t   rc                      = DEVICE_ERROR_TIMEOUT;
  uint8_t  rawData[0x04]           = {0x00},
           configByte, i2cReg;

  // FULL_ON and FULL_OFF states can't be turned on at the same time
  if (4096 == _onTime && 4096 == _offTime) { goto finish; }

  // LED_ON and LED_OFF values can't be more than 4096 
  if (4096 < _onTime || 4096 < _offTime)   { goto finish; }

  // Channel Idx can't be equal or more than PCA9685_CHANNEL_COUNT or less than PCA9685_CHANNEL_ALL_LEDS (All leds Idx)
  if (PCA9685_CHANNEL_COUNT <= _outputIdx || PCA9685_CHANNEL_LEDS_ALL > _outputIdx ) { goto finish; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }
  
  rc = DEVICE_ERROR_TIMEOUT; 

  configByte = (0x01 << PCA9685_REG_MODE1_AI) | (0x01 << PCA9685_REG_MODE1_ALLCALL); // drop 'sleep' bit which set on IC start

  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, PCA9685_REG_MODE1, configByte)) { goto finish; }

  // Refer to i2c_pca9685.h to know something about MODE2 settings
  // configByte = (0x00 << PCA9685_REG_MODE2_OUTNE) | ((PCA9685_MODE_TOTEM_POLE == _outputMode ? PCA9685_OUTDRV_TOTEM_POLE: PCA9685_OUTDRV_OPEN_DRAIN) << PCA9685_REG_MODE2_OUTDRV);
  configByte = (0x00 << PCA9685_REG_MODE2_OUTNE) | (PCA9685_OUTDRV_TOTEM_POLE << PCA9685_REG_MODE2_OUTDRV);

  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, PCA9685_REG_MODE2, configByte)) { goto finish; }

  i2cReg = (PCA9685_CHANNEL_LEDS_ALL == _outputIdx) ? PCA9685_REG_ALL_LED : (PCA9685_REG_LED_00 + (_outputIdx * 0x04));
  //DEBUG_PORT.print("i2cReg: "); DEBUG_PORT.println(i2cReg, HEX);

  U16LToWire(&rawData[0x00], _onTime); U16LToWire(&rawData[0x02], _offTime);

  if (0x04 != writeBytesToI2C(_softTWI, _i2cAddress, i2cReg, rawData, 0x04)) { goto finish; }

  rc = RESULT_IS_OK; 

finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}

