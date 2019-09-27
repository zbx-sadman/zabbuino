// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_veml6070.h"

/*****************************************************************************************************************************
*
*  Read specified metric's value of the VEML6070 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success when RAW metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getVEML6070Metric(SoftwareWire* _softTWI, uint8_t _integrationFactor, uint32_t _rsetValue, const uint8_t _metric, int32_t* _value) {
  int8_t   rc                = DEVICE_ERROR_NOT_SUPPORTED;
  uint8_t  configReg         = (0x01 << VEML6070_CONFIG_RESERVED), 
           rawData[0x02]     = {0x00};
  uint32_t refreshTime;
  uint16_t rawValue, 
           uviFactor;

  switch (_integrationFactor) {
    // 1T measurement interval
    case 0x00:
      _integrationFactor = 0x01;

    case 0x01:
      // Datasheet say: (1) clear ACK state of UVS and (2) fill the initial value, 06 (HEX), into the 0x70 addresses
      // 0x06 => B00000110 => B00000100 (1T refresh time) + B00000010 (bit 2, reserved - set initial value to 1)
      configReg |= (VEML6070_1_T << VEML6070_CONFIG_IT0);
      // "Designing the VEML6070 UV Light Sensor Into Applications", table 2 => 2055 / 11
      uviFactor = 187;
      break;

    // 2T measurement interval
    case 0x02:
      // -"-"-
      configReg |= (VEML6070_2_T << VEML6070_CONFIG_IT0);
      // -"-"- => 4109 / 11
      uviFactor = 373;
      break;

    // 4T measurement interval
    case 0x04:
      // -"-"-
      configReg |= (VEML6070_4_T << VEML6070_CONFIG_IT0);
      // -"-"- => 8217 / 11
      uviFactor = 747;
      break;

    // Other modes not supported
    default:
      goto finish;
  } // switch (_integrationFactor)

  if (0x00 == _rsetValue) {
    _rsetValue = VEML6070_RSET_VALUE_DEFAULT;
  }

  if (VEML6070_MIN_RSET_VALUE > _rsetValue || VEML6070_MAX_RSET_VALUE < _rsetValue) {
    goto finish;
  }

  if (!isI2CDeviceReady(_softTWI, VEML6070_ADDR_L)) {
    rc = DEVICE_ERROR_CONNECT;
    goto finish;
  }

  rc = DEVICE_ERROR_TIMEOUT;

  // Init phase
  if (0x01 != writeByteToI2C(_softTWI, VEML6070_ADDR_L, I2C_NO_REG_SPECIFIED, configReg)) {
    goto finish;
  }
  // After the initialization is completed, VEML6070 can be
  // programmable for operation by write command setting from the host. VEML6070 initialization is recommended to be completed within 150 ms.
  delay(VEML6070_INIT_TIME_MS);

  // 1T refreshTime = Rset / 2.344, VEML6070 datasheet, fig.7 "Refresh time"
  refreshTime = _integrationFactor * (_rsetValue * 1000 / 2344);
  
  delay(refreshTime);
  
  if (0x01 != readBytesFromI2C(_softTWI, VEML6070_ADDR_H, I2C_NO_REG_SPECIFIED, &rawData[0], 1)) {
    goto finish;
  }

  if (0x01 != readBytesFromI2C(_softTWI, VEML6070_ADDR_L, I2C_NO_REG_SPECIFIED, &rawData[1], 1)) {
    goto finish;
  }

  // Switch sensor to SD (shutdown) mode
  configReg |= (0x01 << VEML6070_CONFIG_SD);
  if (0x01 != writeByteToI2C(_softTWI, VEML6070_ADDR_L, I2C_NO_REG_SPECIFIED, configReg)) {
    goto finish;
  }

  rawValue = WireToU16(rawData);

  // This is UV Level scaled to the default Rset = 270kOhm and 1T integration time.
  // "Designing the VEML6070 UV Light Sensor Into Applications" document contain table to translate this "default" UV level to the UV-Index
  // ...
  // scaledUVLevel = (rawValue * VEML6070_RSET_VALUE_DEFAULT * 1000) / (_rsetValue * 100) =>  (rawValue * VEML6070_RSET_VALUE_DEFAULT * 10) / _rsetValue;
  // ...
  //scaledUVLevel = ((uint32_t) rawValue * VEML6070_RSET_VALUE_DEFAULT * 10) / _rsetValue;
  // *_value = scaledUVLevel / uviFactor;
  //scaledUVLevel = ((uint32_t) rawValue * VEML6070_RSET_VALUE_DEFAULT * 10) / _rsetValue;
  *_value = ((uint32_t) rawValue * VEML6070_RSET_VALUE_DEFAULT * 10) / _rsetValue;

  switch (_metric) {
    case SENS_READ_UVA:
      // UVA (1T) = scaledUVLevel (nT) / _integrationFactor (1, 2 or 4) / 10 (scale down to "unfloat" value);
      *_value /= 10 * _integrationFactor;
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

    case SENS_READ_UVI:
      // UVI = scaledUVLevel / uviFactor;
      *_value /=  uviFactor;
      rc = RESULT_IS_FLOAT_01_DIGIT;
      break;
/*
    case SENS_READ_RAW:
      *_value = rawValue;
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;
*/
    // Other modes not supported
    default:
      goto finish;
  }


finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;
}
