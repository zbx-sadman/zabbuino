// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_adps9960.h"

/*****************************************************************************************************************************
*
*   Waiting for a timeout to detect the ADPS9960 sensor finshed conversion
*
*   Returns: 
*     - true on sensor ready
*     - false on timeout
*
**************************************************************************************************************************** */
static uint8_t waitToADPS9960Finished(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _mask, const uint16_t _timeout) {
  uint8_t  rawData;
  uint32_t startTime;

  startTime = millis();
  
  while((millis() - startTime) < _timeout) {
     if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, _registerAddress, &rawData, 0x01)) { return false; }
     if (rawData & _mask) { return true; }
     delay(10);  
  }
  return false;
}

/*****************************************************************************************************************************
*
*  Read specified metric's value of the ADPS9960 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success 
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getADPS9960Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint16_t _integrationTime, uint8_t _gain, uint8_t _ledDrive, const uint8_t _metric, uint32_t* _value) {
  int8_t   rc                = DEVICE_ERROR_TIMEOUT;
  uint8_t  rawData[0x02]     = {0x00}, 
           configByte, 
           sensorMode;

  if (!_i2cAddress) { _i2cAddress = ADPS9960_I2C_ADDRESS; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_ID, rawData, 0x01)) { goto finish; }

  if (APDS9960_ID_01 != rawData[0x00] && APDS9960_ID_02 != rawData[0x00]) { rc = DEVICE_ERROR_NOT_SUPPORTED; goto finish; }

  if (0x00 == _integrationTime) { _integrationTime = APDS9960_DEFAULT_ADC_INTEGRATION_TIME; }

  _integrationTime = constrain(_integrationTime, APDS9960_MIN_ADC_INTEGRATION_TIME, APDS9960_MAX_ADC_INTEGRATION_TIME);

  // WAIT time bit is cleared. No wait time before ALS Engine starting
  sensorMode  =  (APDS9960_POWER_ON | APDS9960_ALS_ENABLE);
  sensorMode &= ~(APDS9960_PROXIMITY_DETECT_ENABLE | APSD9960_WAIT_ENABLE | APSD9960_ALS_INTERRUPT_ENABLE | APSD9960_ALS_PROXIMITY_ENABLE | APSD9960_GESTURE_ENABLE);

  if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_ENABLE, rawData, 0x01)) { goto finish; }
  if (sensorMode != rawData[0]) { 
     if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, APDS9960_REG_ENABLE, sensorMode)) { goto finish; }
  }

  delay(7); // 7 ms for EXIT SLEEP stage

  _integrationTime = (256 - (uint8_t) (((uint32_t) _integrationTime) * 100 / 278));

  //DEBUG_PORT.print("_integrationTime: "); DEBUG_PORT.println(_integrationTime); 


  switch (_gain) {
    case 0x01:
      _gain = APDS9960_ALS_GAIN_1X;
      break;
    case 0x04:
      _gain = APDS9960_ALS_GAIN_4X;
      break;
    case 0x10:
      _gain = APDS9960_ALS_GAIN_16X;
      break;
    case 0x40:
      _gain = APDS9960_ALS_GAIN_64X;
      break;
    default:
      _gain = APDS9960_DEFAULT_ALS_GAIN;
      break;
  }

  switch (_ledDrive) {
    case 0x0C:
      _ledDrive = APDS9960_LED_DRIVE_12_5MA;
      break;
    case 0x19:
      _ledDrive = APDS9960_LED_DRIVE_25MA;
      break;
    case 0x32:
      _ledDrive = APDS9960_LED_DRIVE_50MA;
      break;
    case 0x64:
      _ledDrive = APDS9960_LED_DRIVE_100MA;
      break;
    default:
      _ledDrive = APDS9960_DEFAULT_LED_DRIVE;
      break;
  }

  
  configByte = (_gain | (_ledDrive << 0x06));
  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, APDS9960_REG_CONTROL_ONE, configByte)) { goto finish; }

  // 0x01 set first bit to 1 as datasheet says
  configByte = (APDS9960_DEFAULT_LED_BOOST | 0x01);
  configByte &= ~(APDS9960_DEFAULT_ALS_SATURATION_INTERRUPT_ENABLE | APDS9960_DEFAULT_PROXIMITY_SATURATION_INTERRUPT_ENABLE);

  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, APDS9960_REG_CONFIG_TWO, configByte)) { goto finish; }
  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, APDS9960_REG_AICLEAR)) { goto finish; }

  // Wait for data ready bit
  // ???: May be _integrationTime+10% will be better that APDS9960_CONVERSION_TIMEOUT?
  if (!waitToADPS9960Finished(_softTWI, _i2cAddress, APDS9960_REG_STATUS, APSD9960_STATUS_AINT, APDS9960_CONVERSION_TIMEOUT)) { goto finish; }

  // RGBC results can be used to calculate ambient light levels
  // (i.e. Lux) and color temperature (i.e. Kelvin).
  switch (_metric) {
    case SENS_READ_LIGHT_AMBIENT: 
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_CDATAL, rawData, 0x02)) { goto finish; }
      break;
   
    case SENS_READ_LIGHT_RED:
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_RDATAL, rawData, 0x02)) { goto finish; }
      break;

    case SENS_READ_LIGHT_GREEN:
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_GDATAL, rawData, 0x02)) { goto finish; }
      break;

    case SENS_READ_LIGHT_BLUE:
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_BDATAL, rawData, 0x02)) { goto finish; }
      break;

    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish; 
   }

  *_value = WireToU16LE(rawData);

  //DEBUG_PORT.print("val: "); DEBUG_PORT.println(*_value, HEX);  
  rc = RESULT_IS_UNSIGNED_VALUE;

finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}
