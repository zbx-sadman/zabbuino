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
static uint8_t waitToADPS9960Finished(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _mask, const uint16_t _timeout)
{
  uint8_t value;
  uint32_t startTime;

  startTime = millis();
  
  while((millis() - startTime) < _timeout){
     if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, _registerAddress, &value, 0x01)) { return false; }
     if (value & _mask) { return true; }
     delay(10);  
  }
  return false;
}

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getADPS9960Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint16_t _integrationTime, uint8_t _gain, uint8_t _ledDrive, const uint8_t _metric, uint32_t* _value)
{
  char stubBuffer;
  return getADPS9960Metric(_softTWI, _i2cAddress, _integrationTime, _gain, _ledDrive,_metric, &stubBuffer, _value, true);

}

int8_t getADPS9960Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint16_t _integrationTime, uint8_t _gain, uint8_t _ledDrive, const uint8_t _metric, char* _dst)
{
  uint32_t stubValue;
  return getADPS9960Metric(_softTWI, _i2cAddress, _integrationTime, _gain, _ledDrive, _metric, _dst, &stubValue, false);
}


/*****************************************************************************************************************************
*
*   Read specified metric's value of the ADPS9960 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on connection error
*     - DEVICE_ERROR_NOT_SUPPORTED on wrong parameter values
*     - RESULT_IS_FAIL on other fails
*
*****************************************************************************************************************************/
int8_t getADPS9960Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, uint16_t _integrationTime, uint8_t _gain, uint8_t _ledDrive, const uint8_t _metric, char* _dst, uint32_t* _value, const uint8_t _wantsNumber)
{
  int8_t rc = RESULT_IS_FAIL;
  uint8_t value[2] = {0x00, 0x00}, configByte, sensorMode;
  int32_t result;

//  if (SENS_READ_LUX != _metric) { rc = DEVICE_ERROR_NOT_SUPPORTED; goto finish; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_ID, value, 0x01)) { goto finish; }


  if (APDS9960_ID_01 != value[0] && APDS9960_ID_02 != value[0]) {
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish; 
  }

  // WAIT time bit is cleared. No wait time before ALS Engine starting
  sensorMode  =  (APDS9960_POWER_ON | APDS9960_ALS_ENABLE);
  sensorMode &= ~(APDS9960_PROXIMITY_DETECT_ENABLE | APSD9960_WAIT_ENABLE | APSD9960_ALS_INTERRUPT_ENABLE | APSD9960_ALS_PROXIMITY_ENABLE | APSD9960_GESTURE_ENABLE);

  if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_ENABLE, value, 0x01)) { goto finish; }
  if (sensorMode != value[0]) { 
     if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, APDS9960_REG_ENABLE, sensorMode)) { goto finish; }
  }

  delay(7); // 7 ms for EXIT SLEEP stage

  _integrationTime = (256 - (uint8_t) (((uint32_t) _integrationTime) * 100 / 278));

  //Serial.print("_integrationTime: "); Serial.println(_integrationTime); 

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

  
  configByte = (_gain | (_ledDrive << 6));
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
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_CDATAL, value, 0x02)) { goto finish; }
      break;
   
    case SENS_READ_LIGHT_RED:
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_RDATAL, value, 0x02)) { goto finish; }
      break;

    case SENS_READ_LIGHT_GREEN:
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_GDATAL, value, 0x02)) { goto finish; }
      break;

    case SENS_READ_LIGHT_BLUE:
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_BDATAL, value, 0x02)) { goto finish; }
      break;

    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish; 
   }

  result = WireToU16LE(value);

  *_value = result;

//  Serial.print("result: "); Serial.println(*_value);  

  if (!_wantsNumber) {
     ltoa(*_value, _dst, 10);
  }

  rc = RESULT_IS_BUFFERED;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}


