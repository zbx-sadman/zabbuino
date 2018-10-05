// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_adps9960.h"

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getADPS9960Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _metric, uint32_t* _value)
{
  char stubBuffer;
  return getADPS9960Metric(_softTWI, _i2cAddress, _metric, &stubBuffer, _value, true);

}

int8_t getADPS9960Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _metric, char* _dst)
{
  uint32_t stubValue;
  return getADPS9960Metric(_softTWI, _i2cAddress, _metric, _dst, &stubValue, false);
}


/*****************************************************************************************************************************
*
*   Read specified metric's value of the TSL2561 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on connection error
*     - DEVICE_ERROR_NOT_SUPPORTED on wrong parameter values
*     - RESULT_IS_FAIL on other fails
*
*****************************************************************************************************************************/
int8_t getADPS9960Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _metric, char* _dst, uint32_t* _value, const uint8_t _wantsNumber)
{
  int8_t rc = RESULT_IS_FAIL;
  uint8_t value[2] = {0x00, 0x00}, configByte, sensorMode;
  int32_t result;

  Serial.println("start"); 

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_ID, value, 0x01)) { goto finish; }

  Serial.print("id: "); Serial.println((uint8_t) value[0], HEX); 

  if (APDS9960_ID_01 != value[0] && APDS9960_ID_02 != value[0]) {
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish; 
  }

  sensorMode  =  (APDS9960_POWER_ON | APDS9960_ALS_ENABLE);
  sensorMode &= ~(APDS9960_PROXIMITY_DETECT_ENABLE | APSD9960_WAIT_ENABLE | APSD9960_ALS_INTERRUPT_ENABLE | APSD9960_ALS_PROXIMITY_ENABLE | APSD9960_GESTURE_ENABLE);

//  Serial.print("sensorMode: "); Serial.println(sensorMode, BIN); 

  if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_ENABLE, value, 0x01)) { goto finish; }
  if (sensorMode != value[0]) { 
     if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, APDS9960_REG_ENABLE, sensorMode)) { goto finish; }
  }
  
  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, APDS9960_REG_ADC_INTEGRATION_TIME, APDS9960_DEFAULT_ADC_INTEGRATION_TIME)) { goto finish; }

  configByte = (APDS9960_DEFAULT_ALS_GAIN | (APDS9960_DEFAULT_LED_DRIVE << 6));

//  Serial.print("Control One: "); Serial.println(configByte, BIN); 

  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, APDS9960_REG_CONTROL_ONE, configByte)) { goto finish; }

  // 0x01 set first bit to 1 as datasheet says
  configByte = (APDS9960_DEFAULT_LED_BOOST | 0x01);
  configByte &= ~(APDS9960_DEFAULT_ALS_SATURATION_INTERRUPT_ENABLE | APDS9960_DEFAULT_PROXIMITY_SATURATION_INTERRUPT_ENABLE);

//  Serial.print("Config Two: "); Serial.println(configByte, BIN); 

  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, APDS9960_REG_CONFIG_TWO, configByte)) { goto finish; }

  delay(250);

  switch (_metric) {
    case SENS_READ_AMBIENT: 
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_CDATAL, value, 0x02)) { goto finish; }
      break;
   
    case SENS_READ_RED:
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_RDATAL, value, 0x02)) { goto finish; }
      break;

    case SENS_READ_GREEN:
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_GDATAL, value, 0x02)) { goto finish; }
      break;

    case SENS_READ_BLUE:
      if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, APDS9960_REG_BDATAL, value, 0x02)) { goto finish; }
      break;

    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish; 
   }

    
  result = WireToU16LE(value);

  *_value = result;

  Serial.print("result: "); Serial.println(*_value); 

  if (!_wantsNumber) {
     ltoa(*_value, _dst, 10);
  }

  rc = RESULT_IS_BUFFERED;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}


