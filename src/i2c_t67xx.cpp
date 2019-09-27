// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_t67xx.h"

static int8_t getT67XXRawData(SoftwareWire* _softTWI, const uint8_t _i2cAddress, uint8_t* _buffer) { 

  int8_t  rc = DEVICE_ERROR_TIMEOUT;

  if (T67XX_I2C_REQUEST_SIZE  != writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, _buffer, T67XX_I2C_REQUEST_SIZE)) { goto finish; }

  delay(T67XX_I2C_RESPONSE_DELAY);

  if (T67XX_I2C_RESPONSE_SIZE != readBytesFromI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, _buffer, T67XX_I2C_RESPONSE_SIZE)) { goto finish; }
/*
  for (uint8_t i = 0x00; i < T67XX_I2C_RESPONSE_SIZE; i++) {
    Serial.print(" 0x"); Serial.print(_buffer[i], HEX);

  }
  Serial.println();
*/
  if (0x04 != _buffer[T67XX_I2C_FIELD_FUNCTION_CODE] || 0x02 != _buffer[T67XX_I2C_FIELD_BYTES_COUNT]) { rc = DEVICE_ERROR_WRONG_ANSWER; goto finish; }

  rc = RESULT_IS_OK;

finish:

 return rc;

}


/*****************************************************************************************************************************
*
*  Read specified metric's value of the T67XX sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE       on success when CONCENTRATION metric specified
*    - DEVICE_ERROR_EEPROM_CORRUPTED  on sensor's flash error detect
*    - DEVICE_ERROR_TIMEOUT           on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT           on connection error
*
*****************************************************************************************************************************/

int8_t getT67XXMetric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint8_t _metric, int32_t* _value) {
  __SUPPRESS_WARNING_UNUSED(_metric);

  int8_t  rc = DEVICE_ERROR_CONNECT;
  uint8_t rawData[T67XX_I2C_REQUEST_SIZE] = {0x04, T67XX_MODBUS_ADDRESS_STATUS_MSB, T67XX_MODBUS_ADDRESS_STATUS_LSB, 0x00, 0x01};
  uint16_t rawValue = 0x00;

  if (!_i2cAddress) { _i2cAddress = T67XX_I2C_ADDRESS; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { goto finish; }

  // Take status

  rc = getT67XXRawData(_softTWI, _i2cAddress, rawData);

  if (RESULT_IS_OK != rc) { goto finish; }

  rawValue = ((uint16_t) rawData[T67XX_I2C_FIELD_DATA_MSB] << 8) | rawData[T67XX_I2C_FIELD_DATA_LSB];

  if (T67XX_MASK_STATUS_ERROR_FLASH & rawValue) { rc = DEVICE_ERROR_EEPROM_CORRUPTED; goto finish; }

  if (T67XX_MASK_STATUS_OK != rawValue) { rawValue = T67XX_PREHEAT_GAS_CONCENTRATION; goto printOut; }

  //Serial.print("Status: B"); Serial.println(rawValue, BIN);

  // Take PPM

  rawData[T67XX_I2C_FIELD_STARTING_ADDRESS_MSB] = T67XX_MODBUS_ADDRESS_GAS_PPM_MSB;
  rawData[T67XX_I2C_FIELD_STARTING_ADDRESS_LSB] = T67XX_MODBUS_ADDRESS_GAS_PPM_LSB;
  rawData[T67XX_I2C_FIELD_REGISTER_MSB] = 0x00;
  rawData[T67XX_I2C_FIELD_REGISTER_LSB] = 0x01;

  rc = getT67XXRawData(_softTWI, _i2cAddress, rawData);

  if (RESULT_IS_OK != rc) { goto finish; }

  //Serial.println("* 4");

  rawValue = ((uint16_t) rawData[T67XX_I2C_FIELD_DATA_MSB] << 8) | rawData[T67XX_I2C_FIELD_DATA_LSB];

  //Serial.print("rawValue: "); Serial.println(rawValue);

printOut:

  rc = RESULT_IS_UNSIGNED_VALUE;
  *_value = rawValue;

finish:


  return rc;
}
