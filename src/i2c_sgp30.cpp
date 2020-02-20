// Config & common included files   
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_sgp30.h"

static uint8_t crcSGP30(uint8_t* _src) {
  uint8_t crc = 0xFF; //Init with 0xFF
  crc ^= _src[0]; // XOR-in the first input byte
  for (uint8_t i = 0 ; i < 8 ; i++) {
    if ((crc & 0x80) != 0) { crc = (uint8_t)((crc << 1) ^ 0x31); } else { crc <<= 1; }
  }

  crc ^= _src[1]; // XOR-in the last input byte

  for (uint8_t i = 0 ; i < 8 ; i++) {
    if ((crc & 0x80) != 0) { crc = (uint8_t)((crc << 1) ^ 0x31); } else { crc <<= 1; }
  }

  return crc; //No output reflection
}

/*****************************************************************************************************************************
*
*  Read specified metric's value of the SGP30 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success 
*    - RESULT_IS_FLOAT_02_DIGIT    on success when LUX metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*    - DEVICE_ERROR_CHECKSUM       on detect data corruption
*
*****************************************************************************************************************************/

int8_t getSGP30Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const int32_t _absHumidity, const uint8_t _metric, const uint8_t _reInit, int32_t* _value) {
  static uint8_t initialized = false;
  int8_t   rc                = DEVICE_ERROR_TIMEOUT;
  uint8_t  buffer[0x06]      = {0x00};

  if (!_i2cAddress) { _i2cAddress = SGP30_I2C_ADDRESS; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  if (_absHumidity > 256000) { goto finish; rc = DEVICE_ERROR_NOT_SUPPORTED; }

  buffer[0x00] = SGP30_CMD_BYTE_COMMON;

  if (!initialized || _reInit) {
     buffer[0x01] = SGP30_CMD_BYTE_IAQ_INIT;
     if (0x02 != writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, buffer, 0x02)) { goto finish; }
     delay(SGP30_TIME_IAQ_INIT);
     initialized = true;   
  }

  if (_absHumidity >= 0x00) {
     uint16_t ahScaled = (uint16_t)(((uint64_t)_absHumidity * 256 * 16777) >> 24);     
     buffer[0x01] = SGP30_CMD_BYTE_SET_ABSOLUTE_HUMIDITY;
     buffer[0x02] = ahScaled >> 8;
     buffer[0x03] = ahScaled & 0xFF;
     buffer[0x04] = crcSGP30(&buffer[0x02]);
     if (0x05 != writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, buffer, 0x05)) { goto finish; }
     delay(50);
  }

  buffer[0x01] = SGP30_CMD_BYTE_IAQ_MEASURE;

  if (0x02 != writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, buffer, 0x02)) { goto finish; }

  delay(SGP30_TIME_IAQ_MEASURE);

  if (0x06 != readBytesFromI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, buffer, 0x06)) { goto finish; }

  // CRC check
  rc = DEVICE_ERROR_CHECKSUM;
  
  if (buffer[SGP30_CO2_BYTE_CRC]  != crcSGP30(&buffer[SGP30_CO2_BYTE_DATA]) || 
      buffer[SGP30_TVOC_BYTE_CRC] != crcSGP30(&buffer[SGP30_TVOC_BYTE_DATA]))  { goto finish; }

                                  
  switch(_metric) {
    case SENS_READ_TVOC:
      *_value = WireToU16((uint8_t*)&buffer[SGP30_TVOC_BYTE_DATA]);
      break;

    case SENS_READ_CO2E:
      *_value = WireToU16((uint8_t*)&buffer[SGP30_CO2_BYTE_DATA]);
      break;
    
    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish; 
  }

  rc = RESULT_IS_UNSIGNED_VALUE;

finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;
}

