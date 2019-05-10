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
*   Read specified metric's value of the SGP30 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on test connection error
*     - RESULT_IS_FAIL - on other fails
*
*****************************************************************************************************************************/
static int8_t getSGP30Metric(SoftwareWire& _softTWI, const uint8_t _i2cAddress, const uint8_t _metric, const uint8_t _reInit, char *_dst, uint32_t* _value, const uint8_t _wantsNumber)
{
  int8_t rc = DEVICE_ERROR_CONNECT;
  uint8_t value[6];
  uint8_t* ptrData;
  uint16_t command;
  static uint8_t initialized = false;

  if (!isI2CDeviceReady(&_softTWI, _i2cAddress)) { goto finish; }

  rc = RESULT_IS_FAIL;

  if (!initialized || _reInit) {
     command = SGP30_CMD_IAQ_INIT;
     if (sizeof(command) != writeBytesToI2C((SoftwareWire*) &_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, (uint8_t*) &command, sizeof(command))) { goto finish; }
     delay(SGP30_TIME_IAQ_INIT);
     initialized = true;   
  }

  command = SGP30_CMD_IAQ_MEASURE;
  if (sizeof(command) != writeBytesToI2C(&_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, (uint8_t*) &command, sizeof(command))) { goto finish; }
  delay(SGP30_TIME_IAQ_MEASURE);
  if (sizeof(value) != readBytesFromI2C(&_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, value, sizeof(value))) { goto finish; }

  // CRC check
  rc = DEVICE_ERROR_CHECKSUM;
  if (value[SGP30_CO2E_CRC_BYTE] != crcSGP30(&value[SGP30_CO2E_DATA_BYTE])) { goto finish; }
  if (value[SGP30_TVOC_CRC_BYTE] != crcSGP30(&value[SGP30_TVOC_DATA_BYTE])) { goto finish; }

                                  
  switch(_metric) {
    case SENS_READ_TVOC:
      ptrData = &value[SGP30_TVOC_DATA_BYTE];
      *_value = WireToU16(ptrData);
    break;

    case SENS_READ_CO2E:
      //*_value = WireToU16(&value[SGP30_CO2_DATA_BYTE]);
      ptrData = &value[SGP30_CO2E_DATA_BYTE];
      *_value = WireToU16(ptrData);
    break;
 
    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish; 
    break;
  }

  if (!_wantsNumber) {
     ltoa(*_value, _dst, 10);
  }

  rc = RESULT_IS_BUFFERED;

finish:
  return rc;
}

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getSGP30Metric(SoftwareWire& _softTWI, const uint8_t _i2cAddress, const uint8_t _metric, const uint8_t _reInit, uint32_t* _value)
{
  char stubBuffer;
  return getSGP30Metric(_softTWI, _i2cAddress, _metric, _reInit, &stubBuffer, _value, true);
}

int8_t getSGP30Metric(SoftwareWire& _softTWI, uint8_t _i2cAddress, const uint8_t _metric, const uint8_t _reInit, char* _dst)
{
  uint32_t stubValue;
  return getSGP30Metric(_softTWI, _i2cAddress, _metric, _reInit, _dst, &stubValue, false);
}
