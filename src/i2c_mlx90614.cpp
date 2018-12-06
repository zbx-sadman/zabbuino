// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_mlx90614.h"

static uint8_t mlx90614crc8 (uint8_t inCrc, uint8_t inData) {
  uint8_t i, data;

  data = inCrc ^ inData;
  for ( i = 0; i < 8; i++ ) {
  	if (( data & 0x80 ) != 0 ) {
           data <<= 1;
	   data ^= 0x07;
	} else {
           data <<= 1;
        }
  }
  return data;
}

static uint8_t makePecWrite(const uint8_t _i2cAddress, const uint8_t _i2cRegister, const uint8_t _dataByteLow, const uint8_t _dataByteHigh) {
  uint8_t pec;

  pec = mlx90614crc8(0x00, (_i2cAddress << 1));
  pec = mlx90614crc8(pec,  _i2cRegister);
  pec = mlx90614crc8(pec,  value[0]);
  pec = mlx90614crc8(pec,  value[1]);

  return pec;
}
*/

static int8_t readRegister(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _i2cRegister, int16_t* _value) {
  uint8_t value[3] = {0x00, 0x00, 0x00}, pec;

  if (0x03 != readBytesFromI2C(_softTWI, _i2cAddress, _i2cRegister, value, 0x03)) { return DEVICE_ERROR_CONNECT; }

  pec = mlx90614crc8(0x00, (_i2cAddress << 1));
  pec = mlx90614crc8(pec,  _i2cRegister);
  pec = mlx90614crc8(pec,  (_i2cAddress << 1) + 1);
  pec = mlx90614crc8(pec,  value[0]);
  pec = mlx90614crc8(pec,  value[1]);

  if (pec != value[2]) { return DEVICE_ERROR_CHECKSUM; }

  *_value = WireToU16LE(value);

  return RESULT_IS_OK;
}


/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getMLX90614Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _temperatureZone, const uint8_t _metric, int32_t* _value)
{
  char stubBuffer;
  return getMLX90614Metric(_softTWI, _i2cAddress, _temperatureZone, _metric, &stubBuffer, _value, true);

}

int8_t getMLX90614Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _temperatureZone, const uint8_t _metric, char* _dst)
{
  int32_t stubValue;
  return getMLX90614Metric(_softTWI, _i2cAddress, _temperatureZone, _metric, _dst, &stubValue, false);
}


/*****************************************************************************************************************************
*
*   Read specified metric's value of the MLX90614 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on connection error
*     - RESULT_IS_FAIL on other fails
*
*****************************************************************************************************************************/
int8_t getMLX90614Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _temperatureZone, const uint8_t _metric, char* _dst, int32_t* _value, const uint8_t _wantsNumber)
{
  int8_t rc = RESULT_IS_FAIL;
  uint8_t readReg;
  int16_t result;

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  //Serial.println(">> 1");

  switch (_metric) {
    case SENS_READ_TEMP:
      switch (_temperatureZone) {
        case MLX90614_TEMPERATURE_ZONE_AMBIENT:
          readReg = MLX90614_TA;
          break;
        case MLX90614_TEMPERATURE_ZONE_01:
          readReg = MLX90614_TOBJ1;
          break;
        case MLX90614_TEMPERATURE_ZONE_02:
          readReg = MLX90614_TOBJ2;
          break;
        default:
          goto finish; 
          break;

      }   
      rc = readRegister(_softTWI, _i2cAddress, readReg, &result);
      if (RESULT_IS_OK != rc) { goto finish; }
      result = (result * 2) - 27315; // (temp * 0.02)-273.15 
      *_value = result;
      if (!_wantsNumber) {
         ltoaf(*_value, _dst, 2);
      }
      break;

    case SENS_READ_ID:
      //result = ((uint32_t) value[0] << 24) | ((uint32_t) value[1] << 16) | ((uint32_t) value[2] << 8) | value[3];
      break;
  }



  rc = RESULT_IS_BUFFERED;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}


