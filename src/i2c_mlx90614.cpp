// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_mlx90614.h"


/*****************************************************************************************************************************
*
*  Update CRC for one byte
*
*****************************************************************************************************************************/
static uint8_t calcMLX90614Crc(const uint8_t _inCrc, const uint8_t _inData) {
  uint8_t crc;

  crc = _inCrc ^ _inData;
  for (uint8_t i = 0x00; 0x08 > i; i++) {
    if ((crc & 0x80) != 0x00) {
       crc <<= 1;
       crc ^= 0x07;
    } else {
       crc <<= 1;
    }
  }
  return crc;
}

/*****************************************************************************************************************************
*
*  Read i6 bit register (send command) from MLX90614 sensor
*  Example: RAM Access opcode is '000x xxxx'. TObj1 register is '0000 0111'. 'Read TObj1' commmand is '000x xxxx' & '0000 0111' => '0000 0111'
*
*****************************************************************************************************************************/
static int8_t readMLX90614Register(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _command, int16_t* _value) {

  uint8_t pec, value[0x03] = {0x00}; 

  if (0x03 != readBytesFromI2C(_softTWI, _i2cAddress, _command, value, 0x03)) { return DEVICE_ERROR_TIMEOUT; }
  pec = calcMLX90614Crc(0x00, (_i2cAddress << 1));
  pec = calcMLX90614Crc(pec,  _command);
  pec = calcMLX90614Crc(pec,  (_i2cAddress << 1) + 1);
  pec = calcMLX90614Crc(pec,  value[0x00]);
  pec = calcMLX90614Crc(pec,  value[0x01]);

  // Is CRC equal PEC ?
  if (pec != value[0x02]) { return DEVICE_ERROR_CHECKSUM; }

  *_value = WireToU16LE(value);

  return RESULT_IS_OK;
}

/*****************************************************************************************************************************
*
*  Read specified metric's value of the MLX90614 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_FAIL              on any error flags 
*    - DEVICE_ERROR_WRONG_ANSWER   on error flag detected in the Ta/To
*    - RESULT_IS_UNSIGNED_VALUE    on success when ID metric specified
*    - RESULT_IS_FLOAT_02_DIGIT    on success when TEMP metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getMLX90614Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint8_t _temperatureZone, const uint8_t _metric, int32_t* _value) {

  int8_t   rc                = DEVICE_ERROR_NOT_SUPPORTED;
  uint8_t  command;
  int16_t  rawValue;

  if (!_i2cAddress) { _i2cAddress = MLX90614_I2C_ADDR; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

/*
  // !!! No native ways found in the SoftwareWire lib to get Flags from the sensor

  // put error code to rc var
  //rc = readMLX90614Register(_softTWI, _i2cAddress, MLX90614_OPCODE_READ_FLAGS, &result);

  if (RESULT_IS_OK != rc) { goto finish; }
  // 'Flags' command returns 0x00 if all OK,
  if (0x00 != result) { rc = RESULT_IS_FAIL; goto finish;}
*/
  switch (_metric) {
    case SENS_READ_TEMP:
      switch (_temperatureZone) {
        case MLX90614_TEMPERATURE_ZONE_AMBIENT:
          command = MLX90614_OPCODE_READ_RAM | MLX90614_TA;
          break;

        case MLX90614_TEMPERATURE_ZONE_01:
          command = MLX90614_OPCODE_READ_RAM | MLX90614_TOBJ1;
          break;

        case MLX90614_TEMPERATURE_ZONE_02:
          command = MLX90614_OPCODE_READ_RAM | MLX90614_TOBJ2;
          break;

        default:
          goto finish; 
      }   

      rc = readMLX90614Register(_softTWI, _i2cAddress, command, &rawValue);
      if (RESULT_IS_OK != rc) { goto finish; }
      // MLX90614 returns error flag in the MSB
      if (MLX90614_TX_ERROR_MASK & rawValue) { rc = DEVICE_ERROR_WRONG_ANSWER; goto finish; }
      *_value = ((int32_t) rawValue * 2) - 27315; // (temp * 0.02)-273.15 
      rc = RESULT_IS_FLOAT_02_DIGIT;
      break;

    case SENS_READ_ID:
      //result = ((uint32_t) value[0] << 24) | ((uint32_t) value[1] << 16) | ((uint32_t) value[2] << 8) | value[3];
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

    default:
      goto finish; 
  }

finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}
