// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_sht.h"


static uint8_t calcSHT2xCRC(uint8_t* _src) {

  uint32_t remainder = WireToU24(_src);

  uint32_t divisor = (uint32_t) SHT2X_CRC_SHIFTED_DIVISOR;

  for (uint8_t i = 0x00 ; 0x10 > i ; i++) {
    if (remainder & ((uint32_t) 0x01 << (0x17 - i))) { remainder ^= divisor; }
    divisor >>= 1; 
  }

  //DEBUG_PORT.print("CRC: "); DEBUG_PORT.println((uint8_t) remainder);
  return (uint8_t) remainder;

}


/*****************************************************************************************************************************
*
*  Read specified metric's value of the SHT2X sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_FLOAT_02_DIGIT    on success when TEMP metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*    - DEVICE_ERROR_CHECKSUM       on detect data corruption
*
*****************************************************************************************************************************/
int8_t getSHT2XMetric(SoftwareTWI* _softTWI, uint8_t _i2cAddress, const uint8_t _metric, int32_t* _value) {
  int8_t   rc                = DEVICE_ERROR_TIMEOUT;
  uint8_t  command;
  uint8_t  rawData[0x03]     = {0x00};
  uint16_t data;
  uint32_t maxConversionTime;

  if (!_i2cAddress) { _i2cAddress = SHT2X_I2C_ADDRESS; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { 
     rc = DEVICE_ERROR_CONNECT;
     goto finish; 
  }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  switch (_metric) {
    case SENS_READ_TEMP:
      command = SHT2X_CMD_MEASURE_TEMPERATURE_HOLD;
      maxConversionTime = SHT2X_TEMPERATURE_14BIT_CONVERSION_TIME_MS;
      break;
      
    case SENS_READ_HUMD:
      command = SHT2X_CMD_MEASURE_HUMIDITY_HOLD;
      maxConversionTime = SHT2X_HUMIDITY_12BIT_CONVERSION_TIME_MS;
      break;

    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish;
  }

  // Set mode
  if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, SHT2X_CMD_READ_USER_REGISTER, rawData, 0x01)) { goto finish; }
  rawData[0x00] &= SHT2X_CONVERSION_MODE_CLEAR_MASK;
  rawData[0x00] |= SHT2X_CONVERSION_MODE_00;
  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, SHT2X_CMD_WRITE_USER_REGISTER, rawData[0x00])) { goto finish; }

  // make conversion
  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, command)) { goto finish; }
  delay(maxConversionTime);    
  // Read data
  if (0x03 != readBytesFromI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, rawData, 0x03)) { goto finish; }
  

  if (0x00 != calcSHT2xCRC(rawData)) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }

  data = WireToU16(rawData);
  data &= ~0x0003;   // clear two low bits (status bits)

  
  switch (_metric) {
    case SENS_READ_TEMP: {
      // Default humidity  resolution - 14 bit => 0.01C
      // all part of equation must be multiplied by 100 to get 2-digit fract part by ltoaf() 
      // H_real = 100 * -6 + (100 * 125 * H_raw) / (2^16)
      *_value = ((int32_t) (((uint32_t) data * 17572) >> 16)) - 4685;
      break;
    }
      
    case SENS_READ_HUMD: {
      // Default humidity  resolution - 12 bit => 0.04%RH
      // all part of equation must be multiplied by 100 to get 2-digit fract part by ltoaf() 
      // H_real = 100 * -6 + (100 * 125 * H_raw) / (2^16)
      *_value = ((int32_t) (((uint32_t) data * 100 * 125) >> 16)) - 600; 
      // Due to normal variations in RH accuracy of the device as described in Table 4, it is possible for the measured value
      // of %RH to be slightly less than 0 when the actual RH level is close to or equal to 0. Similarly, the measured value
      // of %RH may be slightly greater than 100 when the actual RH level is close to or equal to 100. This is expected
      // behavior, and it is acceptable to limit the range of RH results to 0 to 100%RH in the host software by truncating
      // values that are slightly outside of this range.
      // Si7021-A20 datashet, 5.1.1. Measuring Relative Humidity
      if (0x00  > *_value) { *_value = 0x00;  }
      if (10000 < *_value) { *_value = 10000; }
      break;
    }
  }

  rc = RESULT_IS_FLOAT_02_DIGIT;

finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}


