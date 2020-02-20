// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_tsl2561.h"


/*****************************************************************************************************************************
*
*  Read specified metric's value of the TSL2561 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success 
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getTSL2561Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint16_t _integrationTime, const uint8_t _gain, const uint8_t _metric, uint32_t* _value){

  int8_t   rc                = DEVICE_ERROR_TIMEOUT;
  uint8_t  configByte, 
           packageType, 
           partNo, 
           rawData[0x02]     = {0x00};
  uint16_t clipThreshold, b, m;
  uint32_t ratio, adcValueChannel00, adcValueChannel01, maxConversionTime, chScale;
  int32_t  result;


  if (SENS_READ_LUX != _metric) { rc = DEVICE_ERROR_NOT_SUPPORTED; goto finish; }

  if (!_i2cAddress) { _i2cAddress = TSL2561_I2C_ADDRESS; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, (TSL2561_COMMAND_BIT | TSL2561_REG_CONTROL), TSL2561_POWERON)) { goto finish; }

  if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, TSL2561_COMMAND_BIT | TSL2561_REG_ID, rawData, 0x01)) { goto finish; }

  partNo = rawData[0x00] & TSL2561_PARTNO_MASK;

  //  T, FN and CL package or CS package
  switch (partNo) {
    case TSL2561_PARTNO_TSL2560CS:
    case TSL2561_PARTNO_TSL2561CS:
      packageType = TSL2561_TYPE_CS;
      break;

    case TSL2561_PARTNO_TSL2560T:
    case TSL2561_PARTNO_TSL2561T:
      packageType = TSL2561_TYPE_T;
      break;

    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish; 
  }
 

  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, TSL2561_REG_INTERRUPT_CONTROL, TSL2561_INTERRUPT_DISABLED)) { goto finish; }

  configByte = 0x00;

  switch (_integrationTime) {
    case TSL2561_INTEGRATION_TIME_13MS: 
      configByte |= TSL2561_INTEGRATION_TIME_13MS_BITS;
      maxConversionTime = TSL2561_INTEGRATION_TIME_13MS_DELAY;
      clipThreshold = TSL2561_CLIPPING_13MS;
      chScale = TSL2561_CHSCALE_TINT0;
      break;
   
    case TSL2561_INTEGRATION_TIME_101MS:
      configByte |= TSL2561_INTEGRATION_TIME_101MS_BITS;
      maxConversionTime = TSL2561_INTEGRATION_TIME_101MS_DELAY;
      clipThreshold = TSL2561_CLIPPING_101MS;
      chScale = TSL2561_CHSCALE_TINT1;
      break;

    case TSL2561_INTEGRATION_TIME_402MS:
      configByte |= TSL2561_INTEGRATION_TIME_402MS_BITS;
      maxConversionTime = TSL2561_INTEGRATION_TIME_402MS_DELAY;
      clipThreshold = TSL2561_CLIPPING_402MS;
      chScale = (0x01 << TSL2561_CH_SCALE); 
      break;

    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish; 
   }

  
  switch (_gain) {
    case TSL2561_GAIN_1X: 
      configByte |= TSL2561_GAIN_1X_BITS;
      chScale = chScale << 0x04;
      break;
    
    case TSL2561_GAIN_16X:
      configByte |= TSL2561_GAIN_16X_BITS;
      break;
    
    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish; 
  }


  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, (TSL2561_COMMAND_BIT | TSL2561_REG_TIMING), configByte)) { goto finish; }

  // Restart conversion with new settings
  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, (TSL2561_COMMAND_BIT | TSL2561_REG_CONTROL), TSL2561_POWEROFF)) { goto finish; }
  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, (TSL2561_COMMAND_BIT | TSL2561_REG_CONTROL), TSL2561_POWERON)) { goto finish; }

  delay(maxConversionTime);

  // FullSpectrum
  if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, (TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_ADC_CHANNEL00_LOW), rawData, 0x02)) { goto finish; }
  
  adcValueChannel00 = WireToU16LE(rawData);

  // Infrared
  if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, (TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_ADC_CHANNEL01_LOW), rawData, 0x02)) { goto finish; }

  adcValueChannel01 = WireToU16LE(rawData);

  if (0x01 != writeByteToI2C(_softTWI, _i2cAddress, (TSL2561_COMMAND_BIT | TSL2561_REG_CONTROL), TSL2561_POWEROFF)) { goto finish; }

  /* Return 65536 lux if the sensor is saturated */
  if ((adcValueChannel00 > clipThreshold) || (adcValueChannel01 > clipThreshold)) { result = 65536; goto printOut; }

  adcValueChannel00 = (adcValueChannel00 * chScale) >> TSL2561_CH_SCALE;
  adcValueChannel01 = (adcValueChannel01 * chScale) >> TSL2561_CH_SCALE;

  ratio = 0x00;

  if (0x00 != adcValueChannel00) { ratio = (adcValueChannel01 << (TSL2561_RATIO_SCALE + 0x01)) / adcValueChannel00; }
  ratio = (ratio + 0x01) >> 0x01;

  switch (packageType) {
    case TSL2561_TYPE_T: // T package
      // !!! ratio variable is unsigned
      // if ((ratio >= 0) && (ratio <= TSL2561_K1T)) {
      if (ratio <= TSL2561_K1T) {
         b = TSL2561_B1T; m = TSL2561_M1T;

      } else if (ratio <= TSL2561_K2T) {
         b = TSL2561_B2T; m = TSL2561_M2T;

      } else if (ratio <= TSL2561_K3T) {
         b = TSL2561_B3T; m = TSL2561_M3T;

      } else if (ratio <= TSL2561_K4T) {
         b = TSL2561_B4T; m = TSL2561_M4T;

      } else if (ratio <= TSL2561_K5T) {
         b = TSL2561_B5T; m = TSL2561_M5T;

      } else if (ratio <= TSL2561_K6T) {
         b = TSL2561_B6T; m = TSL2561_M6T;

      } else if (ratio <= TSL2561_K7T) {
         b = TSL2561_B7T; m = TSL2561_M7T;

      } else if (ratio > TSL2561_K8T) {
         b = TSL2561_B8T; m = TSL2561_M8T;

      }
      break;

    case TSL2561_TYPE_CS:// CS package
      // !!! ratio variable is unsigned
      //if ((ratio >= 0) && (ratio <= TSL2561_K1C)) {
      if (ratio <= TSL2561_K1C) {
         b = TSL2561_B1C; m = TSL2561_M1C;

      } else if (ratio <= TSL2561_K2C) {
         b = TSL2561_B2C; m = TSL2561_M2C;

      } else if (ratio <= TSL2561_K3C) { 
         b = TSL2561_B3C; m = TSL2561_M3C;

      } else if (ratio <= TSL2561_K4C) {
         b = TSL2561_B4C; m = TSL2561_M4C;

      } else if (ratio <= TSL2561_K5C) {
         b = TSL2561_B5C; m = TSL2561_M5C;

      } else if (ratio <= TSL2561_K6C) {
         b = TSL2561_B6C; m = TSL2561_M6C;

      } else if (ratio <= TSL2561_K7C) {
         b = TSL2561_B7C; m = TSL2561_M7C; 

      } else if (ratio > TSL2561_K8C) {
         b = TSL2561_B8C; m = TSL2561_M8C;

      }
      break;
  } // switch (packageType)
    
  
  // TSL2561 datasheet:
  //    unsigned long temp;
  //    temp = ((channel0 * b) - (channel1 * m));
  //    // do not allow negative lux value
  //    if (temp < 0) temp = 0; <<<<<< WTF? temp variable is unsigned long
  //
  result = ((adcValueChannel00 * b) - (adcValueChannel01 * m));

  // do not allow negative lux value
  if (result < 0x00) { result = 0x00; }
  // round lsb (2^(LUX_SCALE-1))
  result += (0x01 << (TSL2561_LUX_SCALE - 0x01));
  // strip off fractional portion
  result = result >> TSL2561_LUX_SCALE;

  // !!! Now result contains lux'es
printOut:

  *_value = result;

  //DEBUG_PORT.print("lux: "); DEBUG_PORT.println(*_value); 
  rc = RESULT_IS_UNSIGNED_VALUE;

finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}


