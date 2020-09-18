// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_bmp.h"


#if defined(FEATURE_BMP_ENABLE)
/*****************************************************************************************************************************
*
*   Waiting for a timeout to detect the BMP sensor ready state
*
*   Returns: 
*     - true on sensor ready
*     - false on timeout
*
**************************************************************************************************************************** */
static uint8_t waitToBMPReady(SoftwareTWI* _softTWI, const uint8_t _i2cAddress, const int16_t _registerAddress, const int16_t _mask, const uint16_t _timeout) {
  uint8_t value;
  uint32_t startTime;

  startTime = millis();
  
  while((millis() - startTime) < _timeout){
     readBytesFromI2C(_softTWI, _i2cAddress, _registerAddress, &value, 0x01);
     if (0x00 == (value & _mask)) return true;
     delay(10);  
  }
  return false;
}
#endif



#ifdef SUPPORT_BMP280_INCLUDE 
/*****************************************************************************************************************************
*
*  Read specified metric's value of the BMP280/BME280 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success when SENS_READ_PRSS metric specified
*    - RESULT_IS_FLOAT_02_DIGIT    on success when SENS_READ_TEMP metric specified
*    - RESULT_IS_FLOAT_QMN         on success when SENS_READ_HUMD metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*
*****************************************************************************************************************************/
static int8_t getBME280Metric(SoftwareTWI* _softTWI, uint8_t _i2cAddress, const uint8_t _overSampling, uint8_t _filterCoef, const uint8_t _metric, int32_t* _value) {
  int8_t  rc              = DEVICE_ERROR_TIMEOUT;
  int32_t adc, var1, var2, fineTemperature;
  uint8_t success         = true,
          rawData[0x03]   = {0x00};

  // set work mode
  switch ( _overSampling) {
    // BMP280 DS 3.4: Drop detection 
    // BMP280 DS 3.8.1, BME280 DS 9.1: Max measurement time: 8.7ms
    case BMP280_LOWPOWER: 
      var1 = BMP280_LOWPOWER_OVERSAMP_TEMPERATURE;
      var2 = BMP280_LOWPOWER_OVERSAMP_PRESSURE;
      break;
    // BMP280 DS 3.4: Elevator / floor change detection , handheld device dynamic (e.g. Android)
    // BMP280 DS 3.8.1, BME280 DS 9.1: Max measurement time: 13.3ms
    case BMP280_STANDARD:
      var1 = BMP280_STANDARD_OVERSAMP_TEMPERATURE;
      var2 = BMP280_STANDARD_OVERSAMP_PRESSURE;
      break;
    // ??
    // BMP280 DS 3.8.1, BME280 DS 9.1: Max measurement time: 22.5ms
    case BMP280_HIGHRES:
      var1 = BMP280_HIGHRES_OVERSAMP_TEMPERATURE;
      var2 = BMP280_HIGHRES_OVERSAMP_PRESSURE;
      break;
    // BMP280 DS 3.4: Indoor navigation, handheld device low-power (e.g. Android)
    // BMP280 DS 3.8.1, BME280 DS 9.1: Max measurement time: 43.2ms
    case BMP280_ULTRAHIGHRES:
      var1 = BMP280_ULTRAHIGHRES_OVERSAMP_TEMPERATURE;
      var2 = BMP280_ULTRAHIGHRES_OVERSAMP_PRESSURE;
      break;
    // BMP280 DS 3.4:  Weather monitoring (lowest power)
    // BMP280 DS 3.8.1, BME280 DS 9.1: Max measurement time: 6.4ms
    case BMP280_ULTRALOWPOWER: 
    default:  
      var1 = BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
      var2 = BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE;
  }
  
  // We must wait to Bxxxx0xx0 value in "Status" register
  if (!waitToBMPReady(_softTWI, _i2cAddress, BMP280_REGISTER_STATUS, BMP280_READY_MASK, BMP280_READY_TIMEOUT)) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value

  // The “ctrl_hum” register sets the humidity data acquisition options of the device. Changes to this register only become effective after a write operation to “ctrl_meas”.
  success &= (0x01 == writeByteToI2C(_softTWI, _i2cAddress, BME280_REGISTER_CONTROLHUMID, BME280_STANDARD_OVERSAMP_HUMIDITY));
  // Set filter coefficient. While BMP280 used in Forced mode - BMP280_STANDBY_TIME_4000_MS can be replaced to any time. '0' - define SPI configuration. Not used.
  success &= (0x01 == writeByteToI2C(_softTWI, _i2cAddress, BMP280_REGISTER_CONFIG, ((BMP280_STANDBY_TIME_4000_MS << 0x06)| (_filterCoef << 0x03) | 0x00 )));
  if (!success) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value

  // BMP280_FORCED_MODE -> BME280 act like BMP180
  // We need "forced mode" to avoid mix-up bytes of data belonging to different measurements of various metrics on "normal mode"
  // Need to double read to take properly data (not from previous measure round)
  // use var as 'mode' variable
  var1 = (var1 << 6)| (var2 << 3) | BMP280_FORCED_MODE;

  for (uint8_t i = 0x00; 0x02 > i; i++) { 
    success &= (0x01 == writeByteToI2C(_softTWI, _i2cAddress, BMP_REGISTER_CONTROL, var1));
    // We must wait to Bxxxx0xx0 value in "Status" register. It's most reliable way to detect end of conversion 
    if (!waitToBMPReady(_softTWI, _i2cAddress, BMP280_REGISTER_STATUS, BMP280_READY_MASK, BMP280_READY_TIMEOUT)) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value  
  }

  // Compensate temperature caculation

  // Read raw value  
  success &= (0x03 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_TEMPDATA, rawData, 0x03));
  
  adc = ((int32_t) (((uint16_t) rawData[0x00] << 0x08) | (uint16_t) rawData[0x01]) << 0x04) | ((uint16_t) rawData[0x02] >> 0x04);
  {
     uint16_t dig_T1;
     int16_t  dig_T2, dig_T3;
     // read calibration data 
     success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_T1, rawData, 0x02));
     dig_T1 = WireToU16LE(rawData);

     success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_T2, rawData, 0x02));
     dig_T2 = WireToS16LE(rawData);
                                                                  
     success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_T3, rawData, 0x02));
     dig_T3 = WireToS16LE(rawData);;
     if (!success) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value


     var1 = ((((adc >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
     var2 = (((((adc >> 4) - ((int32_t) dig_T1)) * ((adc >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3)) >> 14;

     fineTemperature = var1 + var2;
  }

  switch (_metric) {
    case SENS_READ_TEMP: {
      // real temperature caculation
      *_value = (fineTemperature * 5 + 128) >> 0x08;
      //  return T/100;    
      rc = RESULT_IS_FLOAT_02_DIGIT;
      break;
    }

    case SENS_READ_PRSS: {
      uint16_t dig_P1;
      int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
      // read calibration data 
      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_P1, rawData, 0x02));
      dig_P1 = WireToU16LE(rawData);
      
      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_P2, rawData, 0x02));
      dig_P2 = WireToS16LE(rawData);
      
      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_P3, rawData, 0x02));
      dig_P3 = WireToS16LE(rawData);

      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_P4, rawData, 0x02));
      dig_P4 = WireToS16LE(rawData);
      
      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_P5, rawData, 0x02));
      dig_P5 = WireToS16LE(rawData);
      
      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_P6, rawData, 0x02));
      dig_P6 = WireToS16LE(rawData);
      
      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_P7, rawData, 0x02));
      dig_P7 = WireToS16LE(rawData);
      
      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_P8, rawData, 0x02));
      dig_P8 = WireToS16LE(rawData);
      
      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_DIG_P9, rawData, 0x02));
      dig_P9 = WireToS16LE(rawData);
 
      // Test value
      // adc = 415148 ( from BOSCH datasheet)
      // Read raw value  
      success &= (0x03 == readBytesFromI2C(_softTWI, _i2cAddress, BMP280_REGISTER_PRESSUREDATA, rawData, 0x03));
      if (!success) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value
 
      adc = ((int32_t) (((uint16_t) rawData[0x00] << 0x08) | (uint16_t) rawData[0x01]) << 0x04) | ((uint16_t) rawData[0x02] >> 0x04);
   
      // Compensate pressure caculation
      var1 = (((int32_t)fineTemperature) >> 1) - (int32_t) 64000;
      var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t) dig_P6);
      var2 = var2 + ((var1 * ((int32_t) dig_P5)) << 1);
      var2 = (var2 >> 2) + (((int32_t) dig_P4) << 16);
      var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t) dig_P2) * var1) >> 1)) >> 18;
      var1 = ((((32768 + var1)) * ((int32_t) dig_P1)) >> 15);

      // avoid exception caused by division by zero
      if (0x00 == var1) { 
         *_value = 0x00; 
      } else {
         *_value = (((uint32_t)(((int32_t) 1048576) - adc)-(var2 >> 12))) * 3125;
         *_value = ((uint32_t) *_value < 0x80000000L) ? ((*_value << 1) / ((uint32_t) var1)) : ((*_value / (uint32_t) var1) * 2); 

/*
      if (*_value < 0x80000000) {
         *_value = (*_value << 1) / ((uint32_t) var1); 
      } else {
         *_value = (*_value / (uint32_t) var1) * 2; 
      }
*/
         var1 = (((int32_t) dig_P9) * ((int32_t)(((*_value >> 3) * (*_value >> 3)) >> 13))) >> 12;
         var2 = (((int32_t)(*_value >> 2)) * ((int32_t) dig_P8)) >> 13;
         *_value = (uint32_t)((int32_t) *_value + ((var1 + var2 + dig_P7) >> 4));
         // BOSCH on page 22 of datasheet say: "Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits)." 
         // 24674867 in Q24.8 is 96386 in whole part (24674867 >> 8) , and 51 in frac part (24674867 & B11111111) => 96386.51
         // But in code example use calculation: 24674867/256 = 96386.19 => 96386.2
         //
         // What way is right, BOSCH? 
         //
      }
     rc = RESULT_IS_UNSIGNED_VALUE;
     break;
    }
     
#ifdef SUPPORT_BME280_INCLUDE
    case SENS_READ_HUMD: {
      uint8_t  dig_H1, dig_H3;
      int8_t   dig_H6;
      int16_t  dig_H2, dig_H4, dig_H5;

      //read calibration data

      success &= (0x01 == readBytesFromI2C(_softTWI, _i2cAddress, BME280_REGISTER_DIG_H1, rawData, 0x01));
      dig_H1 = WireToU8(rawData);
  
      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BME280_REGISTER_DIG_H2, rawData, 0x02));
      dig_H2 = WireToS16LE(rawData);

      success &= (0x01 == readBytesFromI2C(_softTWI, _i2cAddress, BME280_REGISTER_DIG_H3, rawData, 0x01));
      dig_H3 = WireToU8(rawData);

      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BME280_REGISTER_DIG_H4, rawData, 0x02));
      dig_H4 = (int16_t) (((uint16_t) rawData[0x00] << 0x04) | (rawData[0x01] & 0x0F));

      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BME280_REGISTER_DIG_H5, rawData, 0x02));
      dig_H5 = (int16_t) ((uint16_t)  rawData[0x01] << 0x04) | (rawData[0x00] >> 0x04);

      success &= (0x01 == readBytesFromI2C(_softTWI, _i2cAddress, BME280_REGISTER_DIG_H6, rawData, 0x01));
      dig_H6 = WireToS8(rawData);

      // Read raw value  
      success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BME280_REGISTER_HUMIDDATA, rawData, 0x02));
      if (!success) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value

      // adc take wrong value due overflow if rawData[] no cast to (uint16_t) 
      adc = (int32_t) WireToU16(rawData);

      // Compensate humidity caculation
      var1 = (fineTemperature -  ((int32_t) 76800));
      var1 = (((((adc << 14) - (((int32_t) dig_H4) << 20) - (((int32_t) dig_H5) * var1)) +
             ((int32_t) 16384)) >> 15) * (((((((var1 * ((int32_t) dig_H6)) >> 10) * (((var1 * 
             ((int32_t) dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
             ((int32_t) dig_H2) + 8192) >> 14));
      var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
      var1 = (var1 < 0 ? 0 : var1);
      var1 = (var1 > 419430400 ? 419430400 : var1);
      *_value = (uint32_t)(var1 >> 12);
      // Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
      //	 Output value of “47445” represents 47445/1024 = 46.333 %RH
      //DEBUG_PORT.print("H: "); DEBUG_PORT.println(*_value);
      rc = RESULT_IS_FLOAT_QMN;
    }
#endif        
  }  // switch (_metric)

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;
}
#endif



#ifdef SUPPORT_BMP180_INCLUDE
/*****************************************************************************************************************************
*
*  Read specified metric's value of the BMP180/BMP085 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success when SENS_READ_PRSS metric specified
*    - RESULT_IS_FLOAT_01_DIGIT    on success when SENS_READ_TEMP metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*
*****************************************************************************************************************************/
static int8_t getBMP180Metric(SoftwareTWI* _softTWI, uint8_t _i2cAddress, uint8_t _overSampling, const uint8_t _metric, int32_t* _value) {
  int8_t  rc              = DEVICE_ERROR_TIMEOUT;
  uint8_t success = true,
          rawData[0x03]   = {0x00};
  // Calibration values
  int16_t ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t b1, b2, mc, md; // mb - not used 
  uint16_t ut;
  uint32_t up;
  int32_t x1, x2, x3, b3, b5, b6;//, result;
  uint32_t b4, b7;

  /* read calibration data */
  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_CAL_AC1, rawData, 0x02));
  ac1 = WireToS16(rawData);
  
  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_CAL_AC2, rawData, 0x02));
  ac2 = WireToS16(rawData);
  
  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_CAL_AC3, rawData, 0x02));
  ac3 = WireToS16(rawData);
  
  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_CAL_AC4, rawData, 0x02));
  ac4 = WireToU16(rawData);
  
  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_CAL_AC5, rawData, 0x02));
  ac5 = WireToU16(rawData);
  
  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_CAL_AC6, rawData, 0x02));
  ac6 = WireToU16(rawData);
  
  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_CAL_B1, rawData, 0x02));
  b1 = WireToS16(rawData);
  
  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_CAL_B2, rawData, 0x02));
  b2 = WireToS16(rawData);
  
  // Not used in calculation (see datasheet)
  //readBytesFromI2C(_i2cAddress, BMP180_REGISTER_CAL_MB, value, 2);
  //mb = WireToS16(value);

  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_CAL_MC, rawData, 0x02));
  mc = WireToS16(rawData);

  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_CAL_MD, rawData, 0x02));
  md = WireToS16(rawData);

  if (!success) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value

  switch ( _overSampling) {
    case BMP180_ULTRALOWPOWER: 
    case BMP180_STANDARD:
    case BMP180_HIGHRES:
    case BMP180_ULTRAHIGHRES:
       break;
     default:  
        _overSampling = BMP180_STANDARD;
  }

  // We must wait for 50ms to Bxx0xxxxx value in "Control" register to know that device is not busy
  if (!waitToBMPReady(_softTWI, _i2cAddress, BMP_REGISTER_CONTROL, BMP180_READY_MASK, BMP180_READY_TIMEOUT)) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value
  // ******** Get Temperature ********
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  success &= (0x01 == writeByteToI2C(_softTWI, _i2cAddress, BMP_REGISTER_CONTROL, BMP180_CMD_READTEMP));

  // Wait at least 4.5ms
  //delay(5);

  // We must wait for 50ms to Bxx0xxxxx value in "Control" register to know about end of conversion
  if (!waitToBMPReady(_softTWI, _i2cAddress, BMP_REGISTER_CONTROL, BMP180_READY_MASK, BMP180_READY_TIMEOUT)) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value
  // Read two bytes from registers 0xF6 and 0xF7
  success &= (0x02 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_TEMPDATA, rawData, 0x02));
  ut = WireToU16(rawData);

  if (!success) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value


  x1 = (((int32_t) ut - (int32_t) ac6) * (int32_t) ac5) >> 15;
  x2 = ((int32_t) mc << 11) / (x1 + (int32_t) md);
  b5 = x1 + x2;
  *_value = ((b5 + 8) >> 4);

  if (SENS_READ_PRSS == _metric) {
    // ******** Get Pressure ********

    // Calculate pressure given up
    // calibration values must be known
    // b5 is also required so get temperature procedure must be called first.
    // Value returned will be pressure in units of Pa.

    // Write 0x34+(_oversampling<<6) into register 0xF4
    // Request a pressure reading w/ oversampling setting
    success &= (0x01 == writeByteToI2C(_softTWI, _i2cAddress, BMP_REGISTER_CONTROL, BMP180_CMD_READPRESSURE + (_overSampling << 0x06)));

    // Wait for conversion, delay time dependent on _oversampling
    delay(0x02 + (0x03 << _overSampling));

    // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
    success &= (0x03 == readBytesFromI2C(_softTWI, _i2cAddress, BMP180_REGISTER_PRESSUREDATA, rawData, 0x03));
    if (!success) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value

    up = (((uint32_t) rawData[0x00] << 16) | ((uint32_t) rawData[0x01] << 0x08) | (uint32_t) rawData[0x02]) >> (0x08 - _overSampling);

    b6 = b5 - 4000;

    // Calculate B3
    x1 = ((int32_t) b2 * (b6 * b6) >> 12) >> 11;
    x2 = ((int32_t) ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)ac1) * 4 + x3) << _overSampling) + 2) >> 2;

    // Calculate B4
    x1 = ((int32_t) ac3 * b6) >> 13;
    x2 = ((int32_t) b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = ((int32_t) ac4 * (uint32_t)(x3 + 32768)) >> 15;

    b7 = ((uint32_t)(up - b3) * (uint32_t)(50000 >> _overSampling));

    *_value = (b7 < 0x80000000L) ? ((b7 << 1) / b4) : ((b7 / b4) << 1);
/*
    if (b7 < 0x80000000)
      *_value = (b7 << 1) / b4;
    else
      *_value = (b7 / b4) << 1;
*/
    x1 = (*_value >> 8) * (*_value >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * *_value) >> 16;

    *_value += (x1 + x2 + (int32_t) 3791) >> 4;
    
    rc = RESULT_IS_UNSIGNED_VALUE;

    // pressure /=100 for hPa
    // ltoaf(cBuffer, result, 2);
    // or pressure /=1 for Pa
  } else {
    // temperature /=10
    //*_value = result;
    rc = RESULT_IS_FLOAT_01_DIGIT;
  }


finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}
#endif

/*****************************************************************************************************************************
*
*  Return code of the called subroutine (based on sensor ID) for obtaining a metric of sensor.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success when SENS_READ_PRSS metric specified
*    - RESULT_IS_FLOAT_01_DIGIT    on success when SENS_READ_TEMP metric specified (BMP085/BMP180)
*    - RESULT_IS_FLOAT_02_DIGIT    on success when SENS_READ_TEMP metric specified (BMP280/BME280)
*    - RESULT_IS_FLOAT_QMN         on success when SENS_READ_HUMD metric specified (BMP280/BME280)
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified or unknown chip ID detected
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getBMPMetric(SoftwareTWI* _softTWI, uint8_t _i2cAddress, const uint8_t _overSampling, const uint8_t _filterCoef, const uint8_t _metric, int32_t* _value) {
  // Just supress warnings if preprocessor do not include some func calls to the source text
  __SUPPRESS_WARNING_UNUSED(_softTWI);
  __SUPPRESS_WARNING_UNUSED(_i2cAddress);
  __SUPPRESS_WARNING_UNUSED(_overSampling);
  __SUPPRESS_WARNING_UNUSED(_filterCoef);
  __SUPPRESS_WARNING_UNUSED(_metric);
  __SUPPRESS_WARNING_UNUSED(_value);

  int8_t rc = DEVICE_ERROR_NOT_SUPPORTED;
#if defined(FEATURE_BMP_ENABLE)
  uint8_t chipID; 

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish;  }

  // Taking Chip ID
  if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, BMP_REGISTER_CHIPID, &chipID, 0x01)) { rc = DEVICE_ERROR_TIMEOUT; goto finish; }

  switch (chipID) {
    // BMP085 and BMP180 have the same ID  
#ifdef SUPPORT_BMP180_INCLUDE
    case BMP180_CHIPID: 
      rc =  getBMP180Metric(_softTWI, _i2cAddress, _overSampling, _metric, _value);
      break;
#endif
#ifdef SUPPORT_BMP280_INCLUDE
    case BMP280_CHIPID_1: 
    case BMP280_CHIPID_2: 
    case BMP280_CHIPID_3: 
    // BME280 is BMP280 with additional humidity sensor
    case BME280_CHIPID:
      rc = getBME280Metric(_softTWI, _i2cAddress, _overSampling, _filterCoef, _metric, _value);
      break;
#endif
    // rc already is DEVICE_ERROR_NOT_SUPPORTED
    default:  
      break;

  }

  finish:
#endif // #if defined(FEATURE_BMP_ENABLE)
  return rc;
}
