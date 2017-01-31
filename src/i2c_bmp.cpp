#include "i2c_bmp.h"


/*****************************************************************************************************************************
*
*   Waiting for a timeout to detect the BMP sensor ready state
*
*   Returns: 
*     - true on sensor ready
*     - false on timeout
*
**************************************************************************************************************************** */
uint8_t waitToBMPReady(const uint8_t _i2cAddress, const int16_t _registerAddress, const int16_t _mask, const uint16_t _timeout)
{
  uint8_t value;
  uint32_t startTime;

  startTime = millis();
  
  while((millis() - startTime) < _timeout){
     readBytesFromi2C(_i2cAddress, _registerAddress, &value, 1);
     if (0 == (value & _mask)) return true;
     delay(10);  
  }
  return false;
}


/*****************************************************************************************************************************
*
*   Call the subroutine (based on sensor ID) for obtaining a metric of sensor 
*
*   Returns: 
*     - result code of the called subroutine 
*     - DEVICE_ERROR_CONNECT on connection error
*     - RESULT_IS_FAIL if unknown chip ID found
*
*****************************************************************************************************************************/
int8_t getBMPMetric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, const uint8_t _overSampling, const uint8_t _filterCoef, const uint8_t _metric, char *_dst)
{
  int8_t rc = RESULT_IS_FAIL;
  uint8_t chipID; 
  switch (_i2cAddress) {
    case BMP180_I2C_ADDRESS:
    //  BMP280_I2C_ADDRESS_2 == BMP180_I2C_ADDRESS
    case BMP280_I2C_ADDRESS_1:
      break;
    default:  
       _i2cAddress = BMP180_I2C_ADDRESS;
  }

  if (false == isI2CDeviceReady(_i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish;  }

  // Taking Chip ID
  // false == 0 == succes transmission
  if (false != readBytesFromi2C(_i2cAddress, BMP_REGISTER_CHIPID, &chipID, 1)) { rc = DEVICE_ERROR_TIMEOUT; goto finish; }
  
  switch (chipID) {
    // BMP085 and BMP180 have the same ID  
#ifdef SUPPORT_BMP180_INCLUDE
    case BMP180_CHIPID: 
       rc =  getBMP180Metric(_sdaPin, _sclPin, _i2cAddress, _overSampling, _metric, _dst);
       break;
#endif
#ifdef SUPPORT_BMP280_INCLUDE
    case BMP280_CHIPID_1: 
    case BMP280_CHIPID_2: 
    case BMP280_CHIPID_3: 
       rc = getBMP280Metric(_sdaPin, _sclPin, _i2cAddress, _overSampling, _filterCoef, _metric, _dst);
       break;
#endif
#ifdef SUPPORT_BME280_INCLUDE
    case BME280_CHIPID:
       // BME280 is BMP280 with additional humidity sensor
       rc = getBMP280Metric(_sdaPin, _sclPin, _i2cAddress, _overSampling, _filterCoef, _metric, _dst);
       break;
#endif
    default:  
       rc = RESULT_IS_FAIL;
  }

  finish:
  return rc;
}


/*****************************************************************************************************************************
*
*   Read specified metric's value of the BMP280/BME280 sensor, put it to output b\uffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if sensor do not ready to work
*
*****************************************************************************************************************************/
int8_t getBMP280Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, const uint8_t _overSampling, uint8_t _filterCoef, const uint8_t _metric, char* _dst)
{
  int8_t rc = DEVICE_ERROR_TIMEOUT;
  uint16_t dig_T1;
  int16_t  dig_T2, dig_T3;

  uint16_t dig_P1;
  int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

  int32_t adc, result, var1, var2, t_fine;
  uint8_t value[3];
  uint8_t i;

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
  if (!waitToBMPReady(_i2cAddress, BMP280_REGISTER_STATUS, BMP280_READY_MASK, BMP280_READY_TIMEOUT)) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value

  // The “ctrl_hum” register sets the humidity data acquisition options of the device. Changes to this register only become effective after a write operation to “ctrl_meas”.
  writeByteToI2C(_i2cAddress, BME280_REGISTER_CONTROLHUMID, BME280_STANDARD_OVERSAMP_HUMIDITY);
  // Set filter coefficient. While BMP280 used in Forced mode - BMP280_STANDBY_TIME_4000_MS can be replaced to any time. '0' - define SPI configuration. Not used.
  writeByteToI2C(_i2cAddress, BMP280_REGISTER_CONFIG, ((BMP280_STANDBY_TIME_4000_MS << 6)| (_filterCoef << 3) | 0 ));
  // BMP280_FORCED_MODE -> BME280 act like BMP180
  // We need "forced mode" to avoid mix-up bytes of data belonging to different measurements of various metrics on "normal mode"
  // Need to double read to take properly data (not from previous measure round)
  // use var as 'mode' variable
  var1 = (var1 << 6)| (var2 << 3) | BMP280_FORCED_MODE;
  for (i = 0; i < 2; i++) { 
    writeByteToI2C(_i2cAddress, BMP_REGISTER_CONTROL, var1);
    // We must wait to Bxxxx0xx0 value in "Status" register. It's most reliable way to detect end of conversion 
    if (!waitToBMPReady(_i2cAddress, BMP280_REGISTER_STATUS, BMP280_READY_MASK, BMP280_READY_TIMEOUT)) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value  
  }


  // Compensate temperature caculation

  // Read raw value  
  readBytesFromi2C(_i2cAddress, BMP280_REGISTER_TEMPDATA, value, 3);
  
  adc = ((int32_t) (((uint16_t) value[0] << 8) | (uint16_t) value[1]) << 4) | ((uint16_t) value[2] >> 4);
  
  // read calibration data 
  readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_T1, value, 2);
  dig_T1 = WireToU16LE(value);

  readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_T2, value, 2);
  dig_T2 = WireToS16LE(value);
                                                                  
  readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_T3, value, 2);
  dig_T3 = WireToS16LE(value);;


  var1 = ((((adc >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
  var2 = (((((adc >> 4) - ((int32_t) dig_T1)) * ((adc >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3)) >> 14;

  t_fine = var1 + var2;

  switch (_metric) {
    case SENS_READ_TEMP:
      // real temperature caculation
      result  = (t_fine * 5 + 128) >> 8;
      //  return T/100;    
      ltoaf(result, _dst, 2);
      break;

    case SENS_READ_PRSS:
      // read calibration data 
      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_P1, value, 2);
      dig_P1 = WireToU16LE(value);
      
      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_P2, value, 2);
      dig_P2 = WireToS16LE(value);
      
      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_P3, value, 2);
      dig_P3 = WireToS16LE(value);

      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_P4, value, 2);
      dig_P4 = WireToS16LE(value);
      
      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_P5, value, 2);
      dig_P5 = WireToS16LE(value);
      
      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_P6, value, 2);
      dig_P6 = WireToS16LE(value);
      
      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_P7, value, 2);
      dig_P7 = WireToS16LE(value);
      
      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_P8, value, 2);
      dig_P8 = WireToS16LE(value);
      
      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_DIG_P9, value, 2);
      dig_P9 = WireToS16LE(value);
 
      // Test value
      // adc = 415148 ( from BOSCH datasheet)
      // Read raw value  
      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_PRESSUREDATA, value, 3);
      adc = ((int32_t) (((uint16_t) value[0] << 8) | (uint16_t) value[1]) << 4) | ((uint16_t) value[2] >> 4);
   
      // Compensate pressure caculation
      var1 = (((int32_t)t_fine) >> 1) - (int32_t) 64000;
      var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t) dig_P6);
      var2 = var2 + ((var1 * ((int32_t) dig_P5)) << 1);
      var2 = (var2 >> 2) + (((int32_t) dig_P4) << 16);
      var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t) dig_P2) * var1) >> 1)) >> 18;
      var1 =((((32768 + var1)) * ((int32_t) dig_P1)) >> 15);

      // avoid exception caused by division by zero
      if (0 == var1) { 
        return 0; 
      }

      result = (((uint32_t)(((int32_t) 1048576) - adc)-(var2 >> 12))) * 3125;
       
      result = ((uint32_t) result < 0x80000000L) ? ((result << 1) / ((uint32_t) var1)) : ((result / (uint32_t) var1) * 2); 

/*
      if (result < 0x80000000) {
         result = (result << 1) / ((uint32_t) var1); 
      } else {
         result = (result / (uint32_t) var1) * 2; 
      }
*/
      var1 = (((int32_t) dig_P9) * ((int32_t)(((result >> 3) * (result >> 3)) >> 13))) >> 12;
      var2 = (((int32_t)(result >> 2)) * ((int32_t) dig_P8)) >> 13;
      result = (uint32_t)((int32_t) result + ((var1 + var2 + dig_P7) >> 4));

     // BOSCH on page 22 of datasheet say: "Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits)." 
     // 24674867 in Q24.8 is 96386 in whole part (24674867 >> 8) , and 51 in frac part (24674867 & B11111111) => 96386.51
     // But in code example use calculation: 24674867/256 = 96386.19 => 96386.2
     //
     // What way is right, BOSCH? 
     //
     ltoa(result, _dst, 10);
     break;
     
#ifdef SUPPORT_BME280_INCLUDE
    case SENS_READ_HUMD:
      uint8_t  dig_H1, dig_H3;
      int8_t   dig_H6;
      int16_t  dig_H2, dig_H4, dig_H5;

      //read calibration data

      readBytesFromi2C(_i2cAddress, BME280_REGISTER_DIG_H1, value, 1);
      dig_H1 = WireToU8(value);
  
      readBytesFromi2C(_i2cAddress, BME280_REGISTER_DIG_H2, value, 2);
      dig_H2 = WireToS16LE(value);

      readBytesFromi2C(_i2cAddress, BME280_REGISTER_DIG_H3, value, 1);
      dig_H3 = WireToU8(value);

      readBytesFromi2C(_i2cAddress, BME280_REGISTER_DIG_H4, value, 2);
      dig_H4 = (int16_t) (((uint16_t) value[0] << 4) | (value[1] & 0xF));

      readBytesFromi2C(_i2cAddress, BME280_REGISTER_DIG_H5, value, 2);
      dig_H5 = (int16_t) ((uint16_t) value[1] << 4) | (value[0] >> 4);

      readBytesFromi2C(_i2cAddress, BME280_REGISTER_DIG_H6, value, 1);
      dig_H6 = WireToS8(value);

      // Read raw value  
      readBytesFromi2C(_i2cAddress, BME280_REGISTER_HUMIDDATA, value, 2);
      // adc take wrong value due overflow if value[] no cast to (uint16_t) 
      adc = (int32_t) WireToU16(value);

      // Compensate humidity caculation
      var1 = (t_fine -  ((int32_t) 76800));
      var1 = (((((adc << 14) - (((int32_t) dig_H4) << 20) - (((int32_t) dig_H5) * var1)) +
             ((int32_t) 16384)) >> 15) * (((((((var1 * ((int32_t) dig_H6)) >> 10) * (((var1 * 
             ((int32_t) dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
             ((int32_t) dig_H2) + 8192) >> 14));
      var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
      var1 = (var1 < 0 ? 0 : var1);
      var1 = (var1 > 419430400 ? 419430400 : var1);
      result = (uint32_t)(var1 >> 12);
      // Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
      //	 Output value of “47445” represents 47445/1024 = 46.333 %RH
      //Serial.print("H: "); Serial.println(result);

      qtoaf(result, _dst, 10);
#endif        
  }  // switch (_metric)

  rc = RESULT_IN_BUFFER;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;
}

/*****************************************************************************************************************************
*
*   Read specified metric's value of the BMP180/BMP085 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if sensor do not ready to work
*
*****************************************************************************************************************************/
int8_t getBMP180Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _overSampling, const uint8_t _metric, char *_dst)
{
  int8_t rc = DEVICE_ERROR_TIMEOUT;
  //uint8_t msb, lsb, xlsb;
  uint8_t value[3];
  // Calibration values
  int16_t ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t b1, b2, mc, md; // mb - not used 
  uint16_t ut;
  uint32_t up;
  int32_t x1, x2, x3, b3, b5, b6, result;
  uint32_t b4, b7;


  /* read calibration data */
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_AC1, value, 2);
  ac1 = WireToS16(value);
  
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_AC2, value, 2);
  ac2 = WireToS16(value);
  
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_AC3, value, 2);
  ac3 = WireToS16(value);
  
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_AC4, value, 2);
  ac4 = WireToU16(value);
  
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_AC5, value, 2);
  ac5 = WireToU16(value);
  
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_AC6, value, 2);
  ac6 = WireToU16(value);
  
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_B1, value, 2);
  b1 = WireToS16(value);
  
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_B2, value, 2);
  b2 = WireToS16(value);
  
  // Not used in calculation (see datasheet)
  //readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_MB, value, 2);
  //mb = WireToS16(value);

  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_MC, value, 2);
  mc = WireToS16(value);

  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_MD, value, 2);
  md = WireToS16(value);

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
  if (!waitToBMPReady(_i2cAddress, BMP_REGISTER_CONTROL, BMP180_READY_MASK, BMP180_READY_TIMEOUT)) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value
  // ******** Get Temperature ********
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  writeByteToI2C(_i2cAddress, BMP_REGISTER_CONTROL, BMP180_CMD_READTEMP);

  // Wait at least 4.5ms
  //delay(5);

  // We must wait for 50ms to Bxx0xxxxx value in "Control" register to know about end of conversion
  if (!waitToBMPReady(_i2cAddress, BMP_REGISTER_CONTROL, BMP180_READY_MASK, BMP180_READY_TIMEOUT)) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value
  // Read two bytes from registers 0xF6 and 0xF7
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_TEMPDATA, value, 2);
  ut = WireToU16(value);

  x1 = (((int32_t) ut - (int32_t) ac6) * (int32_t) ac5) >> 15;
  x2 = ((int32_t) mc << 11) / (x1 + (int32_t) md);
  b5 = x1 + x2;
  result = ((b5 + 8) >> 4);

  if (SENS_READ_PRSS == _metric) {
    // ******** Get Pressure ********

    // Calculate pressure given up
    // calibration values must be known
    // b5 is also required so get temperature procedure must be called first.
    // Value returned will be pressure in units of Pa.

    // Write 0x34+(_oversampling<<6) into register 0xF4
    // Request a pressure reading w/ oversampling setting
    writeByteToI2C(_i2cAddress, BMP_REGISTER_CONTROL, BMP180_CMD_READPRESSURE + (_overSampling << 6));

    // Wait for conversion, delay time dependent on _oversampling
    delay(2 + (3 << _overSampling));

    // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
    readBytesFromi2C(_i2cAddress, BMP180_REGISTER_PRESSUREDATA, value, 3);
    up = (((uint32_t) value[0] << 16) | ((uint32_t) value[1] << 8) | (uint32_t) value[2]) >> (8 - _overSampling);

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

    result = (b7 < 0x80000000L) ? ((b7 << 1) / b4) : ((b7 / b4) << 1);
/*
    if (b7 < 0x80000000)
      result = (b7 << 1) / b4;
    else
      result = (b7 / b4) << 1;
*/
    x1 = (result >> 8) * (result >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * result) >> 16;

    result += (x1 + x2 + (int32_t) 3791) >> 4;

    // pressure /=100 for hPa
    // ltoaf(cBuffer, result, 2);
    // or pressure /=1 for Pa
    ltoa(result, _dst, 10);
  } else {
    // temperature /=10
    ltoaf(result, _dst, 1);
  }

  rc = RESULT_IN_BUFFER;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}

