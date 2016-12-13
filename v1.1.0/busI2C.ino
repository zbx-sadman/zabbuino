/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                      COMMON I2C SECTION
*/

#define WireToU8(_source)  ((uint8_t) _source[0])
#define WireToS8(_source)  ((int8_t) _source[0])

#define WireToU16(_source)  ((uint16_t) ( ((uint16_t) _source[0] << 8)| _source[1]))
#define WireToS16(_source)  ((int16_t) ( ((uint16_t) _source[0] << 8)| _source[1]))

#define WireToU16LE(_source)  ((uint16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))
#define WireToS16LE(_source)  ((int16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))

#define WireToU24(_source)  ((uint32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))
#define WireToS24(_source)  ((int32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))

/* ****************************************************************************************************************************
*
*   Scan I2C bus and print to ethernet client addresses of all detected devices 
*
**************************************************************************************************************************** */
int32_t scanI2C()
{
  int8_t i2cAddress, numDevices;

  for(i2cAddress = 0x01; i2cAddress < 0x7F; i2cAddress++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(i2cAddress);
    // 0:success
    // 1:data too long to fit in transmit buffer
    // 2:received NACK on transmit of address
    // 3:received NACK on transmit of data
    // 4:other error
    if (0 == Wire.endTransmission(true)) {
      numDevices++;
      ethClient.print("0x");
      if (i2cAddress < 0x0F){ ethClient.print("0"); }
      ethClient.println(i2cAddress, HEX);
    }
  } 
  return (numDevices ? RESULT_IS_PRINTED : RESULT_IS_FAIL);
}

/* ****************************************************************************************************************************
*
*  Write 1 byte to I2C device register or just to device
*
**************************************************************************************************************************** */
uint8_t writeByteToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _data)
{
  Wire.beginTransmission(_i2cAddress); // start transmission to device 
  // registerAddress is 0x00 and above ?
  if (I2C_NO_REG_SPECIFIED < _registerAddress) {
     Wire.write(_registerAddress); // sends register address to be written
  }
  Wire.write(_data);  // write data
  return Wire.endTransmission(true); // end transmission
}

/* ****************************************************************************************************************************
*
*  Reads N bytes from device's register (or not) over I2C
*
**************************************************************************************************************************** */
uint8_t readBytesFromi2C(const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t _buff[], const uint8_t length)
{
    if (!length) return false;
    Wire.beginTransmission(_i2cAddress); 	// Adress + WRITE (0)
    if (I2C_NO_REG_SPECIFIED < _registerAddress) {
       Wire.write(_registerAddress);
       Wire.endTransmission(false); 		// No Stop Condition, for repeated Talk
    }

    uint8_t i = 0;
    Wire.requestFrom(_i2cAddress, length); 	// Address + READ (1)
    while(i < length) {
      _buff[i] = Wire.read();
      i++;
    }

    return Wire.endTransmission(true); 		// Stop Condition
};


/* ****************************************************************************************************************************
*
*  Ping I2C client
*
**************************************************************************************************************************** */
uint8_t inline isI2CDeviceReady(uint8_t _i2cAddress)
{
  //
  Wire.beginTransmission(_i2cAddress);
  return (0 == Wire.endTransmission(true));
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           SHT2x SECTION
*/

#define SHT2X_I2C_ADDRESS                                       0x40
#define SHT2X_CMD_GETTEMP_HOLD                                  0xE3
#define SHT2X_CMD_GETHUMD_HOLD                                  0xE5

uint16_t getRawDataFromSHT2X(const uint8_t _i2cAddress, const uint8_t _command)
{
    uint16_t result;
    Wire.beginTransmission(_i2cAddress);
    Wire.write(_command);
    delay(100);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)_i2cAddress, (uint8_t) 3);
    uint32_t timeout = millis() + 300;       // Don't hang here for more than 300ms
    while (Wire.available() < 3) {
      if ((millis() - timeout) > 0) {
            return 0;
        }
    }
    //Store the result
    result = Wire.read() << 8;
    result += Wire.read();
    result &= ~0x0003;   // clear two low bits (status bits)
    //Clear the final byte from the buffer
    Wire.read();
    return result;
}

int32_t getSHT2XMetric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, const uint8_t _metric, char* _outBuffer) 
{
  int32_t result = 0;
  uint16_t rawData; 
  
  switch (_i2cAddress) {
    case SHT2X_I2C_ADDRESS:
      break;
    default:  
       _i2cAddress = SHT2X_I2C_ADDRESS;
  }

  if (!isI2CDeviceReady(_i2cAddress)) {return DEVICE_ERROR_CONNECT; }
  
  switch (_metric) {
    case SENS_READ_TEMP:
      rawData = getRawDataFromSHT2X(_i2cAddress, SHT2X_CMD_GETTEMP_HOLD);
      // Default humidity  resolution - 14 bit => 0.01C
      // all part of equation must be multiplied by 100 to get 2-digit fract part by ltoaf() 
      // H_real = 100 * -6 + (100 * 125 * H_raw) / (2^16)
      result = (((uint32_t) rawData * 17572) >> 16) - 4685;
      break;
      
    case SENS_READ_HUMD:
      rawData = getRawDataFromSHT2X(_i2cAddress, SHT2X_CMD_GETHUMD_HOLD);
      // Default humidity  resolution - 12 bit => 0.04%RH
      // all part of equation must be multiplied by 100 to get 2-digit fract part by ltoaf() 
      // H_real = 100 * -6 + (100 * 125 * H_raw) / (2^16)
      if (0 < rawData) { result = (((uint32_t) rawData * 100 * 125) >> 16) - 600; }
  }
  // result /=100
  ltoaf(result, _outBuffer, 2);
  return RESULT_IN_BUFFER;  

}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     BMP SECTION
*/

#define BMP180_I2C_ADDRESS                                      0x77  // I2C address of BMP085/180 sensor

#define BMP280_I2C_ADDRESS_1                                    0x76  // I2C address of BMP280/BME280 sensor
#define BMP280_I2C_ADDRESS_2                                    0x77


#define BMP_REGISTER_CHIPID                                     0xD0

#define BMP_REGISTER_RESET                                      0xE0 // for all BMP or not?
#define BMP_REGISTER_CONTROL                                    0xF4 


#define BMP085_CHIPID                                           0x55
#define BMP180_CHIPID                                           0x55
#define BMP280_CHIPID_1                                         0x56 // Only for engeneering samples (BMP180 datasheet, page 24)
#define BMP280_CHIPID_2                                         0x57 // Only for engeneering samples
#define BMP280_CHIPID_3                                         0x58 // Mass production ID

#define BME280_CHIPID                                           0x60 // 

#define BMP180_ULTRALOWPOWER                                    0x00
#define BMP180_STANDARD                                         0x01
#define BMP180_HIGHRES                                          0x02
#define BMP180_ULTRAHIGHRES                                     0x03

#define BMP180_REGISTER_CAL_AC1                                 0xAA  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC2                                 0xAC  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC3                                 0xAE  // R   Calibration data (16 bits)    
#define BMP180_REGISTER_CAL_AC4                                 0xB0  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC5                                 0xB2  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC6                                 0xB4  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_B1                                  0xB6  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_B2                                  0xB8  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_MB                                  0xBA  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_MC                                  0xBC  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_MD                                  0xBE  // R   Calibration data (16 bits)

#define BMP180_REGISTER_TEMPDATA                                0xF6
#define BMP180_REGISTER_PRESSUREDATA                            0xF6
#define BMP180_CMD_READTEMP                                     0x2E
#define BMP180_CMD_READPRESSURE                                 0x34

#define BMP180_READY_MASK                                       0x20  // Sco (register F4h <5>): Start of conversion. 
                                                                      // The value of this bit stays “1” during conversion and is reset to “0” after conversion is 
                                                                      // complete (data registers are filled). 

#define BMP280_REGISTER_DIG_T1                                  0x88
#define BMP280_REGISTER_DIG_T2                                  0x8A
#define BMP280_REGISTER_DIG_T3                                  0x8C

#define BMP280_REGISTER_DIG_P1                                  0x8E
#define BMP280_REGISTER_DIG_P2                                  0x90
#define BMP280_REGISTER_DIG_P3                                  0x92
#define BMP280_REGISTER_DIG_P4                                  0x94
#define BMP280_REGISTER_DIG_P5                                  0x96
#define BMP280_REGISTER_DIG_P6                                  0x98
#define BMP280_REGISTER_DIG_P7                                  0x9A
#define BMP280_REGISTER_DIG_P8                                  0x9C
#define BMP280_REGISTER_DIG_P9                                  0x9E

#define BMP280_REGISTER_CAL26                                   0xE1  // R calibration stored in 0xE1-0xF0

#define BMP280_REGISTER_STATUS                                  0xF3
#define BMP280_REGISTER_CONTROL                                 0xF4
#define BMP280_REGISTER_CONFIG                                  0xF5
#define BMP280_REGISTER_PRESSUREDATA                            0xF7
#define BMP280_REGISTER_TEMPDATA                                0xFA

#define BMP280_OVERSAMP_SKIPPED          			0x00
#define BMP280_OVERSAMP_1X               			0x01
#define BMP280_OVERSAMP_2X               			0x02
#define BMP280_OVERSAMP_4X               			0x03
#define BMP280_OVERSAMP_8X               			0x04
#define BMP280_OVERSAMP_16X              			0x05

#define BMP280_ULTRALOWPOWER            			0x00
#define BMP280_LOWPOWER	                 			0x01
#define BMP280_STANDARD      	                		0x02
#define BMP280_HIGHRES          		        	0x03
#define BMP280_ULTRAHIGHRES    			                0x04

#define BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE          	BMP280_OVERSAMP_1X
#define BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE       	BMP280_OVERSAMP_1X

#define BMP280_LOWPOWER_OVERSAMP_PRESSURE	         	BMP280_OVERSAMP_2X
#define BMP280_LOWPOWER_OVERSAMP_TEMPERATURE	         	BMP280_OVERSAMP_1X

#define BMP280_STANDARD_OVERSAMP_PRESSURE     	                BMP280_OVERSAMP_4X
#define BMP280_STANDARD_OVERSAMP_TEMPERATURE  	                BMP280_OVERSAMP_1X

#define BMP280_HIGHRES_OVERSAMP_PRESSURE         	        BMP280_OVERSAMP_8X
#define BMP280_HIGHRES_OVERSAMP_TEMPERATURE      	        BMP280_OVERSAMP_1X

#define BMP280_ULTRAHIGHRES_OVERSAMP_PRESSURE       	        BMP280_OVERSAMP_16X
#define BMP280_ULTRAHIGHRES_OVERSAMP_TEMPERATURE    	        BMP280_OVERSAMP_2X

#define BMP280_SLEEP_MODE                    			0x00
#define BMP280_FORCED_MODE                   			0x01
#define BMP280_NORMAL_MODE                   			0x03

#define BMP280_FILTER_COEFF_OFF                                 0x00
#define BMP280_FILTER_COEFF_2                                   0x01
#define BMP280_FILTER_COEFF_4                                   0x02
#define BMP280_FILTER_COEFF_8                                   0x03
#define BMP280_FILTER_COEFF_16                                  0x04

#define BMP280_STANDBY_TIME_1_MS                                0x00
#define BMP280_STANDBY_TIME_63_MS                               0x01
#define BMP280_STANDBY_TIME_125_MS                              0x02
#define BMP280_STANDBY_TIME_250_MS                              0x03
#define BMP280_STANDBY_TIME_500_MS                              0x04
#define BMP280_STANDBY_TIME_1000_MS                             0x05
#define BMP280_STANDBY_TIME_2000_MS                             0x06
#define BMP280_STANDBY_TIME_4000_MS                             0x07

#define BMP280_READY_MASK                                       0x09 // Byte 0 + Byte 3 must be equial 0 if BMP280 do not busy

// BME280 additional registers and constants
#define BME280_STANDARD_OVERSAMP_HUMIDITY 	                BMP280_OVERSAMP_1X
#define BME280_REGISTER_DIG_H1                                  0xA1
#define BME280_REGISTER_DIG_H2                                  0xE1
#define BME280_REGISTER_DIG_H3                                  0xE3
#define BME280_REGISTER_DIG_H4                                  0xE4
#define BME280_REGISTER_DIG_H5                                  0xE5
#define BME280_REGISTER_DIG_H6                                  0xE7

#define BME280_REGISTER_CONTROLHUMID                            0xF2
#define BME280_REGISTER_HUMIDDATA                               0xFD

#define T_INIT_MAX						20   // 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX					37   // 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX					10   // 10/16 = 0.625 ms
#define T_SETUP_HUMIDITY_MAX					10   // 10/16 = 0.625 ms


uint8_t waitToBMPReady(const uint8_t _i2cAddress, const int16_t _registerAddress, const int16_t _mask, const uint16_t _timeout)
{
  uint8_t value[1], i;
  uint32_t nowTime;

  nowTime = millis();
  
  while((millis() - nowTime) < _timeout){
     readBytesFromi2C(_i2cAddress, _registerAddress, value, 1);
     if (0 == (value[0] & _mask)) return true;
     delay(10);  
  }
}

int32_t getBMPMetric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, const uint8_t _overSampling, const uint8_t _filterCoef, const uint8_t _metric, char* _outBuffer)
{
  uint8_t chipID; 

  switch (_i2cAddress) {
    case BMP180_I2C_ADDRESS:
    //  BMP280_I2C_ADDRESS_2 == BMP180_I2C_ADDRESS
    case BMP280_I2C_ADDRESS_1:
      break;
    default:  
       _i2cAddress = BMP180_I2C_ADDRESS;
  }


  if (!isI2CDeviceReady(_i2cAddress)) {return DEVICE_ERROR_CONNECT; }

  // Taking Chip ID
  Wire.beginTransmission(_i2cAddress);
  Wire.write(BMP_REGISTER_CHIPID);
  Wire.endTransmission();

  Wire.beginTransmission(_i2cAddress);
  Wire.requestFrom((uint8_t) _i2cAddress, (uint8_t) 1);
  chipID = Wire.read();
  Wire.endTransmission();
  
  switch (chipID) {
    // BMP085 and BMP180 have the same ID  
#ifdef SUPPORT_BMP180_INCLUDE
    case BMP180_CHIPID: 
       getBMP180Metric(_sdaPin, _sclPin, _i2cAddress, _overSampling, _metric, _outBuffer);
       break;
#endif
#ifdef SUPPORT_BMP280_INCLUDE
    case BMP280_CHIPID_1: 
    case BMP280_CHIPID_2: 
    case BMP280_CHIPID_3: 
       getBMP280Metric(_sdaPin, _sclPin, _i2cAddress, _overSampling, _filterCoef, _metric, _outBuffer);
       break;
#endif
#ifdef SUPPORT_BME280_INCLUDE
    case BME280_CHIPID:
       // BME280 is BMP280 with additional humidity sensor
       getBMP280Metric(_sdaPin, _sclPin, _i2cAddress, _overSampling, _filterCoef, _metric, _outBuffer);
       break;
#endif
    default:  
       return DEVICE_ERROR_CONNECT;  
  }
  return RESULT_IN_BUFFER;  
}


/* ****************************************************************************************************************************
*
*  Read temperature or pressure from digital sensor BMP280
*
**************************************************************************************************************************** */
int32_t getBMP280Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, const uint8_t _overSampling, uint8_t _filterCoef, const uint8_t _metric, char* _outBuffer)
{
  uint16_t dig_T1;
  int16_t  dig_T2, dig_T3;

  uint16_t dig_P1;
  int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

  uint8_t dig_H1, dig_H3;
  int8_t dig_H6;
  int16_t  dig_H2, dig_H4, dig_H5;

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

  // var1 is oversamp_temperature
  // var2 is oversamp_pressure
  // Use var1 as temp variable for store conversion delay time
  // BME280 calculation, BMP280 don't take humidity conversion time into account 
  //   var1 = (T_INIT_MAX + 
  //           T_MEASURE_PER_OSRS_MAX * (((1 << var1) >> 1)
  //           + ((1 << var2) >> 1) 
  //           + ((1 << BME280_STANDARD_OVERSAMP_HUMIDITY) >> 1))
  //           + ((var2 > 0) ? T_SETUP_PRESSURE_MAX : 0) 
  //           + ((BME280_STANDARD_OVERSAMP_HUMIDITY > 0) ? T_SETUP_HUMIDITY_MAX : 0) + 15) / 16;
  
  // We must wait to Bxxxx0xx0 value in "Status" register
  if (false == waitToBMPReady(_i2cAddress, BMP280_REGISTER_STATUS, BMP280_READY_MASK, 50)) {
       return DEVICE_ERROR_TIMEOUT;
  }

  // The “ctrl_hum” register sets the humidity data acquisition options of the device. Changes to this register only become effective after a write operation to “ctrl_meas”.
  writeByteToI2C(_i2cAddress, BME280_REGISTER_CONTROLHUMID, BME280_STANDARD_OVERSAMP_HUMIDITY);
  // Set filter coefficient. While BMP280 used in Forced mode - BMP280_STANDBY_TIME_4000_MS can be replaced to any time. '0' - define SPI configuration. Not used.
  writeByteToI2C(_i2cAddress, BMP280_REGISTER_CONFIG, ((BMP280_STANDBY_TIME_4000_MS << 6)| (_filterCoef << 3) | 0 ));
  // BMP280_FORCED_MODE -> BME280 act like BMP180
  // We need "forced mode" to avoid mix-up bytes of data belonging to different measurements of various metrics on "normal mode"
  // Need to double read to take properly data (not from previous measure round)
  // use var as mode variable
  var1 = (var1 << 6)| (var2 << 3) | BMP280_FORCED_MODE;
  for (i = 0; i < 2; i++) { 
    writeByteToI2C(_i2cAddress, BMP_REGISTER_CONTROL, var1);
    // We must wait to Bxxxx0xx0 value in "Status" register. It's most reliable way to detect end of conversion 
    if (false == waitToBMPReady(_i2cAddress, BMP280_REGISTER_STATUS, BMP280_READY_MASK, 50)) {
       return DEVICE_ERROR_TIMEOUT;
    }
  }

  // Compensate temperature caculation

  // Read raw value  
  readBytesFromi2C(_i2cAddress, BMP280_REGISTER_TEMPDATA, value, 3);
  
  adc = ((int32_t) (((uint16_t) value[0] << 8) | (uint16_t) value[1]) << 4) | ((uint16_t) value[2] >> 4);
  //adc = WireToS24(value);
  //adc >>= 4;

  
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
      ltoaf(result, _outBuffer, 2);
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
      // adc = 415148 ( from BOSH datasheet)
      // Read raw value  
      readBytesFromi2C(_i2cAddress, BMP280_REGISTER_PRESSUREDATA, value, 3);
      adc = ((int32_t) (((uint16_t) value[0] << 8) | (uint16_t) value[1]) << 4) | ((uint16_t) value[2] >> 4);
//      adc = WireToS24(value);
//      adc >>= 4;
   
      // Compensate pressure caculation
      var1 = (((int32_t)t_fine) >> 1) - (int32_t) 64000;
      var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t) dig_P6);
      var2 = var2 + ((var1 * ((int32_t) dig_P5)) << 1);
      var2 = (var2 >> 2) + (((int32_t) dig_P4) << 16);
      var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t) dig_P2) * var1) >> 1)) >> 18;
      var1 =((((32768 + var1)) * ((int32_t) dig_P1)) >> 15);

      // avoid exception caused by division by zero
      if (var1 == 0) { 
        return 0; 
      }

      result = (((uint32_t)(((int32_t) 1048576) - adc)-(var2 >> 12))) * 3125;
      if (result < 0x80000000) {
         result = (result << 1) / ((uint32_t) var1); 
      } else {
         result = (result / (uint32_t) var1) * 2; 
      }

      var1 = (((int32_t) dig_P9) * ((int32_t)(((result >> 3) * (result >> 3)) >> 13))) >> 12;
      var2 = (((int32_t)(result >> 2)) * ((int32_t) dig_P8)) >> 13;
      result = (uint32_t)((int32_t) result + ((var1 + var2 + dig_P7) >> 4));

     // BOSH on page 22 of datasheet say: "Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits)." 
     // 24674867 in Q24.8 is 96386 in whole part (24674867 >> 8) , and 51 in frac part (24674867 & B11111111) => 96386.51
     // But in code example use calculation: 24674867/256 = 96386.19 => 96386.2
     //
     // What way is right, BOSCH? 
     //
     ltoa(result, _outBuffer, 10);
     break;
     
#ifdef SUPPORT_BME280_INCLUDE
    case SENS_READ_HUMD:
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

      qtoaf(result, _outBuffer, 10);
#endif        
}
  return RESULT_IN_BUFFER;
}

/* ****************************************************************************************************************************
*
*  Read temperature or pressure from digital sensor BMP180(BMP085)
*
**************************************************************************************************************************** */
int32_t getBMP180Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _overSampling, const uint8_t _metric, char* _outBuffer)
{
  // Calibration values
  int16_t ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t b1, b2, mb, mc, md;
  uint16_t ut;
  uint32_t up;
  int32_t x1, x2, x3, b3, b5, b6, result;
  uint32_t b4, b7;
  uint8_t msb, lsb, xlsb;
  uint8_t value[3];


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
  
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_CAL_MB, value, 2);
  mb = WireToS16(value);

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

  // We must wait to Bxx0xxxxx value in "Control" register to know that device is not busy
  if (false == waitToBMPReady(_i2cAddress, BMP_REGISTER_CONTROL, BMP180_READY_MASK, 50)) {
       return DEVICE_ERROR_TIMEOUT;
  }
  // ******** Get Temperature ********
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  writeByteToI2C(_i2cAddress, BMP_REGISTER_CONTROL, BMP180_CMD_READTEMP);

  // Wait at least 4.5ms
  //delay(5);

  // We must wait to Bxx0xxxxx value in "Control" register to know about end of conversion
  if (false == waitToBMPReady(_i2cAddress, BMP_REGISTER_CONTROL, BMP180_READY_MASK, 50)) {
       return DEVICE_ERROR_TIMEOUT;
  }
  // Read two bytes from registers 0xF6 and 0xF7
  readBytesFromi2C(_i2cAddress, BMP180_REGISTER_TEMPDATA, value, 2);
  ut = WireToU16(value);

  x1 = (((int32_t) ut - (int32_t) ac6) * (int32_t) ac5) >> 15;
  x2 = ((int32_t) mc << 11) / (x1 + (int32_t) md);
  b5 = x1 + x2;
  result = ((b5 + 8) >> 4);

  if ( SENS_READ_PRSS == _metric)
  {
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

    if (b7 < 0x80000000)
      result = (b7 << 1) / b4;
    else
      result = (b7 / b4) << 1;

    x1 = (result >> 8) * (result >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * result) >> 16;

    result += (x1 + x2 + (int32_t) 3791) >> 4;

    // pressure /=100 for hPa
    // ltoaf(cBuffer, result, 2);
    // or pressure /=1 for Pa
    ltoa(result, _outBuffer, 10);
  } else {
    // temperature /=10
    ltoaf(result, _outBuffer, 1);
  }

  return RESULT_IN_BUFFER;
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     LCD SECTION


Original code on: https://github.com/marcoschwartz/LiquidCrystal_I2C
version 1.1.2 is used


*/
// Some pin mappings not used at presently
/* LCD functional pin             ===>                   PCF8574 port (bit # in byte which send to I2C expander) */

#define LCD_RS                                                  0      // P0 (pin 4) on PCF8574 - expander pin #4  (used)
#define LCD_RW                                                  1      // P1 (pin 5) on PCF8574 - expander pin #5
#define LCD_E                                                   2      // P2 (pin 6) on PCF8574 - expander pin #6  (used)
#define LCD_BL                                                  3      // P3 (pin 7) on PCF8574 - expander pin #7  (used)

#define LCD_D4                                                  4      // P4 (pin 9) on PCF8574  - expander pin #11
#define LCD_D5                                                  5      // P5 (pin 10) on PCF8574 - expander pin #12
#define LCD_D6                                                  6      // P6 (pin 11) on PCF8574 - expander pin #13
#define LCD_D7                                                  7      // P7 (pin 12) on PCF8574 - expander pin #14

// LCD types.  1602 => 16 chars * 2 rows
#define LCD_TYPE_801                                            801
#define LCD_TYPE_1601                                           1601
                            
#define LCD_TYPE_802                                            802 
#define LCD_TYPE_1202                                           1202 
#define LCD_TYPE_1602                                           1602 
#define LCD_TYPE_2002                                           2002
#define LCD_TYPE_2402                                           2402
#define LCD_TYPE_4002                                           4002

#define LCD_TYPE_1604                                           1604
#define LCD_TYPE_2004                                           2004
#define LCD_TYPE_4004                                           4004

// LCD control codes
#define LCD_TAB_SIZE                                            0x04   // 4 space
#define LCD_BLINK_DUTY_CYCLE                                    250    // 250 ms
#define LCD_BLINK_TIMES                                         0x04   // in cycle of 4 times - 2 off state & 2 on state. Need use even numbers for save previous backlight state on cycle finish

// HD44780-compatible commands
#define LCD_CMD_BACKLIGHT_BLINK                                 0x03   // ASCII 03 - backlight blink
#define LCD_CMD_HT                                              0x09   // ASCII 09 - horizontal tabulation
#define LCD_CMD_LF                                              0x0A   // ASCII 10 - line feed

#define LCD_CMD_DISPLAYOFF                                      0x00
#define LCD_CMD_CLEARDISPLAY                                    0x01
#define LCD_CMD_RETURNHOME                                      0x02
#define LCD_CMD_ENTRYMODE_RIGHTTOLEFT                           0x04
#define LCD_CMD_ENTRYMODE_RIGHTTOLEFT_SCREENSHIFT               0x05
#define LCD_CMD_ENTRYMODE_LEFTTORIGHT                           0x06
#define LCD_CMD_ENTRYMODE_LEFTTORIGHT_SCREENSHIFT               0x07
#define LCD_CMD_BLANKSCREEN                                     0x08
#define LCD_CMD_CURSOROFF                                       0x0C
#define LCD_CMD_UNDERLINECURSORON                               0x0E
#define LCD_CMD_BLINKINGBLOCKCURSORON                           0x0F
#define LCD_CMD_CURSORMOVELEFT                                  0x10
#define LCD_CMD_CURSORMOVERIGHT                                 0x14
#define LCD_CMD_SCREENSHIFTLEFT                                 0x18
#define LCD_CMD_SCREENSHIFTRIGHT                                0x1E


#define LCD_DISPLAYCONTROL                                      0x08
#define LCD_CURSORSHIFT                                         0x10
#define LCD_FUNCTIONSET                                         0x20
#define LCD_SETDDRAMADDR                                        0x80

// flags for display on/off control
#define LCD_DISPLAYON                                           0x04
#define LCD_DISPLAYOFF                                          0x00
#define LCD_CURSORON                                            0x02
#define LCD_CURSOROFF                                           0x00
#define LCD_BLINKON                                             0x01
#define LCD_BLINKOFF                                            0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE                                         0x08
#define LCD_CURSORMOVE                                          0x00
#define LCD_MOVERIGHT                                           0x04
#define LCD_MOVELEFT                                            0x00

// flags for function set
#define LCD_8BITMODE                                            0x10
#define LCD_4BITMODE                                            0x00
#define LCD_2LINE                                               0x08
#define LCD_1LINE                                               0x00


void sendToLCD(const uint8_t _i2cAddress, const uint8_t _data, const uint8_t _mode)
{
  // Send high bit
  write4bitsToLCD(_i2cAddress, (_data & 0xF0) | _mode);
  // Send low bit
  write4bitsToLCD(_i2cAddress, ((_data << 4) & 0xF0) | _mode);
}

void write4bitsToLCD(const uint8_t _i2cAddress, const uint8_t _data) 
{
  writeByteToI2C(_i2cAddress, I2C_NO_REG_SPECIFIED, _data);
  pulseEnableOnLCD(_i2cAddress, _data);
}

void pulseEnableOnLCD(const uint8_t _i2cAddress, const uint8_t _data)
{
  writeByteToI2C(_i2cAddress, I2C_NO_REG_SPECIFIED, _data | _BV(LCD_E));	// 'Enable' high
  delayMicroseconds(1);		                        // enable pulse must be >450ns
  writeByteToI2C(_i2cAddress, I2C_NO_REG_SPECIFIED, _data & ~_BV(LCD_E));	// 'Enable' low
  delayMicroseconds(50);	                        // commands need > 37us to settle
} 

int32_t printToPCF8574LCD(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _lcdBacklight, const uint16_t _lcdType, const char *_data)
{
  uint8_t displayFunction, lastLine, currLine, currChar, i;
  uint8_t rowOffsets[] = { 0x00, 0x40, 0x14, 0x54 };

  if (!isI2CDeviceReady(_i2cAddress)) {return DEVICE_ERROR_CONNECT; }

  switch (_lcdType) {
   case LCD_TYPE_1602:
     displayFunction = LCD_2LINE;
     // 0, 1 = 2 line
     lastLine = 1;
     break;
   case LCD_TYPE_2002:
     displayFunction = LCD_2LINE;
     // 0, 1 = 2 line
     lastLine = 1;
     break;
   case LCD_TYPE_2004:
     // ???
     displayFunction = LCD_2LINE;
     // 0, 1, 2, 3 = 4 line
     lastLine = 3;
     break;
   default:
     return false;
  }

  _lcdBacklight = _lcdBacklight ? _BV(LCD_BL) : 0;

  // just tooggle backlight and go back if no data given 
  writeByteToI2C(_i2cAddress, I2C_NO_REG_SPECIFIED, _lcdBacklight);
  if (! *_data) {return RESULT_IS_OK; }
 
  if (!haveHexPrefix(_data)) {return RESULT_IS_FAIL;}
  _data+=2;

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  delay(50); 

  // Now we pull both RS and R/W low to begin commands
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46
	
  // we start in 8bit mode, try to set 4 bit mode
  write4bitsToLCD(_i2cAddress, (0x03 << 4) | _lcdBacklight);
  delayMicroseconds(4500); // wait min 4.1ms
   
  // second try
  write4bitsToLCD(_i2cAddress, (0x03 << 4) | _lcdBacklight);
  delayMicroseconds(4500); // wait min 4.1ms
   
  // third go!
  write4bitsToLCD(_i2cAddress, (0x03 << 4) | _lcdBacklight);
  delayMicroseconds(150);
   
  // finally, set to 4-bit interface
  write4bitsToLCD(_i2cAddress, (0x02 << 4) | _lcdBacklight);

  // set # lines, font size, etc.
  sendToLCD(_i2cAddress, (LCD_FUNCTIONSET | displayFunction), 0 | _lcdBacklight);
  sendToLCD(_i2cAddress, (LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF), 0 | _lcdBacklight);

  // Always begin from 0,0
  currLine = 0; 
  while(*_data && *(_data+1)) {
    // restore full byte (ASCII char) from HEX nibbles
    currChar = (htod(*_data) << 4) + htod(*(_data+1));
    if (currChar > 0x7F && currChar < 0x9F) {
     sendToLCD(_i2cAddress, LCD_SETDDRAMADDR | ((currChar - 0x80) + rowOffsets[currLine]), 0 | _lcdBacklight);
  } else {

    switch (currChar) {
      case LCD_CMD_CLEARDISPLAY:
        sendToLCD(_i2cAddress, LCD_CMD_CLEARDISPLAY, 0 | _lcdBacklight);// clear display, set cursor position to zero
        delayMicroseconds(2000);  // or 2000 ?  - this command takes a long time!
        break;

      case LCD_CMD_BACKLIGHT_BLINK:
        for (i = 0; i < LCD_BLINK_TIMES; i++) {
          // _lcdBacklight is not false/true, is 0x00 / 0x08
          _lcdBacklight = _lcdBacklight ? 0x00 : _BV(LCD_BL);
          writeByteToI2C(_i2cAddress, I2C_NO_REG_SPECIFIED, _lcdBacklight);
          delay (LCD_BLINK_DUTY_CYCLE);
        }
        
        break;

      case LCD_CMD_DISPLAYOFF:
      case LCD_CMD_RETURNHOME:
      case LCD_CMD_ENTRYMODE_RIGHTTOLEFT:
      case LCD_CMD_ENTRYMODE_RIGHTTOLEFT_SCREENSHIFT:
      case LCD_CMD_ENTRYMODE_LEFTTORIGHT:
      case LCD_CMD_ENTRYMODE_LEFTTORIGHT_SCREENSHIFT:
      case LCD_CMD_BLANKSCREEN:
      case LCD_CMD_CURSOROFF:
      case LCD_CMD_UNDERLINECURSORON:
      case LCD_CMD_BLINKINGBLOCKCURSORON:
      case LCD_CMD_CURSORMOVELEFT:
      case LCD_CMD_CURSORMOVERIGHT:
      case LCD_CMD_SCREENSHIFTLEFT:
      case LCD_CMD_SCREENSHIFTRIGHT:
        sendToLCD(_i2cAddress, currChar, 0 | _lcdBacklight);  // set cursor position to zero
        delayMicroseconds(40);  // this command takes a long time!
        break;


      // Print 'Tab'
      case LCD_CMD_HT:
        for (i = 0; i < LCD_TAB_SIZE; i++) {
            // Space is 0x20 ASCII
            // 32 => htod('2') << 4
            // 0  => htod('0') << 4
            write4bitsToLCD(_i2cAddress, 32 | _BV(LCD_RS) | _lcdBacklight);
            write4bitsToLCD(_i2cAddress, 0  | _BV(LCD_RS) | _lcdBacklight);
        }
        break;
      // Go to new line
      case LCD_CMD_LF:
        if (lastLine > currLine) { currLine++; }
        sendToLCD(_i2cAddress, (LCD_SETDDRAMADDR | rowOffsets[currLine]), 0 | _lcdBacklight);
        break;

      default:
        // Send first nibble (first four bits) into high byte area
        write4bitsToLCD(_i2cAddress, (htod(*_data) << 4) | _BV(LCD_RS) | _lcdBacklight);
        // Send second nibble (second four bits) into high byte area
        write4bitsToLCD(_i2cAddress, (htod(*(_data+1)) << 4) | _BV(LCD_RS) | _lcdBacklight);
    }
  }
    _data += 2;
  }
  return RESULT_IS_OK;
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     BH1750 SECTION
*/


#define BH1750_I2C_FIRST_ADDRESS                                0x23
#define BH1750_I2C_SECOND_ADDRESS                               0x5C
/*

  Datasheet: http://rohmfs.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1750fvi-e.pdf
*/

// No active state
#define BH1750_CMD_POWERDOWN                                    0x00
// Wating for measurment command
#define BH1750_CMD_POWERON                                      0x01
// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_CMD_RESET                                        0x07

// Start measurement at 1 lux resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGHRES                               0x10
// Start measurement at 0.5 lux resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGHRES_2                             0x11
// Start measurement at 4 lux resolution. Measurement time is approx 16ms.
#define BH1750_CONTINUOUS_LOWRES                                0x13
// Start measurement at 1 lux resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_HIGHRES                                  0x20
// Start measurement at 0.5 lux resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_HIGHRES_2                                0x21
// Start measurement at 4 lux resolution. Measurement time is approx 16ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_LOWRES                                   0x23


int32_t getBH1750Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _metric, char* _outBuffer)
{
  int32_t result;
  uint8_t setModeTimeout = 24; // 24ms - max time to complete measurement in low-resolution

  switch (_i2cAddress) {
    case BH1750_I2C_FIRST_ADDRESS:
    case BH1750_I2C_SECOND_ADDRESS: 
      break;
    default:  
       _i2cAddress = BH1750_I2C_FIRST_ADDRESS;
  }

  if (!isI2CDeviceReady(_i2cAddress)) {return DEVICE_ERROR_CONNECT; }


  switch (_mode) {
    case BH1750_CONTINUOUS_HIGHRES:
    case BH1750_CONTINUOUS_HIGHRES_2:
    case BH1750_ONETIME_HIGHRES: 
    case BH1750_ONETIME_HIGHRES_2:
         setModeTimeout = 180; // 180ms - max time to complete measurement in High-resolution
    case BH1750_CONTINUOUS_LOWRES:
    case BH1750_ONETIME_LOWRES:
       break;
     default:  
       _mode = BH1750_CONTINUOUS_LOWRES;
  }


  Wire.beginTransmission(_i2cAddress);
  Wire.write(BH1750_CMD_POWERON);
  Wire.endTransmission(true);
  //_delay_ms(10);

  delay(10);

  // Refer to Technical Note, v2010.04-Rev.C, page 7/17
  // Send "go to _mode" instruction
  // Need double read for OneTime measuring or just use continuos modes?
/*  switch (_mode) {
    case BH1750_ONETIME_HIGHRES: 
    case BH1750_ONETIME_HIGHRES_2:
    case BH1750_ONETIME_LOWRES:
      Wire.beginTransmission(_i2cAddress);
      Wire.write(_mode);
      Wire.endTransmission();
      // Wait to complete, 180ms max for H-resolution, 24ms max to L-resolution
     delayMicroseconds(setModeTimeout);
  }
*/
  Wire.beginTransmission(_i2cAddress);
  Wire.write(_mode);
  Wire.endTransmission();
  // Wait to complete, 180ms max for H-resolution, 24ms max to L-resolution
  delayMicroseconds(setModeTimeout);

  Wire.beginTransmission(_i2cAddress);
  Wire.requestFrom(_i2cAddress, (uint8_t) 2);
  result = Wire.read();
  result <<= 8;
  result |= Wire.read();
  Wire.endTransmission(true);

  if (SENS_READ_RAW == _metric) {
    ltoa(result, _outBuffer, 10);
  } else {
    // Prepare to using with ltoaf() subroutine
    // level = level/1.2; // convert to lux
    // 5 / 1.2 => 4,16
    // (5 * 1000) / 12 => 416 ==> ltoaf (..., ..., 2) ==> 4.16
    // max raw level = 65535 => 65535 * 1000 => long int
    result = (result * 1000) / 12;    
    ltoaf(result, _outBuffer, 2);
  }

  return RESULT_IN_BUFFER;
}

