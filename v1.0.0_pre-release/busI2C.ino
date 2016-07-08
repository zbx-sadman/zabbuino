/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     COMMON I2C SECTION
*/

/* ****************************************************************************************************************************
*
*   Scan I2C bus and print to ethernet client addresses of all detected devices 
*
**************************************************************************************************************************** */
int32_t i2CScan()
{
  int8_t i2cAddress, numDevices;

  for(i2cAddress = 1; i2cAddress < 0x7F; i2cAddress++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(i2cAddress);
    // 0:success
    // 1:data too long to fit in transmit buffer
    // 2:received NACK on transmit of address
    // 3:received NACK on transmit of data
    // 4:other error
    if (0 == Wire.endTransmission()) {
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
*  Reads a 16 bit value over I2C
*
**************************************************************************************************************************** */
uint16_t i2CRead16(int8_t _i2cAddress, uint8_t _registerAddress)
{
  uint16_t result;
  // start transmission to device
  Wire.beginTransmission(_i2cAddress);
  // sends register address to read from
  Wire.write(_registerAddress);
  // end transmission
  Wire.endTransmission();

  // start transmission to device
  Wire.beginTransmission(_i2cAddress);
  Wire.requestFrom(_i2cAddress, 2);
  // Infinite loop can be caused if bus not ready or broken 
//  while (Wire.available() < 2)
//    ;
  result = Wire.read();
  result <<= 8;
  result |= Wire.read();
  // end transmission
  Wire.endTransmission();

  return result;
}

/* ****************************************************************************************************************************
*
*   Reads a signed 16 bit value over I2C 
*
**************************************************************************************************************************** */
uint16_t i2CRead16_LE(int8_t _i2cAddress, uint8_t _registerAddress) {
  uint16_t temp = i2CRead16(_i2cAddress, _registerAddress);
  return (temp >> 8) | (temp << 8);
}


/* ****************************************************************************************************************************
*
*  Reads a 24 bit value over I2C
*
**************************************************************************************************************************** */
uint32_t i2CRead24(int8_t _i2cAddress, uint8_t _registerAddress)
{
  uint16_t result;
  // start transmission to device
  Wire.beginTransmission(_i2cAddress);
  // sends register address to read from
  Wire.write(_registerAddress);
  // end transmission
  Wire.endTransmission();

  // start transmission to device
  Wire.beginTransmission(_i2cAddress);
  Wire.requestFrom(_i2cAddress, 3);

  result = Wire.read();
  result <<= 8;
  result |= Wire.read();
  result <<= 8;
  result |= Wire.read();
  // end transmission
  Wire.endTransmission();

  return result;
}

/* ****************************************************************************************************************************
*
*  Write 1 byte to I2C device register
*
**************************************************************************************************************************** */
void i2CWriteByte(uint8_t _i2cAddress, uint8_t _registerAddress, uint8_t _data)
{
  Wire.beginTransmission(_i2cAddress); // start transmission to device 
  Wire.write(_registerAddress); // sends register address to be written
  Wire.write(_data);  // write data
  Wire.endTransmission(); // end transmission
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     BMP SECTION
*/

#define BMP180_I2C_ADDRESS             0x77  // I2C address of BMP sensor

#define BMP280_I2C_ADDRESS_1           0x76
#define BMP280_I2C_ADDRESS_2           0x77


#define BMP_REG_CHIPID                 0xD0

#define BMP_REG_CONTROL                0xF4 
#define BMP_REG_TEMPDATA               0xF6
#define BMP_REG_PRESSUREDATA           0xF6
#define BMP_CMD_READTEMP               0x2E
#define BMP_CMD_READPRESSURE           0x34

#define BMP085_CHIPID                  0x55
#define BMP180_CHIPID                  0x55
#define BMP280_CHIPID_1                0x56
#define BMP280_CHIPID_2                0x57
#define BMP280_CHIPID_3                0x58


#define BMP085_ULTRALOWPOWER           0x00
#define BMP085_STANDARD                0x01
#define BMP085_HIGHRES                 0x02
#define BMP085_ULTRAHIGHRES            0x03

#define BMP085_REG_CAL_AC1             0xAA  // R   Calibration data (16 bits)
#define BMP085_REG_CAL_AC2             0xAC  // R   Calibration data (16 bits)
#define BMP085_REG_CAL_AC3             0xAE  // R   Calibration data (16 bits)    
#define BMP085_REG_CAL_AC4             0xB0  // R   Calibration data (16 bits)
#define BMP085_REG_CAL_AC5             0xB2  // R   Calibration data (16 bits)
#define BMP085_REG_CAL_AC6             0xB4  // R   Calibration data (16 bits)
#define BMP085_REG_CAL_B1              0xB6  // R   Calibration data (16 bits)
#define BMP085_REG_CAL_B2              0xB8  // R   Calibration data (16 bits)
#define BMP085_REG_CAL_MB              0xBA  // R   Calibration data (16 bits)
#define BMP085_REG_CAL_MC              0xBC  // R   Calibration data (16 bits)
#define BMP085_REG_CAL_MD              0xBE  // R   Calibration data (16 bits)


#define BMP280_REGISTER_DIG_T1         0x88
#define BMP280_REGISTER_DIG_T2         0x8A
#define BMP280_REGISTER_DIG_T3         0x8C

#define BMP280_REGISTER_DIG_P1         0x8E
#define BMP280_REGISTER_DIG_P2         0x90
#define BMP280_REGISTER_DIG_P3         0x92
#define BMP280_REGISTER_DIG_P4         0x94
#define BMP280_REGISTER_DIG_P5         0x96
#define BMP280_REGISTER_DIG_P6         0x98
#define BMP280_REGISTER_DIG_P7         0x9A
#define BMP280_REGISTER_DIG_P8         0x9C
#define BMP280_REGISTER_DIG_P9         0x9E

#define BMP280_REGISTER_CAL26          0xE1  // R calibration stored in 0xE1-0xF0

#define BMP280_REGISTER_CONTROL        0xF4
#define BMP280_REGISTER_CONFIG         0xF5
#define BMP280_REGISTER_PRESSUREDATA   0xF7
#define BMP280_REGISTER_TEMPDATA       0xFA

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

#define T_INIT_MAX						20   // 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX					37   // 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX					10   // 10/16 = 0.625 ms

int32_t BMPRead(uint8_t _sdaPin, uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _overSampling, uint8_t _filterCoef, uint8_t _metric, char* _outBuffer)
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
  
  // Taking Chip ID
  Wire.beginTransmission(_i2cAddress);
  Wire.write(BMP_REG_CHIPID);
  // 0:success
  // 1:data too long to fit in transmit buffer
  // 2:received NACK on transmit of address
  // 3:received NACK on transmit of data
  // 4:other error
  if (0 != Wire.endTransmission()) { return DEVICE_DISCONNECTED_C; }

  Wire.beginTransmission(_i2cAddress);
  Wire.requestFrom((uint8_t) _i2cAddress, (uint8_t) 1);
  chipID = Wire.read();
  Wire.endTransmission();

  switch (chipID) {
    // BMP085 and BMP180 have the same ID  
#ifdef SUPPORT_BMP180_INCLUDE
    case BMP180_CHIPID: 
       BMP085Read(_sdaPin, _sclPin, _i2cAddress, _overSampling, _metric, _outBuffer);
       break;
#endif
#ifdef SUPPORT_BMP280_INCLUDE
    case BMP280_CHIPID_1: 
    case BMP280_CHIPID_2: 
    case BMP280_CHIPID_3: 
       BMP280Read(_sdaPin, _sclPin, _i2cAddress, _overSampling, _filterCoef, _metric, _outBuffer);
       break;
#endif
    default:  
       ; 
  }
  return RESULT_IN_BUFFER;  
}

/* ****************************************************************************************************************************
*
*  Read temperature or pressure from digital sensor BMP280
*
**************************************************************************************************************************** */
int32_t BMP280Read(uint8_t _sdaPin, uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _overSampling, uint8_t _filterCoef, uint8_t _metric, char* _outBuffer)
{
  uint16_t dig_T1;
  int16_t  dig_T2, dig_T3;

  uint16_t dig_P1;
  int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

/*      uint8_t dig_H1, dig_H3;
      int16_t  dig_H2, dig_H4, dig_H5;
      int8_t dig_H6;
*/
  int32_t adc;
  int64_t pvar1, pvar2, result;
  int32_t var1, var2, t_fine;

  /* read calibration data */
  dig_T1 = i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_T1);
  dig_T2 = (int16_t) i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_T2);
  dig_T3 = (int16_t) i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_T3);

  dig_P1 = i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_P1);
  dig_P2 = (int16_t) i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_P2);
  dig_P3 = (int16_t) i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_P3);
  dig_P4 = (int16_t) i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_P4);
  dig_P5 = (int16_t) i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_P5);
  dig_P6 = (int16_t) i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_P6);
  dig_P7 = (int16_t) i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_P7);
  dig_P8 = (int16_t) i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_P8);
  dig_P9 = (int16_t) i2CRead16_LE(_i2cAddress, BMP280_REGISTER_DIG_P9);

/*  testing values */
/*   
    dig_T1 = 27504;
    dig_T2 = 26435;
    dig_T3 = -1000;

    dig_P1 = 36477;
    dig_P2 = -10685;
    dig_P3 = 3024;
    dig_P4 = 2855;
    dig_P5 = 140;
    dig_P6 = -7;
    dig_P7 = 15500;
    dig_P8 = -14600;
    dig_P9 = 6000;

    adc = 519888;
*/


/*
Original BOSCH code:

    BMP280_S32_t var1, var2, T;
    var1 = ((((adc_T>>3) – ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) – ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) – ((BMP280_S32_t)dig_T1))) >> 12) * ((BMP280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
*/


  switch (_filterCoef) {
    case BMP280_FILTER_COEFF_OFF: 
    case BMP280_FILTER_COEFF_2: 
    case BMP280_FILTER_COEFF_4: 
    case BMP280_FILTER_COEFF_8: 
    case BMP280_FILTER_COEFF_16: 
      break;
    default:
      _filterCoef = BMP280_FILTER_COEFF_OFF;   
   }
   
  // set work mode
  switch ( _overSampling) {
    case BMP280_ULTRALOWPOWER: 
      var1 = BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
      var2 = BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE;
      break;
    case BMP280_LOWPOWER: 
      var1 = BMP280_LOWPOWER_OVERSAMP_TEMPERATURE;
      var2 = BMP280_LOWPOWER_OVERSAMP_PRESSURE;
      break;
    case BMP280_HIGHRES:
      var1 = BMP280_HIGHRES_OVERSAMP_TEMPERATURE;
      var2 = BMP280_HIGHRES_OVERSAMP_PRESSURE;
      break;
    case BMP280_ULTRAHIGHRES:
      var1 = BMP280_ULTRAHIGHRES_OVERSAMP_TEMPERATURE;
      var2 = BMP280_ULTRAHIGHRES_OVERSAMP_PRESSURE;
      break;
    case BMP280_STANDARD:
    default:  
      var1 = BMP280_STANDARD_OVERSAMP_TEMPERATURE;
      var2 = BMP280_STANDARD_OVERSAMP_PRESSURE;
  }

  // Set filter coefficient. While BMP280 used in Forced mode - BMP280_STANDBY_TIME_4000_MS can be any. '0' - define SPI configuration. Not used.
  i2CWriteByte(_i2cAddress, BMP280_REGISTER_CONFIG, ((BMP280_STANDBY_TIME_4000_MS << 6)| (_filterCoef << 3) | 0 ));

  // var1 is oversamp_temperature
  // var2 is oversamp_pressure
  // Use var1 as temp variable for store conversion delay time
  var1 = (T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << var1) >> 1) + ((1 << var2) >> 1)) + ((var2 > 0) ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16;
  // BMP280_FORCED_MODE -> act like BMP180
  i2CWriteByte(_i2cAddress, BMP_REG_CONTROL, ((var1 << 6)| (var2 << 3) | BMP280_FORCED_MODE));
  delay(var1);


  // Compensate temperature caculation
  adc = i2CRead24(_i2cAddress, BMP280_REGISTER_TEMPDATA);
  adc >>= 3;
  var1  = (((adc - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
  adc = (adc >> 1) - (int32_t) dig_T1;
  var2  = (((adc * adc) >> 12) * ((int32_t) dig_T3)) >> 14;
  t_fine = var1 + var2;


  if ( SENS_READ_PRSS == _metric) {
     // Pressure caculation
     adc = i2CRead24(_i2cAddress, BMP280_REGISTER_PRESSUREDATA);
     // Test value
     // adc = 415148;
     // It's the BOSH code, page 22 of datasheet
     pvar1 = ((int64_t) t_fine) - 128000;
     pvar2 = pvar1 * pvar1 * (int64_t) dig_P6;
     pvar2 = pvar2 + ((pvar1*(int64_t) dig_P5) << 17);
     pvar2 = pvar2 + (((int64_t) dig_P4) << 35);
     pvar1 = ((pvar1 * pvar1 * (int64_t) dig_P3) >> 8) + ((pvar1 * (int64_t) dig_P2) << 12);
     pvar1 = (((((int64_t) 1) << 47)+pvar1))*((int64_t) dig_P1) >> 33;

     if (pvar1 == 0) {
        return 0;  // avoid exception caused by division by zero
     }
     result = 1048576 - adc;
     result = (((result << 31) - pvar2) * 3125) / pvar1;
     pvar1 = (((int64_t) dig_P9) * (result >> 13) * (result  >> 13)) >> 25;
     pvar2 = (((int64_t) dig_P8) * result) >> 19;
     result = ((result + pvar1 + pvar2) >> 8) + (((int64_t) dig_P7) << 4);

     // BOSH on page 22 of datasheet say: "Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits)." 
     // 24674867 in Q24.8 is 96386 in whole part (24674867 >> 8) , and 51 in frac part (24674867 & B11111111) => 96386.51
     // But in code example use calculation: 24674867/256 = 96386.19 => 96386.2
     //
     // What way is right, BOSCH? 
     //
     qtoaf(result, _outBuffer, 8);
  } else {
     // Temperature caculation
     result  = (t_fine * 5 + 128) >> 8;
     //  return T/100;    
     ltoaf(result, _outBuffer, 2);
  }
  return RESULT_IN_BUFFER;
}

/* ****************************************************************************************************************************
*
*  Read temperature or pressure from digital sensor BMP085(BMP180)
*
**************************************************************************************************************************** */
int32_t BMP085Read(uint8_t _sdaPin, uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _oversampling, uint8_t _metric, char* _outBuffer)
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


  /* read calibration data */
  ac1 = i2CRead16(_i2cAddress, BMP085_REG_CAL_AC1);
  ac2 = i2CRead16(_i2cAddress, BMP085_REG_CAL_AC2);
  ac3 = i2CRead16(_i2cAddress, BMP085_REG_CAL_AC3);
  ac4 = i2CRead16(_i2cAddress, BMP085_REG_CAL_AC4);
  ac5 = i2CRead16(_i2cAddress, BMP085_REG_CAL_AC5);
  ac6 = i2CRead16(_i2cAddress, BMP085_REG_CAL_AC6);
  b1 = i2CRead16(_i2cAddress, BMP085_REG_CAL_B1);
  b2 = i2CRead16(_i2cAddress, BMP085_REG_CAL_B2);
  mb = i2CRead16(_i2cAddress, BMP085_REG_CAL_MB);
  mc = i2CRead16(_i2cAddress, BMP085_REG_CAL_MC);
  md = i2CRead16(_i2cAddress, BMP085_REG_CAL_MD);


  switch ( _oversampling) {
    case BMP085_ULTRALOWPOWER: 
    case BMP085_STANDARD:
    case BMP085_HIGHRES:
    case BMP085_ULTRAHIGHRES:
       break;
     default:  
        _oversampling = BMP085_STANDARD;
  }



  // ******** Get Temperature ********
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  i2CWriteByte(_i2cAddress, BMP_REG_CONTROL, BMP_CMD_READTEMP);

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = i2CRead16(_i2cAddress, BMP_REG_TEMPDATA);

  x1 = (((int32_t) ut - (int32_t) ac6) * (int32_t) ac5) >> 15;
  x2 = ((int32_t) mc << 11) / (x1 + (int32_t) md);
  b5 = x1 + x2;
  result = ((b5 + 8) >> 4);

  if ( SENS_READ_PRSS == _metric)
  {
    // ******** Get pressureTemperature ********

    // Calculate pressure given up
    // calibration values must be known
    // b5 is also required so bmp085GetTemperature(...) must be called first.
    // Value returned will be pressure in units of Pa.

    // Write 0x34+(_oversampling<<6) into register 0xF4
    // Request a pressure reading w/ oversampling setting
    i2CWriteByte(_i2cAddress, BMP_REG_CONTROL, BMP_CMD_READPRESSURE + (_oversampling << 6));

    // Wait for conversion, delay time dependent on _oversampling
    delay(2 + (3 << _oversampling));

    // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
    Wire.beginTransmission(_i2cAddress);
    Wire.write(BMP_REG_PRESSUREDATA);
    Wire.endTransmission();

    Wire.beginTransmission(_i2cAddress);
    Wire.requestFrom(_i2cAddress, (uint8_t) 3);

    msb = Wire.read();
    lsb = Wire.read();
    xlsb = Wire.read();

    up = (((uint32_t) msb << 16) | ((uint32_t) lsb << 8) | (uint32_t) xlsb) >> (8 - _oversampling);

    b6 = b5 - 4000;

    // Calculate B3
    x1 = ((int32_t) b2 * (b6 * b6) >> 12) >> 11;
    x2 = ((int32_t) ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)ac1) * 4 + x3) << _oversampling) + 2) >> 2;

    // Calculate B4
    x1 = ((int32_t) ac3 * b6) >> 13;
    x2 = ((int32_t) b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = ((int32_t) ac4 * (uint32_t)(x3 + 32768)) >> 15;

    b7 = ((uint32_t)(up - b3) * (uint32_t)(50000 >> _oversampling));

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
                                                                     BH1750 SECTION
*/


#define BH1750_I2C_FIRST_ADDRESS            0x23
#define BH1750_I2C_SECOND_ADDRESS           0x5C
/*

  Datasheet: http://rohmfs.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1750fvi-e.pdf
*/

// No active state
#define BH1750_CMD_POWERDOWN                0x00
// Wating for measurment command
#define BH1750_CMD_POWERON                  0x01
// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_CMD_RESET                    0x07
// Start measurement at 1lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGHRES           0x10
// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGHRES_2         0x11
// Start measurement at 4lx resolution. Measurement time is approx 16ms.
#define BH1750_CONTINUOUS_LOWRES            0x13
// Start measurement at 1lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_HIGHRES              0x20
// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_HIGHRES_2            0x21
// Start measurement at 4lx resolution. Measurement time is approx 16ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_LOWRES               0x23


int32_t BH1750Read(uint8_t _sdaPin, uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _mode, uint8_t _metric, char* _outBuffer)
{
  int32_t result;
  uint8_t setModeTimeout = 24; // 24ms - max time to complete measurement in High-resolution

  switch (_mode) {
    case BH1750_ONETIME_HIGHRES: 
    case BH1750_ONETIME_HIGHRES_2:
         setModeTimeout = 180; // 180ms - max time to complete measurement in High-resolution
    case BH1750_ONETIME_LOWRES:
       break;
     default:  
       _mode = BH1750_ONETIME_LOWRES;
  }

  switch (_i2cAddress) {
    case BH1750_I2C_FIRST_ADDRESS:
    case BH1750_I2C_SECOND_ADDRESS: 
      break;
    default:  
       _i2cAddress = BH1750_I2C_FIRST_ADDRESS;
  }

  Wire.beginTransmission(_i2cAddress);
  Wire.write(BH1750_CMD_POWERON);
  Wire.endTransmission();
  _delay_ms(10);

  Wire.beginTransmission(_i2cAddress);
  Wire.write(BH1750_CMD_RESET);
  Wire.endTransmission();
  _delay_ms(10);

  // Refer to Technical Note, v2010.04-Rev.C, page 7/17
  // Send "go to _mode" instruction
  Wire.beginTransmission(_i2cAddress);
  Wire.write(_mode);
  Wire.endTransmission();
  // Wait to complete, 180ms max for H-resolution, 24ms max to L-resolution
  delayMicroseconds(setModeTimeout);

  // result = i2CRead16(_i2cAddress, 0x00);
  // Why do not point to register address before read - it's 0x00 or BH1750 do not need to use register address to return light raw value?
  Wire.beginTransmission(_i2cAddress);
  Wire.requestFrom(_i2cAddress, (uint8_t) 2);
  result = Wire.read();
  result <<= 8;
  result |= Wire.read();
  Wire.endTransmission();
 
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

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     LCD SECTION


Original code on: https://github.com/marcoschwartz/LiquidCrystal_I2C
version 1.1.2 is used


*/



/* LCD finction     PC8574 port (bit # in byte which send to I2C expander) */
#define LCD_RS                    0                          // P0 (pin 4) on PC8574 - expander pin #4
#define LCD_RW                    1                          // P1 (pin 5) on PC8574 - expander pin #5
#define LCD_E                     2                          // P2 (pin 6) on PC8574 - expander pin #6
#define LCD_BL                    3                          // P3 (pin 7) on PC8574 - expander pin #7

#define LCD_D4                    4                          // P4 (pin 9) on PC8574  - expander pin #11
#define LCD_D5                    5                          // P5 (pin 10) on PC8574 - expander pin #12
#define LCD_D6                    6                          // P6 (pin 11) on PC8574 - expander pin #13
#define LCD_D7                    7                          // P7 (pin 12) on PC8574 - expander pin #14

#define LCD_TYPE_801              801
#define LCD_TYPE_1601             1601
                            
#define LCD_TYPE_802              802 
#define LCD_TYPE_1202             1202 
#define LCD_TYPE_1602             1602 
#define LCD_TYPE_2002             2002
#define LCD_TYPE_2402             2402
#define LCD_TYPE_4002             4002

#define LCD_TYPE_1604             1604
#define LCD_TYPE_2004             2004
#define LCD_TYPE_4004             4004

#define LCD_TAB_SIZE              0x04 // 4 space
#define LCD_BLINK_DUTY_CYCLE      250  // 250 ms
#define LCD_BLINK_TIMES           4    // in cycle of 4 times - 2 off state & 2 on state. Need use even numbers for save previous backlight state on cycle finish

// commands
#define LCD_CMD_BACKLIGHT_BLINK                          0x03 // ASCII    - backlight blink
#define LCD_CMD_HT                                       0x09 // ASCII 09 - horizontal tabulation
#define LCD_CMD_LF                                       0x0A // ASCII 10 - line feed

#define LCD_CMD_DISPLAYOFF                               0x00
#define LCD_CMD_CLEARDISPLAY                             0x01
#define LCD_CMD_RETURNHOME                               0x02
#define LCD_CMD_ENTRYMODE_RIGHTTOLEFT                    0x04
#define LCD_CMD_ENTRYMODE_RIGHTTOLEFT_SCREENSHIFT        0x05
#define LCD_CMD_ENTRYMODE_LEFTTORIGHT                    0x06
#define LCD_CMD_ENTRYMODE_LEFTTORIGHT_SCREENSHIFT        0x07
#define LCD_CMD_BLANKSCREEN                              0x08
#define LCD_CMD_CURSOROFF                                0x0C
#define LCD_CMD_UNDERLINECURSORON                        0x0E
#define LCD_CMD_BLINKINGBLOCKCURSORON                    0x0F
#define LCD_CMD_CURSORMOVELEFT                           0x10
#define LCD_CMD_CURSORMOVERIGHT                          0x14
#define LCD_CMD_SCREENSHIFTLEFT                          0x18
#define LCD_CMD_SCREENSHIFTRIGHT                         0x1E


#define LCD_DISPLAYCONTROL    0x08
#define LCD_CURSORSHIFT       0x10
#define LCD_FUNCTIONSET       0x20
#define LCD_SETDDRAMADDR      0x80

// flags for display on/off control
#define LCD_DISPLAYON             0x04
#define LCD_DISPLAYOFF            0x00
#define LCD_CURSORON              0x02
#define LCD_CURSOROFF             0x00
#define LCD_BLINKON               0x01
#define LCD_BLINKOFF              0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE           0x08
#define LCD_CURSORMOVE            0x00
#define LCD_MOVERIGHT             0x04
#define LCD_MOVELEFT              0x00


// flags for function set
#define LCD_8BITMODE              0x10
#define LCD_4BITMODE              0x00
#define LCD_2LINE                 0x08
#define LCD_1LINE                 0x00

// flags for backlight control
#define LCD_BACKLIGHT             0x08
#define LCD_NOBACKLIGHT           0x00


void lcdSend(uint8_t _i2cAddress, uint8_t _data, uint8_t _mode)
{
  // Send high bit
  lcdWrite4bits(_i2cAddress, (_data & 0xF0) | _mode);
  // Send low bit
  lcdWrite4bits(_i2cAddress, ((_data << 4) & 0xF0) | _mode);
}

void lcdWrite4bits(uint8_t _i2cAddress, uint8_t _data) 
{
  expanderWrite(_i2cAddress, _data);
  lcdPulseEnable(_i2cAddress, _data);
}

void lcdPulseEnable(uint8_t _i2cAddress, uint8_t _data)
{
  expanderWrite(_i2cAddress, _data | _BV(LCD_E));	// 'Enable' high
  delayMicroseconds(1);		                        // enable pulse must be >450ns
  expanderWrite(_i2cAddress, _data & ~_BV(LCD_E));	// 'Enable' low
  delayMicroseconds(50);	                        // commands need > 37us to settle
} 

void expanderWrite(uint8_t _i2cAddress, uint8_t _data)
{                                        
  Wire.beginTransmission(_i2cAddress);
  Wire.write((uint8_t) _data);
  Wire.endTransmission();   
}

uint8_t pc8574LCDOutput(uint8_t _sdaPin, uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _lcdBacklight, uint16_t _lcdType, const char *_data)
{
  uint8_t displayFunction, lastLine, currLine, currChar, i;
  uint8_t rowOffsets[] = { 0x00, 0x40, 0x14, 0x54 };

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

  if (!haveHexPrefix(_data)) {return false;}
  _data+=2;

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  delay(50); 

  // Now we pull both RS and R/W low to begin commands
//  expanderWrite(_i2cAddress, ((_lcdBacklight > 0) ? LCD_BACKLIGHT : LCD_NOBACKLIGHT));
  expanderWrite(_i2cAddress, _lcdBacklight);
  delayMicroseconds(40000); // wait 40ms

  //put the LCD into 4 bit mode
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46
	
  // we start in 8bit mode, try to set 4 bit mode
  lcdWrite4bits(_i2cAddress, (0x03 << 4) | _lcdBacklight);
  delayMicroseconds(4500); // wait min 4.1ms
   
  // second try
  lcdWrite4bits(_i2cAddress, (0x03 << 4) | _lcdBacklight);
  delayMicroseconds(4500); // wait min 4.1ms
   
  // third go!
  lcdWrite4bits(_i2cAddress, (0x03 << 4) | _lcdBacklight);
  delayMicroseconds(150);
   
  // finally, set to 4-bit interface
  lcdWrite4bits(_i2cAddress, (0x02 << 4) | _lcdBacklight);

  // set # lines, font size, etc.
  lcdSend(_i2cAddress, (LCD_FUNCTIONSET | displayFunction), 0 | _lcdBacklight);
  lcdSend(_i2cAddress, (LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF), 0 | _lcdBacklight);

  // Always begin from 0,0
  currLine = 0; 
  while(*_data && *(_data+1)) {
    // restore full byte (ASCII char) from HEX nibbles
    currChar = (htod(*_data) << 4) + htod(*(_data+1));
    if (currChar > 0x7F && currChar < 0x9F) {
     lcdSend(_i2cAddress, LCD_SETDDRAMADDR | ((currChar - 0x80) + rowOffsets[currLine]), 0 | _lcdBacklight);
  } else {

    switch (currChar) {
      case LCD_CMD_CLEARDISPLAY:
        lcdSend(_i2cAddress, LCD_CMD_CLEARDISPLAY, 0 | _lcdBacklight);// clear display, set cursor position to zero
        delayMicroseconds(2000);  // or 2000 ?  - this command takes a long time!
        break;

      case LCD_CMD_BACKLIGHT_BLINK:
//        uint8_t tmp_lcdBacklight = (uint8_t) _lcdBacklight;
        for (i = 0; i < LCD_BLINK_TIMES; i++) {
          // tmp_lcdBacklight is not false/true, is 0x00 / 0x08
          _lcdBacklight = _lcdBacklight ? 0x00 : _BV(LCD_BL);
          expanderWrite(_i2cAddress, _lcdBacklight);
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
        lcdSend(_i2cAddress, currChar, 0 | _lcdBacklight);  // set cursor position to zero
        delayMicroseconds(40);  // this command takes a long time!
        break;


      // Print 'Tab'
      case LCD_CMD_HT:
        for (i = 0; i < LCD_TAB_SIZE; i++) {
            // Space is 0x20 ASCII
            // 32 => htod('2') << 4
            // 0  => htod('0') << 4
            lcdWrite4bits(_i2cAddress, 32 | _BV(LCD_RS) | _lcdBacklight);
            lcdWrite4bits(_i2cAddress, 0  | _BV(LCD_RS) | _lcdBacklight);
        }
        break;
      // Go to new line
      case LCD_CMD_LF:
        if (lastLine > currLine) { currLine++; }
        lcdSend(_i2cAddress, (LCD_SETDDRAMADDR | rowOffsets[currLine]), 0 | _lcdBacklight);
        break;

      default:
        // Send first nibble (first four bits) into high byte area
        lcdWrite4bits(_i2cAddress, (htod(*_data) << 4) | _BV(LCD_RS) | _lcdBacklight);
        // Send second nibble (second four bits) into high byte area
        lcdWrite4bits(_i2cAddress, (htod(*(_data+1)) << 4) | _BV(LCD_RS) | _lcdBacklight);
    }
  }
    _data += 2;
  }
  return true;
}

uint8_t pc8574LCDBackLight(uint8_t _sdaPin, uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _lcdBacklight)
{  
  expanderWrite(_i2cAddress, ((_lcdBacklight) ? LCD_BACKLIGHT : LCD_NOBACKLIGHT));
}


