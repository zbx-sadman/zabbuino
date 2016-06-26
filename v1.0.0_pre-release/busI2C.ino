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
*  Read 2 bytes (int16_t) from the I2C device
*
**************************************************************************************************************************** */
uint16_t i2CReadInt(int8_t _i2cAddress, uint8_t _registerAddress)
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
*  Write 1 byte to I2C device register
*
**************************************************************************************************************************** */
void i2CWriteByte(uint8_t _i2cAddress, uint8_t _registerAddress, uint8_t _data)
{
  Wire.beginTransmission(_i2cAddress); // start transmission to device 
  Wire.write(_registerAddress); // sends register address to read from
  Wire.write(_data);  // write data
  Wire.endTransmission(); // end transmission
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     BMP085 SECTION
*/

#define BMP085_I2C_FIRST_ADDRESS   0x77  // I2C address of BMP085

#define BMP085_ULTRALOWPOWER       0x00
#define BMP085_STANDARD            0x01
#define BMP085_HIGHRES             0x02
#define BMP085_ULTRAHIGHRES        0x03

#define BMP085_CAL_AC1             0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2             0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3             0xAE  // R   Calibration data (16 bits)    
#define BMP085_CAL_AC4             0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5             0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6             0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1              0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2              0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB              0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC              0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD              0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL             0xF4 
#define BMP085_TEMPDATA            0xF6
#define BMP085_PRESSUREDATA        0xF6
#define BMP085_READTEMPCMD         0x2E
#define BMP085_READPRESSURECMD     0x34

/* ****************************************************************************************************************************
*
*  Read temperature or pressure from digital sensor BMP085(BMP180);
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

  switch ( _oversampling) {
    case BMP085_ULTRALOWPOWER: 
    case BMP085_STANDARD:
    case BMP085_HIGHRES:
    case BMP085_ULTRAHIGHRES:
       break;
     default:  
        _oversampling = BMP085_STANDARD;
  }

  switch (_i2cAddress) {
    case BMP085_I2C_FIRST_ADDRESS:
      break;
    default:  
       _i2cAddress = BMP085_I2C_FIRST_ADDRESS;
  }
  
  //if (read8(0xD0) != 0x55) return false; < ??
  /* read calibration data */
  ac1 = i2CReadInt(_i2cAddress, BMP085_CAL_AC1);
  ac2 = i2CReadInt(_i2cAddress, BMP085_CAL_AC2);
  ac3 = i2CReadInt(_i2cAddress, BMP085_CAL_AC3);
  ac4 = i2CReadInt(_i2cAddress, BMP085_CAL_AC4);
  ac5 = i2CReadInt(_i2cAddress, BMP085_CAL_AC5);
  ac6 = i2CReadInt(_i2cAddress, BMP085_CAL_AC6);
  b1 = i2CReadInt(_i2cAddress, BMP085_CAL_B1);
  b2 = i2CReadInt(_i2cAddress, BMP085_CAL_B2);
  mb = i2CReadInt(_i2cAddress, BMP085_CAL_MB);
  mc = i2CReadInt(_i2cAddress, BMP085_CAL_MC);
  md = i2CReadInt(_i2cAddress, BMP085_CAL_MD);


  // ******** Get Temperature ********
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  i2CWriteByte(_i2cAddress, BMP085_CONTROL, BMP085_READTEMPCMD);

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = i2CReadInt(_i2cAddress, BMP085_TEMPDATA);

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
    i2CWriteByte(_i2cAddress, BMP085_CONTROL, BMP085_READPRESSURECMD + (_oversampling << 6));

    // Wait for conversion, delay time dependent on _oversampling
    delay(2 + (3 << _oversampling));

    // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
    Wire.beginTransmission(_i2cAddress);
    Wire.write(BMP085_PRESSUREDATA);
    Wire.endTransmission();

    Wire.beginTransmission(_i2cAddress);
    Wire.requestFrom(_i2cAddress, (uint8_t) 3);

    // Wait for data to become available
    // Infinite loop can be caused if bus not ready or broken 
//    while (Wire.available() < 3) { ; }
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
#define BH1750_POWER_DOWN                   0x00
// Wating for measurment command
#define BH1750_POWER_ON                     0x01
// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_RESET                        0x07
// Start measurement at 1lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGH_RES_MODE     0x10
// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2   0x11
// Start measurement at 4lx resolution. Measurement time is approx 16ms.
#define BH1750_CONTINUOUS_LOW_RES_MODE      0x13
// Start measurement at 1lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_HIGH_RES_MODE       0x20
// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_HIGH_RES_MODE_2     0x21
// Start measurement at 4lx resolution. Measurement time is approx 16ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_LOW_RES_MODE        0x23


int32_t BH1750Read(uint8_t _sdaPin, uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _mode, uint8_t _metric, char* _outBuffer)
{
  int32_t result;
  uint8_t setModeTimeout = 24; // 24ms - max time to complete measurement in High-resolution

  switch (_mode) {
    case BH1750_ONE_TIME_HIGH_RES_MODE: 
    case BH1750_ONE_TIME_HIGH_RES_MODE_2:
         setModeTimeout = 180; // 180ms - max time to complete measurement in High-resolution
    case BH1750_ONE_TIME_LOW_RES_MODE:
       break;
     default:  
       _mode = BH1750_ONE_TIME_LOW_RES_MODE;
  }

  switch (_i2cAddress) {
    case BH1750_I2C_FIRST_ADDRESS:
    case BH1750_I2C_SECOND_ADDRESS: 
      break;
    default:  
       _i2cAddress = BH1750_I2C_FIRST_ADDRESS;
  }

  Wire.beginTransmission(_i2cAddress);
  Wire.write(BH1750_POWER_ON);
  Wire.endTransmission();
  _delay_ms(10);

  Wire.beginTransmission(_i2cAddress);
  Wire.write(BH1750_RESET);
  Wire.endTransmission();
  _delay_ms(10);

  // Refer to Technical Note, v2010.04-Rev.C, page 7/17
  // Send "go to _mode" instruction
  Wire.beginTransmission(_i2cAddress);
  Wire.write(_mode);
  Wire.endTransmission();
  // Wait to complete, 180ms max for H-resolution, 24ms max to L-resolution
  delayMicroseconds(setModeTimeout);

  // result = i2CReadInt(_i2cAddress, 0x00);
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

