/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     COMMON I2C SECTION
*/
int32_t i2CScan()
{
  int8_t i2cAddress;

  for(i2cAddress = 1; i2cAddress < 0x7F; i2cAddress++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(i2cAddress);
   if (0 == Wire.endTransmission()) {
      ethClient.print("0x");
      if (i2cAddress<16){ ethClient.print("0"); }
      ethClient.println(i2cAddress, HEX);
    }
  } 
  // what is 4 == Wire.endTransmission() - wrong answer from existing device?

  return RESULT_IS_PRINTED;
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     BMP085 SECTION
*/

#define BMP085_I2C_FIRST_ADDRESS       0x77  // I2C address of BMP085

/* ****************************************************************************************************************************
*
*  Read temperature or pressure from digital sensor BMP085(BMP180);
*
**************************************************************************************************************************** */
int32_t BMP085Read(uint8_t _sdaPin, uint8_t _sclPin, uint8_t _i2cAddress, int8_t _oversampling, uint8_t _metric, char* _outBuffer)
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

  switch (_i2cAddress) {
    case BMP085_I2C_FIRST_ADDRESS:
      break;
    default:  
       _i2cAddress = BMP085_I2C_FIRST_ADDRESS;
  }
  
  ac1 = bmp085ReadInt(0xAA, _i2cAddress);
  ac2 = bmp085ReadInt(0xAC, _i2cAddress);
  ac3 = bmp085ReadInt(0xAE, _i2cAddress);
  ac4 = bmp085ReadInt(0xB0, _i2cAddress);
  ac5 = bmp085ReadInt(0xB2, _i2cAddress);
  ac6 = bmp085ReadInt(0xB4, _i2cAddress);
  b1 = bmp085ReadInt(0xB6, _i2cAddress);
  b2 = bmp085ReadInt(0xB8, _i2cAddress);
  mb = bmp085ReadInt(0xBA, _i2cAddress);
  mc = bmp085ReadInt(0xBC, _i2cAddress);
  md = bmp085ReadInt(0xBE, _i2cAddress);

  // ******** Get Temperature ********
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(_i2cAddress);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6, _i2cAddress);

  x1 = (((long)ut - (long)ac6) * (long)ac5) >> 15;
  x2 = ((long)mc << 11) / (x1 + md);
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
    Wire.beginTransmission(_i2cAddress);
    Wire.write(0xF4);
    Wire.write(0x34 + (_oversampling << 6));
    Wire.endTransmission();

    // Wait for conversion, delay time dependent on _oversampling
    delay(2 + (3 << _oversampling));

    // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
    Wire.beginTransmission(_i2cAddress);
    Wire.write(0xF6);
    Wire.endTransmission();
    Wire.requestFrom(_i2cAddress, 2);

    // Wait for data to become available
    while (Wire.available() < 3) { ; }
    msb = Wire.read();
    lsb = Wire.read();
    xlsb = Wire.read();

    up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8 - _oversampling);

    b6 = b5 - 4000;

    // Calculate B3
    x1 = (b2 * (b6 * b6) >> 12) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((long)ac1) * 4 + x3) << _oversampling) + 2) >> 2;

    // Calculate B4
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;

    b7 = ((unsigned long)(up - b3) * (50000 >> _oversampling));

    if (b7 < 0x80000000)
      result = (b7 << 1) / b4;
    else
      result = (b7 / b4) << 1;

    x1 = (result >> 8) * (result >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * result) >> 16;

    result += (x1 + x2 + 3791) >> 4;

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

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int16_t  bmp085ReadInt(unsigned char _address, int8_t _i2cAddress)
{
  uint8_t msb, lsb;

  Wire.beginTransmission(_i2cAddress);
  Wire.write(_address);
  Wire.endTransmission();
  Wire.requestFrom(_i2cAddress, 2);
  while (Wire.available() < 2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int16_t) msb << 8 | lsb;
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     BH1750 SECTION
*/


#define BH1750_I2C_FIRST_ADDRESS                  0x23
#define BH1750_I2C_SECOND_ADDRESS                 0x5C
/*

  Datasheet: http://rohmfs.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1750fvi-e.pdf
*/

// No active state
#define BH1750_POWER_DOWN                   0x00
// Wating for measurment command
#define BH1750_POWER_ON                     0x01
// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_RESET                         0x07
// Start measurement at 1lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGH_RES_MODE      0x10
// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2    0x11
// Start measurement at 4lx resolution. Measurement time is approx 16ms.
#define BH1750_CONTINUOUS_LOW_RES_MODE       0x13
// Start measurement at 1lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_HIGH_RES_MODE        0x20
// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_HIGH_RES_MODE_2      0x21
// Start measurement at 4lx resolution. Measurement time is approx 16ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_LOW_RES_MODE         0x23


int32_t BH1750Read(uint8_t _sdaPin, uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _mode, uint8_t _metric, char* _outBuffer)
{
   int32_t result;

  // Need call begin()? 
  // Wire.begin();

  switch (_mode) {
    case BH1750_ONE_TIME_HIGH_RES_MODE: 
    case BH1750_ONE_TIME_HIGH_RES_MODE_2:
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

  Wire.beginTransmission(_i2cAddress);
  Wire.write(_mode);
  Wire.endTransmission();
  _delay_ms(10);

  Wire.beginTransmission(_i2cAddress);
  Wire.requestFrom(_i2cAddress, 2);
  result = Wire.read();
  result <<= 8;
  result |= Wire.read();
  Wire.endTransmission();

  if (SENS_READ_RAW == _metric) {
    ltoa(result, _outBuffer, 10);
  } else {
    // level = level/1.2; // convert to lux
    // 5 / 1.2 => 4,16
    // (5 * 1000) / 12 => 416 ==ltoaf (..., ..., 2)==> 4.16
    // max raw level = 65535 => 65535 * 1000 => long int
    result = (result * 1000) / 12;    
    ltoaf(result, _outBuffer, 2);
  }

  return RESULT_IN_BUFFER;
}

