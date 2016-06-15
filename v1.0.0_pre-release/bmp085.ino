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




