#define BH1750_I2C_ADDRESS                  0x23

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
// Start measurement at 1lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_LOW_RES_MODE         0x23


int16_t BH1750Read(uint8_t _sdaPin, uint8_t _sclPin, uint8_t _mode, uint8_t _metric, char* _outBuffer)
{
   uint16_t result;

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
  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.write(_mode);
  Wire.endTransmission();
  _delay_ms(10);

  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.requestFrom(BH1750_I2C_ADDRESS, 2);
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
