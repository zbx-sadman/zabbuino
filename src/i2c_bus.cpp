#include "i2c_bus.h"

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
int8_t scanI2C(EthernetClient *_ethClient)
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
      _ethClient->print("0x");
      if (i2cAddress < 0x0F){ _ethClient->print("0"); }
      _ethClient->println(i2cAddress, HEX);
    }
  } 
  return (numDevices ? RESULT_IS_PRINTED : RESULT_IS_FAIL);
}

/* ****************************************************************************************************************************
*
*  Write 1 byte to I2C device register or just to device. All calls must be re-pointed to writeByte_s_ToI2C()
*
**************************************************************************************************************************** */
//#define writeByteToI2C((_i2cAddress), (_registerAddress), (_data)) 
uint8_t writeByteToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _data)
{
  return writeBytesToI2C(_i2cAddress, _registerAddress, &_data, 1);
}


/* ****************************************************************************************************************************
*
*  Write N bytes to I2C device register or just to device
*
**************************************************************************************************************************** */
uint8_t writeBytesToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t* _data, uint8_t _len) 
{
  Wire.beginTransmission(_i2cAddress); // start transmission to device 
  // registerAddress is 0x00 and above ?
  if (I2C_NO_REG_SPECIFIED < _registerAddress) {
     Wire.write((uint8_t) _registerAddress); // sends register address to be written
  }
  while (_len) {
    Wire.write(*_data);  // write data
    _data++;
    _len--;
  }
  return Wire.endTransmission(true); // end transmission
}

/* ****************************************************************************************************************************
*
*  Reads N bytes from device's register (or not) over I2C
*
**************************************************************************************************************************** */
uint8_t readBytesFromi2C(const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t* _dst, uint8_t _len)
{
    if (!_len) return false;
    Wire.beginTransmission(_i2cAddress); 	// Adress + WRITE (0)
    if (I2C_NO_REG_SPECIFIED < _registerAddress) {
       Wire.write((uint8_t) _registerAddress);
       Wire.endTransmission(false); 		// No Stop Condition, for repeated Talk
    }

    uint32_t lastTimeCheck = millis();         
    Wire.requestFrom(_i2cAddress, _len); 	// Address + READ (1)
    while(_len && (Wire.available() > 0)) {
      // 100 => 0.1 sec timeout
      if ((millis() - lastTimeCheck) > 300) {
         Wire.endTransmission(true);
         return false;
      }
      *_dst = Wire.read();
      _dst++;
      _len--;
    }

    return Wire.endTransmission(true); 		// Stop Condition
};




/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     BH1750 SECTION
*/


int8_t getBH1750Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _metric, char* _dst)
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
    ltoa(result, _dst, 10);
  } else {
    // Prepare result's value to using in ltoaf() subroutine
    // level = level/1.2; // convert to lux
    // 5 / 1.2 => 4,16
    // (5 * 1000) / 12 => 416 ==> ltoaf (..., ..., 2) ==> 4.16
    // max raw level = 65535 => 65535 * 1000 => long int
    result = (result * 1000) / 12;    
    ltoaf(result, _dst, 2);
  }

  return RESULT_IN_BUFFER;
}

