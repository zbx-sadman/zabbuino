#include "i2c_bus.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                      COMMON I2C SECTION

 -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

/*****************************************************************************************************************************
*
*   Scan I2C bus and print to ethernet client addresses of all detected devices 
*
*   Returns: 
*     - RESULT_IS_PRINTED on success
*     - RESULT_IS_FAIL of no devices found 
*
*****************************************************************************************************************************/
int8_t scanI2C(NetworkClass *_network)
{
#if !defined(NETWORK_RS485)

  int8_t i2cAddress, numDevices = 0;

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
      _network->client.print("0x");
      if (i2cAddress < 0x0F){ _network->client.print("0"); }
      _network->client.println(i2cAddress, HEX);
    }
  } 
  return (numDevices ? RESULT_IS_PRINTED : RESULT_IS_FAIL);
#else
  return (RESULT_IS_FAIL);
#endif
}

/*****************************************************************************************************************************
*
*   Send one byte to writeBytesToI2C() subroutine
*
*   Returns: 
*     - writeBytesToI2C()'s result code
*
*****************************************************************************************************************************/
//#define writeByteToI2C((_i2cAddress), (_registerAddress), (_data)) 
uint8_t writeByteToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _src)
{
  return writeBytesToI2C(_i2cAddress, _registerAddress, &_src, 1);
}


/*****************************************************************************************************************************
*
*   Write incoming bytes to I2C device register (if specified) or just to device
*
*   Returns: 
*     - Wire.endTransmission result code
*       0 - success
*       1 - data too long to fit in transmit buffer
*       2 - received NACK on transmit of address
*       3 - received NACK on transmit of data
*       4 - other error
*
*****************************************************************************************************************************/
uint8_t writeBytesToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t *_src, uint8_t _len) 
{
  Wire.beginTransmission(_i2cAddress); // start transmission to device 
  // registerAddress is 0x00 and above ?
  if (I2C_NO_REG_SPECIFIED < _registerAddress) {
     Wire.write((uint8_t) _registerAddress); // sends register address to be written
  }
  while (_len) {
    Wire.write(*_src);  // write data
    _src++;
    _len--;
  }
  return Wire.endTransmission(true); // end transmission
}

/*****************************************************************************************************************************
*
*   Reads bytes from device's register (or not) over I2C.
*
*   Returns: 
*     - Wire.endTransmission result code
*       0 - success
*       1 - data too long to fit in transmit buffer
*       2 - received NACK on transmit of address
*       3 - received NACK on transmit of data
*       4 - other error
*
*
*****************************************************************************************************************************/
uint8_t readBytesFromi2C(const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t *_dst, uint8_t _len)
{
    if (!_len) return false;
    Wire.beginTransmission(_i2cAddress); 	
    if (I2C_NO_REG_SPECIFIED < _registerAddress) {
       Wire.write((uint8_t) _registerAddress);
       // No Stop Condition, for repeated Talk
       Wire.endTransmission(false); 		
    }

    uint32_t lastTimeCheck = millis();         
    Wire.requestFrom(_i2cAddress, _len); 	
    while(_len && (Wire.available() > 0)) {
      // 100 => 0.1 sec timeout
      if ((millis() - lastTimeCheck) > 300UL) {
         Wire.endTransmission(true);
         return false;
      }
      *_dst = Wire.read();
      _dst++;
      _len--;
    }
    // Stop Condition
    return Wire.endTransmission(true); 		
}