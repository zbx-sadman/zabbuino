// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "network.h"

#include "i2c_bus.h"
#include "i2c_common.h"

/*****************************************************************************************************************************
*
*   Scan I2C bus and print to ethernet client addresses of all detected devices 
*
*   Returns: 
*     - RESULT_IS_PRINTED on success
*     - RESULT_IS_FAIL of no devices found 
*
*****************************************************************************************************************************/
int8_t scanI2C(SoftwareWire* _softTWI, NetworkClass *_network)
{
// #if !defined(NETWORK_RS485)

  int8_t i2cAddress, numDevices = 0;

  for(i2cAddress = 0x01; i2cAddress < 0x7F; i2cAddress++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    _softTWI->beginTransmission(i2cAddress);
    // 0:success
    // 1:data too long to fit in transmit buffer
    // 2:received NACK on transmit of address
    // 3:received NACK on transmit of data
    // 4:other error
    if (0 == _softTWI->endTransmission(true)) {
      numDevices++;
      _network->client.print("0x");
      DTSL ( Serial.print("0x"); ) 
      if (i2cAddress < 0x0F){ 
          _network->client.print("0"); 
          DTSL ( Serial.print("0"); ) 
      }
      _network->client.println(i2cAddress, HEX);
      DTSL ( Serial.println(i2cAddress, HEX); ) 
    }
  } 
  return (numDevices ? RESULT_IS_PRINTED : RESULT_IS_FAIL);
/*
#else
  return (RESULT_IS_FAIL);
#endif
*/
}
