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
*   Scan I2C bus and store I2C addresses of detected devices 
*
*   Returns: 
*     - number of found devices
*
*****************************************************************************************************************************/
int8_t scanI2C(SoftwareWire* _softTWI, uint8_t *_dst)
{

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
      _dst[numDevices]=i2cAddress;
      numDevices++;
    }
  } 
  return numDevices;
}
