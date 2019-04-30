// Config & common included files
#include "sys_includes.h"

// OneWire lib for Dallas sensors
#include "OneWire\OneWire.h"

#include "service.h"
#include "network.h"

#include "ow_bus.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
*
*                                                                     COMMON 1-WIRE SECTION
*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

/*****************************************************************************************************************************
*
*   Scan 1-Wire bus and store ID's (addresses) of detected devices 
*
*   Returns: 
*     - number of found devices
*
*****************************************************************************************************************************/
int8_t scanOneWire(const uint8_t _pin, uint8_t* _dst, size_t _maxLen) {
  uint8_t dsAddr[8], 
          numDevices = 0x00;
  OneWire owDevice(_pin);

  // Test the bus
  if (!owDevice.reset()) { goto finish; }

  owDevice.reset_search();
  while (owDevice.search(dsAddr)) {
    // Do not write more that _maxLen (buffer size)
    if (_maxLen > sizeof(dsAddr)) {
      _maxLen -= sizeof(dsAddr);
       memcpy(_dst, dsAddr, sizeof(dsAddr));
      _dst += sizeof(dsAddr);
      numDevices++;
    }

  }

finish:
  return numDevices;
}

