// Config & common included files
#include "sys_includes.h"

// OneWire lib for Dallas sensors
#include "OneWire/OneWire.h"

#include "service.h"

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
int8_t scanOneWire(const uint8_t _pin, uint8_t* _dst, size_t _bufferSize) {
  uint8_t dsAddr[ONEWIRE_ID_SIZE], 
          numDevices = 0x00;

  OneWire owDevice(_pin);

  // Test the bus
  if (!owDevice.reset()) { goto finish; }

  owDevice.reset_search();
  // Do not write more that _bufferSize (buffer size)
  while (owDevice.search(dsAddr)) {
    yield();
    if (_bufferSize >= sizeof(dsAddr)) { 
       memcpy(_dst, dsAddr, ONEWIRE_ID_SIZE);
       _bufferSize -= ONEWIRE_ID_SIZE;
       _dst += ONEWIRE_ID_SIZE;
        numDevices++;
    }
  }

finish:
  return numDevices;
}

