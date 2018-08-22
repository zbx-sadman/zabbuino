// Config & common included files
#include "sys_includes.h"

// OneWire lib for Dallas sensors
#include <OneWire.h>

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
int8_t scanOneWire(const uint8_t _pin, uint8_t* _dst) {
//#if !defined(NETWORK_RS485)
  uint8_t dsAddr[8], 
          numDevices = 0;
  OneWire owDevice(_pin);

  owDevice.reset_search();
  delay(250);
  owDevice.reset();
  while (owDevice.search(dsAddr)) {
    memcpy(_dst, dsAddr, sizeof(dsAddr));
    _dst += sizeof(dsAddr);
    ++numDevices;
  }
  return numDevices;
}


/*

    if (_networkStream) { _networkStream->print("0x"); }
    DTSL ( Serial.print("0x"); ) 
    for (i = 0; i < arraySize(dsAddr); i++ ) {
      if (dsAddr[i] < 0x10) {
         if (_networkStream) { _networkStream->print("0"); }
         DTSL ( Serial.print("0"); ) 
      }
      if (_networkStream) { _networkStream->print(dsAddr[i], HEX); }
      DTSL ( Serial.print(dsAddr[i], HEX); ) 
    }
    if (_networkStream) { _networkStream->print('\n'); }
    DTSL ( Serial.print('\n'); )

*/