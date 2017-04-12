#include "ow_bus.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
*
*                                                                     COMMON 1-WIRE SECTION
*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/


/*****************************************************************************************************************************
*
*   Scan 1-Wire bus and print to ethernet client ID's (Addresses) of all detected devices
*
*   Returns: 
*     - RESULT_IS_PRINTED on success
*     - RESULT_IS_FAIL of no devices found 
*
*****************************************************************************************************************************/
int8_t scanOneWire(const uint8_t _pin, NetworkClass *_network) {
//#if !defined(NETWORK_RS485)
  uint8_t dsAddr[8], numDevices = 0, i;
  OneWire owDevice(_pin);
  owDevice.reset_search();
  delay(250);
  owDevice.reset();
  while (owDevice.search(dsAddr)) {
    numDevices++;
    _network->client.print("0x");
    DTSL ( Serial.print("0x"); ) 
    for (i = 0; i < arraySize(dsAddr); i++ ) {
      if (dsAddr[i] < 0x10) {
         _network->client.print("0"); 
         DTSL ( Serial.print("0"); ) 
      }
      _network->client.print(dsAddr[i], HEX);
      DTSL ( Serial.print(dsAddr[i], HEX); ) 
    }
    _network->client.print('\n');
    DTSL ( Serial.print('\n'); )
  }
  return ((0 < numDevices) ? RESULT_IS_PRINTED : RESULT_IS_FAIL);
/*
#else
  return (RESULT_IS_FAIL);
#endif
*/
}


