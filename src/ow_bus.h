#ifndef ZabbuinoBUSONEWIRE_h
#define ZabbuinoBUSONEWIRE_h

#include <Arduino.h>
// OneWire lib for Dallas sensors
#include <OneWire.h>
#include "../zabbuino.h"
#include "defaults.h"
#include "service.h"

/*****************************************************************************************************************************
*
*   Scan 1-Wire bus and print to ethernet client ID's (Addresses) of all detected devices
*
*   Returns: 
*     - RESULT_IS_PRINTED on success
*     - RESULT_IS_FAIL of no devices found 
*
*****************************************************************************************************************************/
int8_t scanOneWire(const uint8_t _pin, EthernetClient *_ethClient);


#endif