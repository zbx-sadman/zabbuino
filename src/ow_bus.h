#ifndef ZabbuinoBUSONEWIRE_h
#define ZabbuinoBUSONEWIRE_h

#include <Arduino.h>
// OneWire lib for Dallas sensors
#include <OneWire.h>
#include "../zabbuino.h"
#include "defaults.h"
#include "service.h"

/* ****************************************************************************************************************************
*
*   Scan 1-Wire bus and print to ethernet client all detected ID's (Addresses)
*
**************************************************************************************************************************** */
int8_t scanOneWire(const uint8_t _pin, EthernetClient *_ethClient);
//int32_t scanOneWire(const uint8_t _pin);


#endif