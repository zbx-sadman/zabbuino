#ifndef _ZABBUINO_OWBUS_H_
#define _ZABBUINO_OWBUS_H_

// OneWire lib for Dallas sensors
#include <OneWire.h>
#include "../basic.h"
#include "tune.h"
#include "service.h"
#include "transport.h"

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


#endif // #ifndef _ZABBUINO_OWBUS_H_