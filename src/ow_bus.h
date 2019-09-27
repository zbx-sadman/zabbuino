#pragma once

#define ONEWIRE_ID_SIZE                                         (0x08)

/*****************************************************************************************************************************
*
*   Scan 1-Wire bus and print to ethernet client ID's (Addresses) of all detected devices
*
*   Returns: 
*     - RESULT_IS_PRINTED on success
*     - RESULT_IS_FAIL of no devices found 
*
*****************************************************************************************************************************/
int8_t scanOneWire(const uint8_t, uint8_t*, size_t);

