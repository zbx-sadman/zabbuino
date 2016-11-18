#ifndef ZabbuinoEEPROM_h
#define ZabbuinoEEPROM_h

#include <Arduino.h>
#include <IPAddress.h>
#include <EEPROM.h>
#include "defaults.h"
#include "../zabbuino.h"
#include "service.h"

/*****************************************************************************************************************************
*
*   Save/Update config to EEPROM
*   On detect EEPROM cell corruption truing to find a new storage area 
*
*   Returns: 
*     - true on success
*     - false on fail
*
*****************************************************************************************************************************/
uint8_t saveConfigToEEPROM(netconfig_t *_configStruct);

/*****************************************************************************************************************************
*
*   Read config from EEPROM
*
*   Returns: 
*     - true on success
*     - false on fail
*
*****************************************************************************************************************************/
uint8_t loadConfigFromEEPROM(netconfig_t *_configStruct);

#endif // #ifndef ZabbuinoEEPROM_h