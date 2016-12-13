#ifndef _ZABBUINO_EEPROM_H_
#define _ZABBUINO_EEPROM_H_

#include <EEPROM.h>
#include "../basic.h"
#include "tune.h"
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

#endif // #ifndef _ZABBUINO_EEPROM_H_