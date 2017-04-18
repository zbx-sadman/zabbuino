#ifndef _ZABBUINO_EEPROM_H_
#define _ZABBUINO_EEPROM_H_

#include <EEPROM.h>
#include "../basic.h"
#include "tune.h"
#include "service.h"

/*****************************************************************************************************************************
*
*   Save/Update config to EEPROM
*   On detecting EEPROM cell corruption trying to find a new storage area 
*  
*   Returns: 
*     - true on success
*     - false on fail
*  
*****************************************************************************************************************************/
uint8_t saveConfigToEEPROM(netconfig_t*);

/*****************************************************************************************************************************
*
*   Read config from EEPROM
*
*   Returns: 
*     - true on success
*     - false on fail
*
*****************************************************************************************************************************/
uint8_t loadConfigFromEEPROM(netconfig_t*);

#endif // #ifndef _ZABBUINO_EEPROM_H_