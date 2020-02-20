#pragma once

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

