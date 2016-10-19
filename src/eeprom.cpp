#include "eeprom.h"

/* ****************************************************************************************************************************
*
*   Save config structure to EEPROM
*
**************************************************************************************************************************** */

uint8_t saveConfigToEEPROM(const netconfig_t* _configStruct)
{
  uint8_t index, *p_configStruct = (uint8_t*) _configStruct;
  // Save every byte of _configStruct to EEPROM
  index = sizeof(netconfig_t);
  while (index) {
    index--;
    // Just simulate EEPROM.update()
    if (EEPROM[index] != p_configStruct[index]) {
       EEPROM[index] = p_configStruct[index];
    }
  }
  // Calculate CRC of _configStruct and store it to EEPROM after all config data
  EEPROM[sizeof(netconfig_t)+1]= dallas_crc8(p_configStruct, sizeof(netconfig_t));

}

/* ****************************************************************************************************************************
*
*   Read config structure from EEPROM
*
**************************************************************************************************************************** */

uint8_t loadConfigFromEEPROM(netconfig_t* _configStruct)
{
  
  uint8_t index, storedCRC, calculatedCRC, *p_configStruct = (uint8_t*) _configStruct;

  // Load _configStruct from EEPROM
  for (index = 0; index < sizeof(netconfig_t); index++) {
    // no sense to call AVR functions directly - sketch still uses the same size of program storage space and RAM.
    // p_configStruct[index] = eeprom_read_byte((uint8_t *) index);
    // EEPROM.read(index) eq EEPROM[index], but last is looks better
    p_configStruct[index] = EEPROM[index];
  }

  // Calculate CRC of _configStruct which load from EEPROM.
  calculatedCRC = dallas_crc8 (p_configStruct, sizeof(netconfig_t));
  // Read _configStruct CRC which early stored to EEPROM
  storedCRC = EEPROM[index+1];
  // Check integrity of loaded structure...
  if (storedCRC == calculatedCRC) {
     // True, if loading finished successfully
     return true;
  }  // Otherwise - False
  return false;

}


