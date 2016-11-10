#include "eeprom.h"

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
uint8_t saveConfigToEEPROM(netconfig_t* _configStruct)
{
  uint8_t index, startAddress, restartWriteCycle, *p_configStruct = (uint8_t*) _configStruct;
  // Calculate CRC of _configStruct and place it to structure to batch writing
#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrintln_P(PSTR("Saving config to EEPROM"));
#endif
  // First byte of structure is CRC, it's must be skipped on CRC calculating
  _configStruct->CRC = dallas_crc8(((uint8_t*) _configStruct) + CONFIG_CRC_LEN, sizeof(netconfig_t));

  // Save every byte of _configStruct to EEPROM
  startAddress = EEPROM[CONFIG_STORE_PTR_ADDRESS];
  // Validate startAddress:  default_start_address < startAddress < (last_eeprom_cell_address - config_structure_size) 
  if ((LAST_EEPROM_CELL_ADDRESS - sizeof(netconfig_t)) < startAddress || CONFIG_STORE_DEFAULT_START_ADDRESS > startAddress) { 
     // Point to default start address if readed value is invalid on first writing to new MCU, for example
     startAddress = CONFIG_STORE_DEFAULT_START_ADDRESS;
  }
  //
  do {

     // EEPROM space enougt to save sizeof() bytes of config?
     if (startAddress > (LAST_EEPROM_CELL_ADDRESS - sizeof(netconfig_t))) { 
        // No, writing must be stopped
#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrintln_P(PSTR("There is not room to save config"));
#endif
        return false;
     }
     restartWriteCycle = false;
     index = sizeof(netconfig_t);
     do {
       index--;
       // Just simulate EEPROM.update()
       if (EEPROM[index + startAddress] != p_configStruct[index]) {
          // write byte
          EEPROM[index + startAddress] = p_configStruct[index];
          // Test to write error
          if (EEPROM[index + startAddress] != p_configStruct[index]) {
             // EEPROM have a lot bytes to config save?
#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrintln_P(PSTR("Probaly EEPROM cell is corrupted..."));
#endif
             // Try to save config again starts on next position of broken cell
             startAddress = startAddress + index + 1;
             restartWriteCycle = true;
          } // if (EEPROM[index + startAddress] != p_configStruct[index])
       } // if (EEPROM[index + startAddress] != p_configStruct[index])
     } while (index && !restartWriteCycle); // Operations must be repeated until all bytes of config structure not saved and no write errors found
  // if (0 != index) - corrupted cells is detected and need to restart config write procedure
  } while (index);

  // Update store config start address if need
  if (startAddress != EEPROM[CONFIG_STORE_PTR_ADDRESS]) {
     EEPROM[CONFIG_STORE_PTR_ADDRESS] = startAddress;
     // Pointer's value is saved succesfully?
     if (startAddress != EEPROM[CONFIG_STORE_PTR_ADDRESS]) { return false; }
  };

  return true;

}

/*****************************************************************************************************************************
*
*   Read config from EEPROM
*
*   Returns: 
*     - true on success
*     - false on fail
*
*****************************************************************************************************************************/
uint8_t loadConfigFromEEPROM(netconfig_t* _configStruct)
{
  
  uint8_t index, startAddress, *p_configStruct = (uint8_t*) _configStruct;
#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrintln_P(PSTR("Load configuration from EEPROM"));
#endif
  // Load _configStruct from EEPROM
  // Read the pointer of config store start address 
  startAddress = EEPROM[CONFIG_STORE_PTR_ADDRESS];
  // Validate startAddress:  default_start_address < startAddress < (last_eeprom_cell_address - config_structure_size) 
  if ((LAST_EEPROM_CELL_ADDRESS - sizeof(netconfig_t)) < startAddress || CONFIG_STORE_DEFAULT_START_ADDRESS > startAddress) { 
     // Wrong start address
     return false; 
  }

  // Read bytes from EEPROM to config structure
  index = sizeof(netconfig_t);
  do {
     index--;
    // no sense to call AVR functions directly - sketch still uses the same size of program storage space and RAM.
    // p_configStruct[index] = eeprom_read_byte((uint8_t *) index);
    // EEPROM.read(index) eq EEPROM[index], but last is looks better
    p_configStruct[index] = EEPROM[startAddress + index];
  } while (index);
                                    
  // Comparing loaded and calculated CRC
  // First byte of structure is CRC, it's must be skipped on CRC calculating
  if (dallas_crc8(((uint8_t*) _configStruct) + CONFIG_CRC_LEN, sizeof(netconfig_t)) != _configStruct->CRC) {
     // CRC is not equal
     return false;
  } 

  // True, if config loading finished successfully
  return true;

}


