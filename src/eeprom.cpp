// Config & common included files
#include "sys_includes.h"

#include <EEPROM.h>

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
uint8_t saveConfigToEEPROM(netconfig_t *_configStruct)
{
  uint8_t index = 1, // '1' need to enter to the loop
          startAddress, 
          restartWriteCycle, 
          *p_configStruct = (uint8_t*) _configStruct;

  DTSM ( SerialPrintln_P(PSTR("Saving config to EEPROM")); )
  // Calculate CRC of _configStruct and place it to first byte of structure to batch writing.
  // CRC-byte must be skipped on CRC calculating
  _configStruct->CRC = dallas_crc8(((uint8_t*) _configStruct) + CONFIG_CRC_LEN, sizeof(netconfig_t));

  // Save every byte of _configStruct to EEPROM

  // Read config store start address from the EEPROM, validate it:  default_start_address < startAddress < (last_eeprom_cell_address - config_structure_size) 
  // If start address point to out of bound cell (first writing to new MCU, for example) - use default value.
  startAddress = EEPROM[CONFIG_STORE_PTR_ADDRESS];
  if ((LAST_EEPROM_CELL_ADDRESS - sizeof(netconfig_t)) < startAddress || CONFIG_STORE_DEFAULT_START_ADDRESS > startAddress) { 
     startAddress = CONFIG_STORE_DEFAULT_START_ADDRESS;
  }

  // if (0 != index) - corrupted cells is detected (or just first write loop executed) and need to restart config write procedure
  while (index) {
     // Writing procedure must be stopped by return operator if EEPROM space is not enought to save sizeof() bytes of config
     if (startAddress > (LAST_EEPROM_CELL_ADDRESS - sizeof(netconfig_t))) { 
        DTSM ( SerialPrintln_P(PSTR("There is not room to save config")); )
        return false;
     }

     restartWriteCycle = false;
     index = sizeof(netconfig_t);
     //Serial.print("Need to write: "); Serial.print(index); Serial.println(" bytes");
          
     // Operations must be repeated until all bytes of config structure not saved and no write errors found
     while (index && !restartWriteCycle) {
       index--;
       // Just simulate EEPROM.update():
       // Write only changed data and immediately tests the written byte. On testing error - restart write cycle with new start address.
       //Serial.print("0x"); Serial.print(p_configStruct[index]); Serial.print(" => '"); Serial.print((char) p_configStruct[index]); Serial.print("'"); 
       if (EEPROM[index + startAddress] != p_configStruct[index]) {
          //Serial.println(" [W]"); 
          EEPROM[index + startAddress] = p_configStruct[index];
          if (EEPROM[index + startAddress] == p_configStruct[index]) { continue; }
          DTSM ( SerialPrintln_P(PSTR("Probaly EEPROM cell is corrupted...")); )
          startAddress = startAddress + index + 1;
          restartWriteCycle = true;
       } // if (EEPROM[index + startAddress] != p_configStruct[index])
       //Serial.println(" [U]"); 

     } // while (index && !restartWriteCycle)
  } // while (index)

  // Update store config start address if need and return false on error
  if (startAddress != EEPROM[CONFIG_STORE_PTR_ADDRESS]) {
     EEPROM[CONFIG_STORE_PTR_ADDRESS] = startAddress;
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
uint8_t loadConfigFromEEPROM(netconfig_t *_configStruct) 
{
  
  uint8_t index, 
          startAddress, 
          *p_configStruct = (uint8_t*) _configStruct;

  DTSM ( SerialPrintln_P(PSTR("Load configuration from EEPROM")); )
  // Read the pointer of config store start address and validate it: default_start_address < startAddress < (last_eeprom_cell_address - config_structure_size) 
  // On error - stop working
  startAddress = EEPROM[CONFIG_STORE_PTR_ADDRESS];
  if ((LAST_EEPROM_CELL_ADDRESS - sizeof(netconfig_t)) < startAddress || CONFIG_STORE_DEFAULT_START_ADDRESS > startAddress) { 
     return false; 
  }

  // Read all bytes from EEPROM to config structure
  // no sense to call AVR functions directly - sketch still uses the same size of program storage space and RAM.
  // p_configStruct[index] = eeprom_read_byte((uint8_t *) index);
  // EEPROM.read(index) eq EEPROM[index], but last is looks nice
  index = sizeof(netconfig_t);
  while (index) {
     index--;
     p_configStruct[index] = EEPROM[startAddress + index];
  }
                                    
  // Comparing loaded and calculated CRC and return false if its not equal
  // First byte of structure is CRC, it's must be skipped on CRC calculating
  if (dallas_crc8(((uint8_t*) _configStruct) + CONFIG_CRC_LEN, sizeof(netconfig_t)) != _configStruct->CRC) {
     return false;
  } 

  return true;
}


