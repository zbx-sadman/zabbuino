#ifndef _ZABBUINO_OW_SENSORS_H_
#define _ZABBUINO_OW_SENSORS_H_

#include "ow_bus.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                                   DS18x20 SECTION

   Based on: Dallas Temperature Control Library
   Version 3.7.2  is used

 -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

// Model IDs
#define DS18S20_ID                                              0x10  // also DS1820
#define DS18B20_ID                                              0x28
#define DS1822_ID                                               0x22
#define DS1825_ID                                               0x3B

// Device resolution
#define DS18X20_MODE_9_BIT                                      0x1F //  9 bit
#define DS18X20_MODE_10_BIT                                     0x3F // 10 bit
#define DS18X20_MODE_11_BIT                                     0x5F // 11 bit
#define DS18X20_MODE_12_BIT                                     0x7F // 12 bit


// OneWire commands
#define DS18X20_CMD_STARTCONVO                                  0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define DS18X20_CMD_COPYSCRATCH                                 0x48  // Copy EEPROM
#define DS18X20_CMD_READSCRATCH                                 0xBE  // Read EEPROM
#define DS18X20_CMD_WRITESCRATCH                                0x4E  // Write to EEPROM
#define DS18X20_CMD_RECALLSCRATCH                               0xB8  // Reload from last known
#define DS18X20_CMD_READPOWERSUPPLY                             0xB4  // Determine if device needs parasite power
#define DS18X20_CMD_ALARMSEARCH                                 0xEC  // Query bus for devices with an alarm condition


// Scratchpad locations
#define DS18X20_BYTE_TEMP_LSB                                   0x00
#define DS18X20_BYTE_TEMP_MSB                                   0x01
#define DS18X20_BYTE_HIGH_ALARM_TEMP                            0x02
#define DS18X20_BYTE_LOW_ALARM_TEMP                             0x03
#define DS18X20_BYTE_CONFIGURATION                              0x04
#define DS18X20_BYTE_INTERNAL_BYTE                              0x05
#define DS18X20_BYTE_COUNT_REMAIN                               0x06
#define DS18X20_BYTE_COUNT_PER_C                                0x07
#define DS18X20_BYTE_SCRATCHPAD_CRC                             0x08


/*****************************************************************************************************************************
*
*  Read specified metric's value of the digital sensor of Dallas DS18x20 family, put it to output buffer on success. 
*
*  Note: subroutine is tested with DS18B20 only. 
*        probably you can meet problems with the correct calculation of temperature due to incorrect 'tRaw' adjustment 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_CONNECT on connection error
*     - DEVICE_ERROR_CHECKSUM on detect data corruption
*
*****************************************************************************************************************************/
int8_t getDS18X20Metric(const uint8_t, uint8_t, char*, char*);

/*****************************************************************************************************************************
*
*  Read DS18x20's scratchpad
*
*   Returns: 
*     - true on success
*     - false on fail
*
*****************************************************************************************************************************/
uint8_t getScratchPadFromDevice(OneWire*, const uint8_t*, uint8_t*);

inline uint8_t isCRCOK(uint8_t*);
#endif // #ifndef _ZABBUINO_OW_SENSORS_H_