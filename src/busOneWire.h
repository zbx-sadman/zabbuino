#ifndef ZabbuinoBUSONEWIRE_h
#define ZabbuinoBUSONEWIRE_h

#include <Arduino.h>
// OneWire lib for Dallas sensors
#include <OneWire.h>
#include "defaults.h"
#include "../zabbuino.h"
#include "service.h"


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     DS18x20 SECTION
*/

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

/* ****************************************************************************************************************************
*
*   Scan 1-Wire bus and print to ethernet client all detected ID's (Addresses)
*
**************************************************************************************************************************** */
int8_t scanOneWire(const uint8_t _pin, EthernetClient *_ethClient);
//int32_t scanOneWire(const uint8_t _pin);


/* ****************************************************************************************************************************
*
*   Read temperature from digital sensor Dallas DS18x20 family
*
*   Subroutine is tested with DS18B20 only. 
*   Probably you can meet problems with the correct calculation of temperature due to incorrect 'tRaw' adjustment 
*
**************************************************************************************************************************** */
int8_t getDS18X20Metric(const uint8_t _pin, uint8_t _resolution, char* _id, char* _outBuffer);

uint8_t getScratchPadFromDS18X20(OneWire* _owDevice, const uint8_t* _addr, uint8_t* _scratchPad);

#endif