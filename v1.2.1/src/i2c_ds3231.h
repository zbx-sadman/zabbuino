/*

Based on https://github.com/Makuna/Rtc/

*/

#ifndef _ZABBUINO_I2C_DS3231_H_
#define _ZABBUINO_I2C_DS3231_H_

#include <time.h>


//DS3231 Register Addresses
#define DS3231_REG_TIMEDATE        0x00
#define DS3231_REG_ALARMONE        0x07
#define DS3231_REG_ALARMTWO        0x0B
                                  
#define DS3231_REG_CONTROL         0x0E
#define DS3231_REG_STATUS          0x0F
#define DS3231_REG_AGING           0x10
                                  
#define DS3231_REG_TEMP            0x11

//DS3231 Register Data Size if not just 1
#define DS3231_REG_TIMEDATE_SIZE   7
#define DS3231_REG_ALARMONE_SIZE   4
#define DS3231_REG_ALARMTWO_SIZE   3

#define DS3231_REG_TEMP_SIZE       2

// DS3231 Control Register Bits
#define DS3231_A1IE                0
#define DS3231_A2IE                1
#define DS3231_INTCN               2
#define DS3231_RS1                 3
#define DS3231_RS2                 4
#define DS3231_CONV                5
#define DS3231_BBSQW               6
#define DS3231_EOSC                7
#define DS3231_AIEMASK             (_BV(DS3231_A1IE) | _BV(DS3231_A2IE))
#define DS3231_RSMASK              (_BV(DS3231_RS1) | _BV(DS3231_RS2))

// DS3231 Status Register Bits
#define DS3231_A1F                 0
#define DS3231_A2F                 1
#define DS3231_BSY                 2
#define DS3231_EN32KHZ             3
#define DS3231_OSF                 7
#define DS3231_AIFMASK             (_BV(DS3231_A1F) | _BV(DS3231_A2F))

/*****************************************************************************************************************************
*
*   Init DS3231 RTC 
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t initDS3231(SoftwareWire*, const uint8_t);

/*****************************************************************************************************************************
*
*   Save date & time to DS3231
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t saveDS3231Time(SoftwareWire*, uint8_t, time_t);

/*****************************************************************************************************************************
*
*   Read date & time from DS3231
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on read error
*     - DEVICE_ERROR_CONNECT on connection error
*     - actual timestamp returns in _timestamp 
*
*****************************************************************************************************************************/
int8_t readDS3231Time(SoftwareWire*, uint8_t, time_t*);


#endif // #ifndef _ZABBUINO_I2C_DS3231_H_