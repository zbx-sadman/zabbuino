#pragma once

/*

Based on https://github.com/Makuna/Rtc/

*/

//DS3231 Register Addresses
#define DS3231_REG_TIMEDATE        (0x00)
#define DS3231_REG_ALARMONE        (0x07)
#define DS3231_REG_ALARMTWO        (0x0B)
                                  
#define DS3231_REG_CONTROL         (0x0E)
#define DS3231_REG_STATUS          (0x0F)
#define DS3231_REG_AGING           (0x10)
                                  
#define DS3231_REG_TEMP            (0x11)

//DS3231 Register Data Size if not just 1
#define DS3231_REG_TIMEDATE_SIZE   (0x07)
#define DS3231_REG_ALARMONE_SIZE   (0x04)
#define DS3231_REG_ALARMTWO_SIZE   (0x03)

#define DS3231_REG_TEMP_SIZE       (0x02)

// DS3231 Control Register Bits
#define DS3231_A1IE                (0x00)
#define DS3231_A2IE                (0x01)
#define DS3231_INTCN               (0x02)
#define DS3231_RS1                 (0x03)
#define DS3231_RS2                 (0x04)
#define DS3231_CONV                (0x05)
#define DS3231_BBSQW               (0x06)
#define DS3231_EOSC                (0x07)
#define DS3231_AIEMASK             (_BV(DS3231_A1IE) | _BV(DS3231_A2IE))
#define DS3231_RSMASK              (_BV(DS3231_RS1)  | _BV(DS3231_RS2))

// DS3231 Status Register Bits
#define DS3231_A1F                 (0x00)
#define DS3231_A2F                 (0x01)
#define DS3231_BSY                 (0x02)
#define DS3231_EN32KHZ             (0x03)
#define DS3231_OSF                 (0x07)
#define DS3231_AIFMASK             (_BV(DS3231_A1F)  | _BV(DS3231_A2F))

#define DS3231_CENTURY_FLAG        (0x80)

/*****************************************************************************************************************************
*
*   Init DS3231 RTC 
*
*   Returns: 
*     - RESULT_IS_OK         on success
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t initDS3231(SoftwareTWI*, const uint8_t);

/*****************************************************************************************************************************
*
*   Save date & time to DS3231
*
*   Returns: 
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t saveDS3231Time(SoftwareTWI*, uint8_t, time_t);

/*****************************************************************************************************************************
*
*   Read date & time from DS3231
*
*   Returns: 
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on read error
*     - DEVICE_ERROR_CONNECT on connection error
*     - actual timestamp returns in _timestamp 
*
*****************************************************************************************************************************/
int8_t readDS3231Time(SoftwareTWI*, uint8_t, time_t*);

                                         
