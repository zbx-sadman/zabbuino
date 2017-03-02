/*

Based on https://github.com/Makuna/Rtc/

*/

#ifndef _ZABBUINO_I2C_DS3231_H_
#define _ZABBUINO_I2C_DS3231_H_

#include <time.h>

#pragma pack(push,1)
// First 7 bytes of DS3231 address map (datasheet, pg.11)
typedef struct {
// 00h
  uint8_t seconds:   4;  // seconds bits [0..3]
  uint8_t seconds10: 3;  // tens of seconds bits [4..6]
  uint8_t secondsU:  1;  // unused bit [7]
// 01h
  uint8_t minutes:   4;  // minutes bits
  uint8_t minutes10: 3;  // minutes of seconds bits
  uint8_t minutesU:  1;  // unused bit
// 02h
  uint8_t hour:      4;  // hour bits
  uint8_t hour10:    1;  // tens hour bit
  uint8_t hour20:    1;  // 20-23 hour bit for 24H mode or AM/PM bit for 12H mode
  uint8_t hour1224:  1;  // 24H/12H mode bit
  uint8_t hourU:     1;  // unused bit
// 03h
  uint8_t day:       3;  // day of week bits, 1 equals Sunday
  uint8_t dayU:      5;  // unused bits
// 04h
  uint8_t date:      4;  // day of month bits
  uint8_t date10:    2;  // tens of day of month bits
  uint8_t dateU:     2;  // unused bits
// 05h
  uint8_t month:     4;  // month bits
  uint8_t month10:   1;  // tens of month bits
  uint8_t monthU:    2;  // unused bits
  uint8_t century:   1;  // century bit. Is toggled when the years register overflows from 99 to 00. 
// 06h
  uint8_t year:      4;  // year bits
  uint8_t year10:    4;  // tens of year bits
} DS3231DateTime_t;
#pragma pack(pop) 


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
*   Put date & time to RTC
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setDateTime(const uint8_t, const uint8_t, uint8_t, uint32_t);

/*****************************************************************************************************************************
*
*   Get date & time from RTC
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on read error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t getDateTime(const uint8_t, const uint8_t, uint8_t, tm*);

/*****************************************************************************************************************************
*
*   Get local time as Unix timestamp
*
*   Returns: 
*     - RESULT_IN_LONGVAR on success
*     - RESULT_IS_FAIL on read error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t getLocalTime(const uint8_t, const uint8_t, uint8_t, int32_t*);

/*****************************************************************************************************************************
*
*   Set local time from Unix timestamp
*
*   Returns: 
*     - RESULT_IN_OK on success
*     - RESULT_IS_FAIL on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setLocalTime(const uint8_t, const uint8_t, uint8_t, int32_t);

/*****************************************************************************************************************************
*
*   Convert the number from uint8_t to BCD format
*
*   Returns: 
*     - BCD value
*
*****************************************************************************************************************************/
uint8_t Uint8ToBcd(uint8_t val);

/*****************************************************************************************************************************
*
*   Convert the number from BCD format to uint8_t
*
*   Returns: 
*     - unit8_t value
*
*****************************************************************************************************************************/
uint8_t BcdToUint8(uint8_t val);

/*****************************************************************************************************************************
*
*   Convert the number from BCD format to 24H format
*
*   Returns: 
*     - 24H format value
*
*****************************************************************************************************************************/
uint8_t BcdToBin24Hour(uint8_t bcdHour);


uint8_t IsDateTimeValid(const uint8_t _i2cAddress);
uint8_t GetIsRunning(const uint8_t _i2cAddress);
void SetIsRunning(const uint8_t _i2cAddress, uint8_t _isRunning);

#endif // #ifndef _ZABBUINO_I2C_DS3231_H_