#ifndef _ZABBUINO_I2C_RTC_H_
#define _ZABBUINO_I2C_RTC_H_

#include <time.h>
#include "i2c_ds3231.h"
#include "i2c_at24c32.h"

/*****************************************************************************************************************************
*
*   Init RTC 
*
*   Returns: 
*     - 
*
*****************************************************************************************************************************/
void initRTC(const uint8_t, const uint8_t, const uint8_t, const uint8_t);

/*****************************************************************************************************************************
*
*   Get UTC time as Unix timestamp
*
*   Returns: 
*     - RESULT_IN_LONGVAR on success
*     - RESULT_IS_FAIL on read error
*     - DEVICE_ERROR_CONNECT on connection error
*     - actual timestamp returns in _unixTimestamp
*
*****************************************************************************************************************************/
int8_t getUnixTime(const uint8_t, const uint8_t, uint8_t, uint32_t*);

/*****************************************************************************************************************************
*
*   Get UTC time as Y2K timestamp
*
*   Returns: 
*     - RESULT_IN_LONGVAR on success
*     - RESULT_IS_FAIL on read error
*     - DEVICE_ERROR_CONNECT on connection error
*     - actual timestamp returns in _Y2KTimestamp
*
*****************************************************************************************************************************/
int8_t getY2KTime(const uint8_t, const uint8_t, uint8_t, time_t*);

/*****************************************************************************************************************************
*
*   Set UTC time taking Unix timestamp
*
*   Returns: 
*     - RESULT_IN_OK on success
*     - RESULT_IS_FAIL on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setUnixTime(const uint8_t, const uint8_t, uint8_t, uint32_t);

/*****************************************************************************************************************************
*
*   Set UTC time taking Y2K timestamp
*
*   Returns: 
*     - RESULT_IN_OK on success
*     - RESULT_IS_FAIL on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setY2KTime(const uint8_t, const uint8_t, uint8_t, time_t);

/*****************************************************************************************************************************
*
*   Set TimeZone offset (in seconds). Actually - just store it in DS3231 module's onboard EEPROM (AT24C32).
*
*   Returns: 
*     - RESULT_IN_OK on success
*     - RESULT_IS_FAIL on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setTZOffset(const uint8_t, const uint8_t, uint8_t, int16_t);

/*****************************************************************************************************************************
*
*   Get TimeZone offset (in seconds). Actually - just read it from DS3231 module's onboard EEPROM (AT24C32).
*
*   Returns: 
*     - RESULT_IN_OK on success
*     - RESULT_IS_FAIL on read error
*     - DEVICE_ERROR_CONNECT on connection error
*     - actual timezone offset returns in _tzOffset
*
*****************************************************************************************************************************/
int8_t getTZOffset(const uint8_t, const uint8_t, uint8_t, int16_t*);

#endif // _ZABBUINO_I2C_RTC_H_