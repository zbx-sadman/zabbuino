#pragma once

/*****************************************************************************************************************************
*
*   Convert the number from uint8_t to BCD format
*
*   Returns: 
*     - BCD value
*
*****************************************************************************************************************************/
uint8_t Uint8ToBcd(uint8_t);

/*****************************************************************************************************************************
*
*   Convert the number from BCD format to uint8_t
*
*   Returns: 
*     - unit8_t value
*
*****************************************************************************************************************************/
uint8_t BcdToUint8(uint8_t);

/*****************************************************************************************************************************
*
*   Init RTC 
*
*   Returns: 
*     - True on success
*     - False otherwise  
*
*****************************************************************************************************************************/
int8_t initRTC(SoftwareWire*);

/*****************************************************************************************************************************
*
*   Set UTC time using Y2K timestamp
*
*   Returns: 
*     - RESULT_IN_OK on success
*     - RESULT_IS_FAIL on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setY2KTime(SoftwareWire*, time_t);

/*****************************************************************************************************************************
*
*   Get UTC time as Y2K timestamp
*
*   Returns: 
*     - RESULT_IS_SIGNED_VALUE on success
*     - RESULT_IS_FAIL on read error
*     - DEVICE_ERROR_CONNECT on connection error
*     - actual timestamp returns in _Y2KTimestamp
*
*****************************************************************************************************************************/
int8_t getY2KTime(SoftwareWire*, time_t*);

/*****************************************************************************************************************************
*
*   Set UTC time using Unix timestamp
*
*   Returns: 
*     - RESULT_IN_OK on success
*     - RESULT_IS_FAIL on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setUnixTime(SoftwareWire*, uint32_t);

/*****************************************************************************************************************************
*
*   Get UTC time as Unix timestamp
*
*   Returns: 
*     - RESULT_IS_SIGNED_VALUE on success
*     - RESULT_IS_FAIL on read error
*     - DEVICE_ERROR_CONNECT on connection error
*     - actual timestamp returns in _unixTimestamp
*
*****************************************************************************************************************************/
int8_t getUnixTime(SoftwareWire*, uint32_t*);

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
//int8_t setTZOffset(const uint8_t, const uint8_t, uint8_t, int16_t);

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
//int8_t getTZOffset(const uint8_t, const uint8_t, uint8_t, int16_t*);


