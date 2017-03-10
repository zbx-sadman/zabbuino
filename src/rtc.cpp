#include "rtc.h"
#include "i2c_bus.h"



/*****************************************************************************************************************************
*
*   Init RTC 
*
*   Returns: 
*     - 
*
*****************************************************************************************************************************/
//void initRTC(const uint8_t _sdaPin, const uint8_t _sclPin, const uint8_t _rtcI2CAddress, const uint8_t _eepromI2CAddress) {
void initRTC(const uint8_t _sdaPin, const uint8_t _sclPin, const uint8_t _rtcI2CAddress) {
  // Init RTC chip
  initDS3231(_sdaPin, _sclPin, _rtcI2CAddress);
/*
#ifdef FEATURE_SYSTEM_RTC_ONBOARD_EEPROM_ENABLE
  int16_t tzOffset;
  //initAT24C32(_sdaPin, _sclPin, _eepromI2CAddress);
  if (getTZOffset(_sdaPin, _sclPin, _eepromI2CAddress, &tzOffset)) {
     set_zone(tzOffset); 
  }
#endif 
*/
}
   
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
int8_t getUnixTime(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint32_t* _unixTimestamp) {
  int8_t rc;
  rc = getY2KTime(_sdaPin, _sclPin, _i2cAddress, (time_t*) _unixTimestamp);
  *_unixTimestamp += UNIX_OFFSET;
  return rc;
}

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
int8_t getY2KTime(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, time_t* _Y2KTimestamp) {
  return readDS3231Time(_sdaPin, _sclPin, _i2cAddress, (time_t*) _Y2KTimestamp);
}

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
int8_t setUnixTime(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint32_t _unixTimestamp) {
  return saveDS3231Time(_sdaPin, _sclPin, _i2cAddress, (time_t) (_unixTimestamp - UNIX_OFFSET));
}

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
int8_t setY2KTime(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, time_t _Y2KTimestamp) {
  return saveDS3231Time(_sdaPin, _sclPin, _i2cAddress, _Y2KTimestamp);
}

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
/*
int8_t setTZOffset(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, int16_t _tzOffset) {
#ifdef FEATURE_SYSTEM_RTC_ONBOARD_EEPROM_ENABLE
  int8_t rc = false;
  rc = saveAT24C32TZOffset(_sdaPin, _sclPin, _i2cAddress, _tzOffset);
  if (rc) { set_zone(_tzOffset); }
  return rc;
#else
  return true;
#endif
}
*/
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
/*
int8_t getTZOffset(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, int16_t* _tzOffset) {
#ifdef FEATURE_SYSTEM_RTC_ONBOARD_EEPROM_ENABLE
  return readAT24C32TZOffset(_sdaPin, _sclPin, _i2cAddress, _tzOffset);
#else
  return true;
#endif
}
*/
