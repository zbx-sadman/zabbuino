// Config & common included files
#include "sys_includes.h"

#include <time.h>
#include "SoftwareWire/SoftwareWire.h"

#include "i2c_bus.h"
#include "i2c_ds3231.h"
#include "i2c_pcf8563.h"
#include "rtc.h"

/*****************************************************************************************************************************
*
*   Convert the number from uint8_t to BCD format
*
*   Returns: 
*     - BCD value
*
*****************************************************************************************************************************/
uint8_t Uint8ToBcd(uint8_t val) {
  return val + 6 * (val / 10);
}

/*****************************************************************************************************************************
*
*   Convert the number from BCD format to uint8_t
*
*   Returns: 
*     - unit8_t value
*
*****************************************************************************************************************************/
uint8_t BcdToUint8(uint8_t val) {
  return val - 6 * (val >> 4);
}

/*****************************************************************************************************************************
*
*   Init RTC 
*
*   Returns: 
*     - 
*
*****************************************************************************************************************************/
//void initRTC(const uint8_t _sdaPin, const uint8_t _sclPin, const uint8_t _rtcI2CAddress, const uint8_t _eepromI2CAddress) {
int8_t initRTC(SoftwareWire* _softTWI) {
  int8_t rc = false;
  // Init RTC chip
  _softTWI->reconfigure(constSystemRtcSDAPin, constSystemRtcSCLPin);
#ifdef FEATURE_SYSTEM_RTC_DS3231_ENABLE
  rc = initDS3231(_softTWI, constSystemRtcI2CAddress);
#elif defined (FEATURE_SYSTEM_RTC_PCF8563_ENABLE)
  rc = initPCF8563(_softTWI, constSystemRtcI2CAddress);
#endif
  return rc;
}
   
/*****************************************************************************************************************************
*
*   Set UTC time taking Y2K timestamp
*
*   Returns: 
*     - True on success
*     - False on error
*
*****************************************************************************************************************************/
int8_t setY2KTime(SoftwareWire* _softTWI, time_t _Y2KTimestamp) {
  int8_t rc = false;
  _softTWI->reconfigure(constSystemRtcSDAPin, constSystemRtcSCLPin);
#ifdef FEATURE_SYSTEM_RTC_DS3231_ENABLE
  rc = saveDS3231Time(_softTWI, constSystemRtcI2CAddress, _Y2KTimestamp);
#elif defined (FEATURE_SYSTEM_RTC_PCF8563_ENABLE)
  rc = savePCF8563Time(_softTWI, constSystemRtcI2CAddress, _Y2KTimestamp);
#endif
  return rc;
}

/*****************************************************************************************************************************
*
*   Get UTC time as Y2K timestamp
*
*   Returns: 
*     - True on success
*     - False on error
*     - actual timestamp returns in _Y2KTimestamp
*
*****************************************************************************************************************************/
int8_t getY2KTime(SoftwareWire* _softTWI, time_t* _Y2KTimestamp) {
  int8_t rc = false;
  _softTWI->reconfigure(constSystemRtcSDAPin, constSystemRtcSCLPin);

#ifdef FEATURE_SYSTEM_RTC_DS3231_ENABLE
  rc = readDS3231Time(_softTWI, constSystemRtcI2CAddress, (time_t*) _Y2KTimestamp);
#elif defined (FEATURE_SYSTEM_RTC_PCF8563_ENABLE)
  rc = readPCF8563Time(_softTWI, constSystemRtcI2CAddress, (time_t*) _Y2KTimestamp);
#endif
  return rc;
}

/*****************************************************************************************************************************
*
*   Set UTC time taking Unix timestamp
*
*   Returns: 
*     - True on success
*     - False on error
*
*****************************************************************************************************************************/
int8_t setUnixTime(SoftwareWire* _softTWI, uint32_t _unixTimestamp) {
  int8_t rc = false;
  _softTWI->reconfigure(constSystemRtcSDAPin, constSystemRtcSCLPin);

#ifdef FEATURE_SYSTEM_RTC_DS3231_ENABLE
  rc = saveDS3231Time(_softTWI, constSystemRtcI2CAddress, (time_t) (_unixTimestamp - UNIX_OFFSET));
#elif defined (FEATURE_SYSTEM_RTC_PCF8563_ENABLE)
  rc = savePCF8563Time(_softTWI, constSystemRtcI2CAddress, (time_t) (_unixTimestamp - UNIX_OFFSET));
#endif
  return rc;
}

/*****************************************************************************************************************************
*
*   Get UTC time as Unix timestamp
*
*   Returns: 
*     - True on success
*     - False on error
*     - actual timestamp returns in _unixTimestamp
*
*****************************************************************************************************************************/
int8_t getUnixTime(SoftwareWire* _softTWI, uint32_t* _unixTimestamp) {
  int8_t rc;
  rc = getY2KTime(_softTWI, (time_t*) _unixTimestamp);
  *_unixTimestamp += UNIX_OFFSET;
  return rc;
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
