// Config & common included files
#if defined(ARDUINO_ARCH_AVR)

#include "sys_includes.h"

#include <time.h>

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
uint8_t Uint8ToBcd(const uint8_t val) {
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
uint8_t BcdToUint8(const uint8_t val) {
  return val - 6 * (val >> 4);
}

/*****************************************************************************************************************************
*
*   Init RTC 
*
*   Returns: 
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on write error
*
*****************************************************************************************************************************/
int8_t initRTC(SoftwareTWI* _softTWI) {
  int8_t rc = RESULT_IS_FAIL;
  // Init RTC chip
  _softTWI->begin(constSystemRtcSDAPin, constSystemRtcSCLPin);
#if defined(FEATURE_SYSTEM_RTC_DS3231_ENABLE)
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
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setY2KTime(SoftwareTWI* _softTWI, const time_t _Y2KTimestamp) {
  __SUPPRESS_WARNING_UNUSED(_Y2KTimestamp);

  int8_t rc = RESULT_IS_FAIL;
  _softTWI->begin(constSystemRtcSDAPin, constSystemRtcSCLPin);
#if defined(FEATURE_SYSTEM_RTC_DS3231_ENABLE)
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
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on read error
*     - DEVICE_ERROR_CONNECT on connection error
*     - actual timestamp returns in _Y2KTimestamp
*
*****************************************************************************************************************************/
int8_t getY2KTime(SoftwareTWI* _softTWI, time_t* _Y2KTimestamp) {
  __SUPPRESS_WARNING_UNUSED(_Y2KTimestamp);

  int8_t rc = RESULT_IS_FAIL;
  _softTWI->begin(constSystemRtcSDAPin, constSystemRtcSCLPin);

#if defined(FEATURE_SYSTEM_RTC_DS3231_ENABLE)
  rc = readDS3231Time(_softTWI, constSystemRtcI2CAddress,  _Y2KTimestamp);
#elif defined (FEATURE_SYSTEM_RTC_PCF8563_ENABLE)
  rc = readPCF8563Time(_softTWI, constSystemRtcI2CAddress, _Y2KTimestamp);
#endif
  return rc;
}

/*****************************************************************************************************************************
*
*   Set UTC time taking Unix timestamp
*
*   Returns: 
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setUnixTime(SoftwareTWI* _softTWI, const uint32_t _unixTimestamp) {
  __SUPPRESS_WARNING_UNUSED(_unixTimestamp);

  int8_t rc = RESULT_IS_FAIL;
  _softTWI->begin(constSystemRtcSDAPin, constSystemRtcSCLPin);

#if defined(FEATURE_SYSTEM_RTC_DS3231_ENABLE)
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
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on read error
*     - DEVICE_ERROR_CONNECT on connection error
*     - actual timestamp returns in _unixTimestamp 
*
*****************************************************************************************************************************/
int8_t getUnixTime(SoftwareTWI* _softTWI, uint32_t* _unixTimestamp) {
  int8_t rc;
  rc = getY2KTime(_softTWI, (time_t*) _unixTimestamp);
  *_unixTimestamp += UNIX_OFFSET;
  return rc;
}
#endif