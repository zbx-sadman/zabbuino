#include "i2c_bus.h"
#include "i2c_ds3231.h"


uint8_t IsDateTimeValid(const uint8_t _i2cAddress) {
  uint8_t status = readBytesFromi2C(_i2cAddress, DS3231_REG_STATUS, &status, 1);
  return !(status & _BV(DS3231_OSF));
}

uint8_t GetIsRunning(const uint8_t _i2cAddress) {
  uint8_t creg;
  readBytesFromi2C(_i2cAddress, DS3231_REG_CONTROL, &creg, 1);
  return !(creg & _BV(DS3231_EOSC));
}

void SetIsRunning(const uint8_t _i2cAddress, uint8_t _isRunning) {
  uint8_t creg;
  readBytesFromi2C(_i2cAddress, DS3231_REG_CONTROL, &creg, 1);
  if (_isRunning) {
      creg &= ~_BV(DS3231_EOSC);
  } else {
      creg |= _BV(DS3231_EOSC);
  }
  writeByteToI2C(_i2cAddress, DS3231_REG_CONTROL, creg);
}

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
*   Convert the number from BCD format to 24H format
*
*   Returns: 
*     - 24H format value
*
*****************************************************************************************************************************/
uint8_t BcdToBin24Hour(uint8_t bcdHour) {
  uint8_t hour;
  if (bcdHour & 0x40) {
     // 12 hour mode, convert to 24
     bool isPm = ((bcdHour & 0x20) != 0);
     hour = BcdToUint8(bcdHour & 0x1f);
     if (isPm) {
        hour += 12;
     }
  } else {
     hour = BcdToUint8(bcdHour);
  }
  return hour;
}

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
int8_t setDateTime(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, tm* _dateTime) {
  int8_t rc = RESULT_IS_FAIL;
  uint8_t status, value[DS3231_REG_TIMEDATE_SIZE];
  uint8_t year, centuryFlag;

  if (false == isI2CDeviceReady(_i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish;  }
  if (!GetIsRunning(_i2cAddress)) { SetIsRunning(_i2cAddress, true); }

  _dateTime->tm_mon++; // tm_mon - months since January [0 to 11], but DS3231 wants [1 to 12]
  _dateTime->tm_wday++; // tm_wday - days since Sunday [0 to 6], but DS3231 wants [1 to 7]
  _dateTime->tm_year += 1900; // tm_year - years since 1900

  /*
  Serial.println("Broken down for RTC: ");

  Serial.print("Sec: "); Serial.println(_dateTime->tm_sec);
  Serial.print("Min: "); Serial.println(_dateTime->tm_min);
  Serial.print("Hour: "); Serial.println(_dateTime->tm_hour);
  Serial.print("Wday: "); Serial.println(_dateTime->tm_wday);
  Serial.print("Mday: "); Serial.println(_dateTime->tm_mday);
  Serial.print("Month: "); Serial.println(_dateTime->tm_mon);
  Serial.print("Year: "); Serial.println(_dateTime->tm_year);
  Serial.println();
  */


  year = _dateTime->tm_year - 2000;
  centuryFlag = 0;
  if (year >= 100) {
     year -= 100;
     centuryFlag = _BV(7);
  }
  value[0]=Uint8ToBcd(_dateTime->tm_sec);
  value[1]=Uint8ToBcd(_dateTime->tm_min);
  value[2]=Uint8ToBcd(_dateTime->tm_hour);
  value[3]=Uint8ToBcd(_dateTime->tm_wday);
  value[4]=Uint8ToBcd(_dateTime->tm_mday);
  value[5]=Uint8ToBcd(_dateTime->tm_mon) | centuryFlag;
  value[6]=Uint8ToBcd(year);

  // clear the invalid flag
  readBytesFromi2C(_i2cAddress, DS3231_REG_STATUS, &status, 1);
  status &= ~_BV(DS3231_OSF); // clear the flag
  writeByteToI2C(_i2cAddress, DS3231_REG_STATUS, status);
  // set the date time and return PK if Wire.endTransmission called from writeBytesToI2C() returns 0
  if (0x00 == writeBytesToI2C(_i2cAddress, DS3231_REG_TIMEDATE, value, sizeof(value))) { rc = RESULT_IS_OK; }

  finish:
  return rc;
}

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
int8_t getDateTime(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, tm* _dateTime) {
  int8_t rc = RESULT_IS_FAIL;
  uint8_t value[DS3231_REG_TIMEDATE_SIZE];

  if (false == isI2CDeviceReady(_i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  // Common Causes:
  //    1) first time you ran and the device wasn't running yet
  //    2) the battery on the device is low or even missing
  // TODO: make new retcode (?)
  if (!IsDateTimeValid(_i2cAddress)) { goto finish; } // rc already init as RESULT_IS_FAIL

  if (!GetIsRunning(_i2cAddress)) { SetIsRunning(_i2cAddress, true); }

  if (0x00 == readBytesFromi2C(_i2cAddress, DS3231_REG_TIMEDATE, value, sizeof(value))) { rc = RESULT_IS_OK; };

  _dateTime->tm_sec   = BcdToUint8(value[0] & 0x7F);
  _dateTime->tm_min   = BcdToUint8(value[1]);
  _dateTime->tm_hour  = BcdToBin24Hour(value[2]);
  _dateTime->tm_wday  = BcdToUint8(value[3]);
  _dateTime->tm_mday  = BcdToUint8(value[4]);
//  monthRaw            = value[5];
  _dateTime->tm_mon   = value[5];
  _dateTime->tm_year  = 2000 + BcdToUint8(value[6]);
  // century wrap flag
  if (_dateTime->tm_mon & _BV(7)) {
     _dateTime->tm_year += 100;
  }
  _dateTime->tm_mon = BcdToUint8(_dateTime->tm_mon & 0x7F);
/*
  Serial.println("Readed from RTC: ");
  Serial.print("Sec: "); Serial.println(_dateTime->tm_sec);
  Serial.print("Min: "); Serial.println(_dateTime->tm_min);
  Serial.print("Hour: "); Serial.println(_dateTime->tm_hour);
  Serial.print("Wday: "); Serial.println(_dateTime->tm_wday);
  Serial.print("Mday: "); Serial.println(_dateTime->tm_mday);
  Serial.print("Month: "); Serial.println(_dateTime->tm_mon);
  Serial.print("Year: "); Serial.println(_dateTime->tm_year);
  Serial.println();
*/

  _dateTime->tm_mon--; // tm_mon - months since January [0 to 11], but DS3231 returns [1 to 12]
  _dateTime->tm_wday--; // tm_wday - days since Sunday [0 to 6], but DS3231 returns [1 to 7]
  _dateTime->tm_year -= 1900; // tm_year - years since 1900

  finish:
  return rc;
}
   
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
int8_t getLocalTime(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, int32_t* _unixTimestamp) {
  int8_t rc;
  tm dateTime;
  *_unixTimestamp = 0;
  rc = getDateTime(_sdaPin, _sclPin, _i2cAddress, &dateTime);

  if (RESULT_IS_OK == rc) { 
     // Make Y2K timestamp from _tm_ structure and convert it to UNIX timestamp
     // mk_gmtime() - This function 'compiles' the elements of a broken-down time structure, returning a binary time stamp. 
     // The elements of timeptr are interpreted as representing UTC.
     *_unixTimestamp = mk_gmtime(&dateTime);
     *_unixTimestamp += UNIX_OFFSET; 
     rc = RESULT_IN_LONGVAR;
  }
  return rc;
}

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
int8_t setLocalTime(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, int32_t _unixTimestamp) {
  // Convert Y2K timestamp to _tm_ structure (refer to <time.h>)
  tm dateTime;
  time_t y2kts = _unixTimestamp - UNIX_OFFSET;
  gmtime_r(&y2kts, &dateTime);
  return setDateTime(_sdaPin, _sclPin, _i2cAddress, &dateTime);
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
int8_t setTZ(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, int16_t _tzOffsetSec) {
  uint8_t rc = RESULT_IS_FAIL, 
          value[3] = {0, 0, 0};  // first two byte - EEPROM address 

  // write first byte to addr: 0000
  value[2] = _tzOffsetSec >> 8;
  if (0x00 != writeBytesToI2C(_i2cAddress, I2C_NO_REG_SPECIFIED, value, sizeof(value))) { goto finish; } 

  // write second byte to addr: 0001
  value[1]++;
  value[2] = _tzOffsetSec & 0x00FF;
  if (0x00 != writeBytesToI2C(_i2cAddress, I2C_NO_REG_SPECIFIED, value, sizeof(value))) { goto finish; } 

  rc = RESULT_IS_OK; 

  finish:
  return rc;
}

/*****************************************************************************************************************************
*
*   Get TimeZone offset (in seconds). Actually - just read it from DS3231 module's onboard EEPROM (AT24C32).
*
*   Returns: 
*     - RESULT_IN_OK on success
*     - RESULT_IS_FAIL on read error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t getTZ(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, int16_t* _tzOffsetSec) {
  uint8_t rc = RESULT_IS_FAIL, 
          value[2] = {0, 0}; // first two byte - EEPROM address 

  // write to controller address from which will be readed data
  if (0x00 != writeBytesToI2C(_i2cAddress, I2C_NO_REG_SPECIFIED, value, sizeof(value))) { goto finish; } 

  // read two byte (uint16_t)
  if (0x00 != readBytesFromi2C(_i2cAddress, I2C_NO_REG_SPECIFIED, value, 2) ) { goto finish; } 

  *_tzOffsetSec = (value[0] << 8) | value[1];

  rc = RESULT_IN_LONGVAR;

  finish:
  return rc;
}
