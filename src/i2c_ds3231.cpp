// Config & common included files
#include "sys_includes.h"

#include <time.h>

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_ds3231.h"

#include "rtc.h"

/*****************************************************************************************************************************
*
*   Test oscillator running state
*
*   Returns: 
*     - True if oscillator is ok
*     - False otherwise
*
*****************************************************************************************************************************/
static uint8_t isDS3231Running(SoftwareWire* _softTWI, const uint8_t _i2cAddress) {
  uint8_t control, status, rc;
  // EOSC - enable Oscillator. When set to logic 0, the oscillator is started. When set to logic 1, the oscillator is stopped 
  // when the DS3231 switches to VBAT. This bit is clear (logic 0) when power is first applied. When the DS3231 is powered by 
  // VCC, the oscillator is always on regardless of the status of the EOSC bit. When EOSC is disabled, all register data is ***static***

  // OSF - Oscillator Stop Flag ). A logic 1 in this bit indicates that the oscillator either is stopped or was stopped for 
  // some period and may be used to judge the validity of the timekeeping data. 
  // OSF bit is be set on:
  //    1) The first time power is applied.
  //    2) The voltages present on both VCC and VBAT are insufficient to support oscillation.
  //    3) The EOSC bit is turned off in battery-backed mode.
  //    4) External influences on the crystal (i.e., noise, leakage,etc.).

  readBytesFromI2C(_softTWI, _i2cAddress, DS3231_REG_CONTROL, &control, 0x01);
  readBytesFromI2C(_softTWI, _i2cAddress, DS3231_REG_STATUS,  &status,  0x01);
  rc = !((control & _BV(DS3231_EOSC)) || (status & _BV(DS3231_OSF)));
  return rc;
}

/*****************************************************************************************************************************
*
*   Start or stop oscillator
*
*   Returns: 
*     - 
*
*****************************************************************************************************************************/
static void setDS3231RunningState(SoftwareWire* _softTWI, const uint8_t _i2cAddress, uint8_t _isRunning) {
  uint8_t control, status;
  readBytesFromI2C(_softTWI, _i2cAddress, DS3231_REG_CONTROL, &control, 0x01);
  readBytesFromI2C(_softTWI, _i2cAddress, DS3231_REG_STATUS,  &status,  0x01);
  if (_isRunning) {
      // clear flags
      control &= ~_BV(DS3231_EOSC);
      status  &= ~_BV(DS3231_OSF); 
  } else {
      control |= _BV(DS3231_EOSC);
      status  |= _BV(DS3231_OSF); 
  }
  writeByteToI2C(_softTWI, _i2cAddress, DS3231_REG_CONTROL, control);
  writeByteToI2C(_softTWI, _i2cAddress, DS3231_REG_STATUS,  status);
}

/*****************************************************************************************************************************
*
*   Convert the number from BCD format to 24H format
*
*   Returns: 
*     - 24H format value
*
*****************************************************************************************************************************/
static uint8_t BcdToBin24Hour(uint8_t bcdHour) {
  uint8_t hour;
  if (bcdHour & 0x40) {
     // 12 hour mode, convert to 24
     int8_t isPm = ((bcdHour & 0x20) != 0x00);
     hour = BcdToUint8(bcdHour & 0x1F);
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
*   Init DS3231 RTC 
*
*   Returns: 
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on write error
*
*****************************************************************************************************************************/
int8_t initDS3231(SoftwareWire* _softTWI, const uint8_t _i2cAddress) {
  int8_t rc = RESULT_IS_FAIL;

  // Unknown stop reason is unknow return time, and unknown result.
  // Do not force start clocks - wait to set proper time in the saveDS3231Time()

  // Kickstart oscillator of it stopped early
  // if (!isDS3231Running(_softTWI, _i2cAddress)) { setDS3231RunningState(_softTWI, _i2cAddress, true); }

  uint8_t control;

  if (0x01 != readBytesFromI2C(_softTWI, _i2cAddress, DS3231_REG_CONTROL, &control, 0x01)) { goto finish; }

  // Set SquareWavePin mode 'None'
  // clear all relevant bits to a known "off" state
  control &= ~(DS3231_AIEMASK | _BV(DS3231_BBSQW));
  control |= _BV(DS3231_INTCN);  // set INTCN to disables SQW

  if (0x00 == writeByteToI2C(_softTWI, _i2cAddress, DS3231_REG_CONTROL, control)) { goto finish; } 

  rc = RESULT_IS_OK;

  finish:
  return rc;

}

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
int8_t saveDS3231Time(SoftwareWire* _softTWI, uint8_t _i2cAddress, time_t _y2Ktimestamp) {
  int8_t rc = RESULT_IS_FAIL;
  uint8_t rawData[DS3231_REG_TIMEDATE_SIZE];
  uint8_t centuryFlag;
  tm dateTime;

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  if (!isDS3231Running(_softTWI, _i2cAddress)) { setDS3231RunningState(_softTWI, _i2cAddress, true); }

  // Convert Y2K timestamp to _tm_ structure (refer to <time.h>)
  gmtime_r(&_y2Ktimestamp, &dateTime);


  dateTime.tm_mon++; // tm_mon - months since January [0 to 11], but DS3231 wants [1 to 12]
  dateTime.tm_wday++; // tm_wday - days since Sunday [0 to 6], but DS3231 wants [1 to 7]
  //dateTime.tm_year += 1900; // tm_year - years since 1900

  //year = dateTime.tm_year - 2000;
  centuryFlag = 0x00;
  if (dateTime.tm_year >= 100) {
     dateTime.tm_year -= 100;
     centuryFlag = DS3231_CENTURY_FLAG;
  }

  rawData[0x00]=Uint8ToBcd(dateTime.tm_sec);
  rawData[0x01]=Uint8ToBcd(dateTime.tm_min);
  rawData[0x02]=Uint8ToBcd(dateTime.tm_hour);
  rawData[0x03]=Uint8ToBcd(dateTime.tm_wday);
  rawData[0x04]=Uint8ToBcd(dateTime.tm_mday);
  rawData[0x05]=Uint8ToBcd(dateTime.tm_mon) | centuryFlag;
  rawData[0x06]=Uint8ToBcd(dateTime.tm_year);

  if (sizeof(rawData) == writeBytesToI2C(_softTWI, _i2cAddress, DS3231_REG_TIMEDATE, rawData, sizeof(rawData))) {rc = RESULT_IS_OK; }
 
finish:
  return rc;
}

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
int8_t readDS3231Time(SoftwareWire* _softTWI, uint8_t _i2cAddress, time_t* _timestamp) {
  int8_t rc = RESULT_IS_FAIL;
  uint8_t rawData[DS3231_REG_TIMEDATE_SIZE];
  uint8_t centuryFlag;
  tm dateTime;

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  // Common Causes:
  //    1) first time you ran and the device wasn't running yet
  //    2) the battery on the device is low or even missing
  // TODO: make new retcode (?)
  if (!isDS3231Running(_softTWI, _i2cAddress)) { goto finish; } // rc already init as RESULT_IS_FAIL

  if (sizeof(rawData) == readBytesFromI2C(_softTWI, _i2cAddress, DS3231_REG_TIMEDATE, rawData, sizeof(rawData))) {
     dateTime.tm_sec   = BcdToUint8(rawData[0x00] & 0x7F);
     dateTime.tm_min   = BcdToUint8(rawData[0x01]);
     dateTime.tm_hour  = BcdToBin24Hour(rawData[0x02]);
     dateTime.tm_wday  = BcdToUint8(rawData[0x03]);
     dateTime.tm_mday  = BcdToUint8(rawData[0x04]);
     dateTime.tm_mon   = rawData[0x05];
     centuryFlag       = dateTime.tm_mon & DS3231_CENTURY_FLAG;
     dateTime.tm_mon   = BcdToUint8(dateTime.tm_mon & 0x7F);
     //  dateTime.tm_year  = 2000 + BcdToUint8(value[6]);
     dateTime.tm_year  = BcdToUint8(rawData[0x06]);
     
     // century wrap flag
     if (centuryFlag) {
        dateTime.tm_year += 100;
     }
     
     dateTime.tm_mon--; // tm_mon - months since January [0 to 11], but DS3231 returns [1 to 12]
     dateTime.tm_wday--; // tm_wday - days since Sunday [0 to 6], but DS3231 returns [1 to 7]
     // dateTime.tm_year -= 1900; // tm_year - years since 1900
     // Make timestamp
     *_timestamp = mk_gmtime(&dateTime);
     rc = RESULT_IS_OK; 
  };
/*
  DEBUG_PORT.println("datetime: ");
  DEBUG_PORT.println(dateTime.tm_sec);
  DEBUG_PORT.println(dateTime.tm_min);
  DEBUG_PORT.println(dateTime.tm_hour);
  DEBUG_PORT.println(dateTime.tm_wday);
  DEBUG_PORT.println(dateTime.tm_mday);
  DEBUG_PORT.println(dateTime.tm_mon);
  DEBUG_PORT.println(dateTime.tm_year);


  DEBUG_PORT.print("*_timestamp: ");
  DEBUG_PORT.println(*_timestamp);
*/
finish:
  return rc;
}
  
