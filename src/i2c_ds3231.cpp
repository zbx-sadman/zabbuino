#include "i2c_bus.h"
#include "i2c_ds3231.h"


/*****************************************************************************************************************************
*
*   Init DS3231 RTC 
*
*   Returns: 
*     - 
*
*****************************************************************************************************************************/
int8_t initDS3231(SoftwareWire* _softTWI, const uint8_t _i2cAddress) {
  int8_t rc = false;

  // Kickstart oscillator of it stopped early
  if (!isDS3231Running(_softTWI, _i2cAddress)) { setDS3231RunningState(_softTWI, _i2cAddress, true); }

  uint8_t creg;

  if (0x00 != readBytesFromi2C(_softTWI, _i2cAddress, DS3231_REG_CONTROL, &creg, 1)) { goto finish; }

  // Set SquareWavePin mode 'None'
  // clear all relevant bits to a known "off" state
  creg &= ~(DS3231_AIEMASK | _BV(DS3231_BBSQW));
  creg |= _BV(DS3231_INTCN);  // set INTCN to disables SQW
  if (0x00 != writeByteToI2C(_softTWI, _i2cAddress, DS3231_REG_CONTROL, creg)) { goto finish; } 

  rc = true;

  finish:
  return rc;

}


/*****************************************************************************************************************************
*
*   Test oscillator's enable flag
*
*   Returns: 
*     - True if oscillator is enabled
*     - False otherwise
*
*****************************************************************************************************************************/
static uint8_t isDS3231Running(SoftwareWire* _softTWI, const uint8_t _i2cAddress) {
  uint8_t creg;
  // EOSC - enable Oscillator. When set to logic 0, the oscillator is started. When set to logic 1, the oscillator is stopped 
  // when the DS3231 switches to VBAT. This bit is clear (logic 0) when power is first applied. When the DS3231 is powered by 
  // VCC, the oscillator is always on regardless of the status of the EOSC bit. When EOSC is disabled, all register data is ***static***
  readBytesFromi2C(_softTWI, _i2cAddress, DS3231_REG_CONTROL, &creg, 1);
  return !(creg & _BV(DS3231_EOSC));
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
  uint8_t creg;
  readBytesFromi2C(_softTWI, _i2cAddress, DS3231_REG_CONTROL, &creg, 1);
  if (_isRunning) {
      creg &= ~_BV(DS3231_EOSC);
  } else {
      creg |= _BV(DS3231_EOSC);
  }
  writeByteToI2C(_softTWI, _i2cAddress, DS3231_REG_CONTROL, creg);
}

/*****************************************************************************************************************************
*
*   "Validate" stored date&time by detect oscillator status
*
*   Returns: 
*     - True if oscillator is running
*     - False otherwise
*
*****************************************************************************************************************************/
static uint8_t isDS3231DateTimeValid(SoftwareWire* _softTWI, const uint8_t _i2cAddress) {
  // OSF - Oscillator Stop Flag ). A logic 1 in this bit indicates that the oscillator either is stopped or was stopped for 
  // some period and may be used to judge the validity of the timekeeping data. 
  // OSF bit is be set on:
  //    1) The first time power is applied.
  //    2) The voltages present on both VCC and VBAT are insufficient to support oscillation.
  //    3) The EOSC bit is turned off in battery-backed mode.
  //    4) External influences on the crystal (i.e., noise, leakage,etc.).
  uint8_t status = readBytesFromi2C(_softTWI, _i2cAddress, DS3231_REG_STATUS, &status, 1);
  return !(status & _BV(DS3231_OSF));
}

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
int8_t saveDS3231Time(SoftwareWire* _softTWI, uint8_t _i2cAddress, time_t _y2Ktimestamp) {
  int8_t rc = false;
  uint8_t status, value[DS3231_REG_TIMEDATE_SIZE];
  uint8_t year, centuryFlag;
  tm dateTime;

  if (false == isI2CDeviceReady(_softTWI, _i2cAddress)) { goto finish;  }

  // Convert Y2K timestamp to _tm_ structure (refer to <time.h>)
  gmtime_r(&_y2Ktimestamp, &dateTime);


  dateTime.tm_mon++; // tm_mon - months since January [0 to 11], but DS3231 wants [1 to 12]
  dateTime.tm_wday++; // tm_wday - days since Sunday [0 to 6], but DS3231 wants [1 to 7]
  dateTime.tm_year += 1900; // tm_year - years since 1900

  year = dateTime.tm_year - 2000;
  centuryFlag = 0;
  if (year >= 100) {
     year -= 100;
     centuryFlag = _BV(7);
  }
  value[0]=Uint8ToBcd(dateTime.tm_sec);
  value[1]=Uint8ToBcd(dateTime.tm_min);
  value[2]=Uint8ToBcd(dateTime.tm_hour);
  value[3]=Uint8ToBcd(dateTime.tm_wday);
  value[4]=Uint8ToBcd(dateTime.tm_mday);
  value[5]=Uint8ToBcd(dateTime.tm_mon) | centuryFlag;
  value[6]=Uint8ToBcd(year);

  // clear the invalid flag
  readBytesFromi2C(_softTWI, _i2cAddress, DS3231_REG_STATUS, &status, 1);
  status &= ~_BV(DS3231_OSF); // clear the flag
  writeByteToI2C(_softTWI, _i2cAddress, DS3231_REG_STATUS, status);
  // set the date time and return OK if Wire.endTransmission called from writeBytesToI2C() returns 0
  rc = (0x00 == writeBytesToI2C(_softTWI, _i2cAddress, DS3231_REG_TIMEDATE, value, sizeof(value)));

  finish:
  return rc;
}

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
int8_t readDS3231Time(SoftwareWire* _softTWI, uint8_t _i2cAddress, time_t* _timestamp) {
  int8_t rc = false;
  uint8_t value[DS3231_REG_TIMEDATE_SIZE];
  tm dateTime;

  if (false == isI2CDeviceReady(_softTWI, _i2cAddress)) { goto finish; }

  // Common Causes:
  //    1) first time you ran and the device wasn't running yet
  //    2) the battery on the device is low or even missing
  // TODO: make new retcode (?)
  if (!isDS3231DateTimeValid(_softTWI, _i2cAddress)) { goto finish; } // rc already init as RESULT_IS_FAIL

  rc = (0x00 == readBytesFromi2C(_softTWI, _i2cAddress, DS3231_REG_TIMEDATE, value, sizeof(value)));

  dateTime.tm_sec   = BcdToUint8(value[0] & 0x7F);
  dateTime.tm_min   = BcdToUint8(value[1]);
  dateTime.tm_hour  = BcdToBin24Hour(value[2]);
  dateTime.tm_wday  = BcdToUint8(value[3]);
  dateTime.tm_mday  = BcdToUint8(value[4]);
  dateTime.tm_mon   = value[5];
  dateTime.tm_year  = 2000 + BcdToUint8(value[6]);
  // century wrap flag
  if (dateTime.tm_mon & _BV(7)) {
     dateTime.tm_year += 100;
  }
  dateTime.tm_mon = BcdToUint8(dateTime.tm_mon & 0x7F);

  dateTime.tm_mon--; // tm_mon - months since January [0 to 11], but DS3231 returns [1 to 12]
  dateTime.tm_wday--; // tm_wday - days since Sunday [0 to 6], but DS3231 returns [1 to 7]
  dateTime.tm_year -= 1900; // tm_year - years since 1900
  // Make timestamp
  *_timestamp = mk_gmtime(&dateTime);

  finish:
  return rc;
}
  

/*****************************************************************************************************************************
*
*   Convert the number from uint8_t to BCD format
*
*   Returns: 
*     - BCD value
*
*****************************************************************************************************************************/
static uint8_t Uint8ToBcd(uint8_t val) {
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
static uint8_t BcdToUint8(uint8_t val) {
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
static uint8_t BcdToBin24Hour(uint8_t bcdHour) {
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

