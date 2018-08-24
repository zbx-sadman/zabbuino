// Config & common included files
#include "sys_includes.h"

#include <time.h>

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_pcf8563.h"

#include "rtc.h"


/*****************************************************************************************************************************
*
*   Test source clock state (STOP bit)
*
*   Returns: 
*     - True if STOP bit is cleared
*     - False otherwise
*
*****************************************************************************************************************************/
/*
uint8_t isPCF8563Running(SoftwareWire* _softTWI, const uint8_t _i2cAddress) {
  uint8_t creg;
  // PCF8563_STOP = 0 - RTC source clock runs 
  readBytesFromI2C(_softTWI, _i2cAddress, PCF8563_REG_CONTROL_STATUS_1, &creg, 1);
  return !(creg & _BV(PCF8563_STOP));
}
*/
/*****************************************************************************************************************************
*
*   Start or stop oscillator
*
*   Returns: 
*     - 
*
*****************************************************************************************************************************/
/*
void setPCF8563RunningState(SoftwareWire* _softTWI, const uint8_t _i2cAddress, uint8_t _isRunning) {
  uint8_t creg;
  readBytesFromI2C(_softTWI, _i2cAddress, PCF8563_REG_CONTROL, &creg, 1);
  if (_isRunning) {
      creg &= ~_BV(PCF8563_EOSC);
  } else {
      creg |= _BV(PCF8563_EOSC);
  }
  writeByteToI2C(_softTWI, _i2cAddress, PCF8563_REG_CONTROL, creg);
}
*/
/*****************************************************************************************************************************
*
*   "Validate" stored date&time by detect "Low Voltage" register status
*
*   Returns: 
*     - True if Voltage is good is running
*     - False otherwise
*
*****************************************************************************************************************************/
uint8_t isPCF8563DateTimeValid(SoftwareWire* _softTWI, const uint8_t _i2cAddress) {
  uint8_t creg;

  // The VL flag is intended to detect the situation when VDD is decreasing slowly, for example
  // under battery operation. Should the oscillator stop or VDD reach Vlow before power is
  // re-asserted, then the VL flag is set. This will indicate that the time may be corrupted.
  readBytesFromI2C(_softTWI, _i2cAddress, PCF8563_REG_VL_SECONDS, &creg, 1);
  return !(creg & PCF8563_VL);
}

/*****************************************************************************************************************************
*
*   Init PCF8563 RTC 
*
*   Returns: 
*     - 
*
*****************************************************************************************************************************/
int8_t initPCF8563(SoftwareWire* _softTWI, const uint8_t _i2cAddress) {
  int8_t rc = false;

  uint8_t creg[2] = {0x00, 0x00};
  // Register Control_status_1 set to 0x00: 
  //    Bit 7, TEST1 = 0 - normal mode; must be set to logic 0 during normal operations 
  //    Bit 5, STOP = 0 - RTC source clock runs
  //    Bit 3, TESTC = 0 - Power-On Reset (POR) override facility is disabled; set to logic 0 for normal operation
  //    Other bits unused
  
  // Register Control_status_2 set to 0x00: 
  //    Bit 4, TI_TP = 0 - INT is active when TF is active (subject to the status of TIE) 
  //    Bit 3, AF = 0 - alarm flag is cleared
  //    Bit 2, TF = 0 - timer flag is cleared
  //    Bit 1, AIE = 0 - alarm interrupt disabled
  //    Bit 0, TIE = 0 - timer interrupt disabled

  if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, PCF8563_REG_CONTROL_STATUS_1, creg, 0x02)) { goto finish; }

  rc = true;

  finish:
  return rc;

}

/*****************************************************************************************************************************
*
*   Save date & time to PCF8563
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t savePCF8563Time(SoftwareWire* _softTWI, uint8_t _i2cAddress, time_t _y2Ktimestamp) {
  int8_t rc = false;
  uint8_t value[PCF8563_REG_TIMEDATE_SIZE];
  uint8_t centuryFlag;
  tm dateTime;

  if (false == isI2CDeviceReady(_softTWI, _i2cAddress)) { goto finish;  }

  // Convert Y2K timestamp to _tm_ structure (refer to <time.h>)
  gmtime_r(&_y2Ktimestamp, &dateTime);

  dateTime.tm_mon++; // tm_mon - months since January [0 to 11], but PCF8563 wants [1 to 12]

  centuryFlag = 0;
  if (dateTime.tm_year >= 100) {
     dateTime.tm_year -= 100;
     centuryFlag = PCF8563_CENTURY_FLAG;
  }

  value[0] = Uint8ToBcd(dateTime.tm_sec);
  value[1] = Uint8ToBcd(dateTime.tm_min);
  value[2] = Uint8ToBcd(dateTime.tm_hour);
  value[3] = Uint8ToBcd(dateTime.tm_mday);
  value[4] = Uint8ToBcd(dateTime.tm_wday);
  value[5] = Uint8ToBcd(dateTime.tm_mon) | centuryFlag;
  value[6] = Uint8ToBcd(dateTime.tm_year);

  // set the date time and return OK if Wire.endTransmission called from writeBytesToI2C() returns 0
  rc = (sizeof(value) == writeBytesToI2C(_softTWI, _i2cAddress, PCF8563_REG_VL_SECONDS, value, sizeof(value)));

  finish:
  return rc;
}

/*****************************************************************************************************************************
*
*   Read date & time from PCF8563
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - RESULT_IS_FAIL on read error
*     - DEVICE_ERROR_CONNECT on connection error
*     - actual timestamp returns in _timestamp 
*
*****************************************************************************************************************************/
int8_t readPCF8563Time(SoftwareWire* _softTWI, uint8_t _i2cAddress, time_t* _timestamp) {
  int8_t rc = false;
  uint8_t centuryFlag;
  uint8_t value[PCF8563_REG_TIMEDATE_SIZE];
  tm dateTime;

  if (false == isI2CDeviceReady(_softTWI, _i2cAddress)) { goto finish; }

  if (!isPCF8563DateTimeValid(_softTWI, _i2cAddress)) { goto finish; } // rc already init as RESULT_IS_FAIL

  rc = (sizeof(value) == readBytesFromI2C(_softTWI, _i2cAddress, PCF8563_REG_VL_SECONDS, value, sizeof(value)));

  dateTime.tm_sec   = BcdToUint8(value[0] & 0x7F);
  dateTime.tm_min   = BcdToUint8(value[1] & 0x7F);
  dateTime.tm_hour  = BcdToUint8(value[2] & 0x3F);
  dateTime.tm_mday  = BcdToUint8(value[3] & 0x3F);
  dateTime.tm_wday  = BcdToUint8(value[4] & 0x7F);
  dateTime.tm_mon   = value[5];
  centuryFlag       = dateTime.tm_mon & PCF8563_CENTURY_FLAG;
  dateTime.tm_mon   = BcdToUint8(dateTime.tm_mon & 0x1F);
  dateTime.tm_year  = BcdToUint8(value[6]);

  // century wrap flag
  if (centuryFlag) {
     dateTime.tm_year += 100;
  }

  dateTime.tm_mon--; // tm_mon - months since January [0 to 11], but PCF8563 returns [1 to 12]

  // Make timestamp
  *_timestamp = mk_gmtime(&dateTime);

  finish:
  return rc;
}
  
