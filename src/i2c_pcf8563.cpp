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
static uint8_t isPCF8563DateTimeValid(SoftwareWire* _softTWI, const uint8_t _i2cAddress) {
  uint8_t vlSeconds;

  // The VL flag is intended to detect the situation when VDD is decreasing slowly, for example
  // under battery operation. Should the oscillator stop or VDD reach Vlow before power is
  // re-asserted, then the VL flag is set. This will indicate that the time may be corrupted.
  readBytesFromI2C(_softTWI, _i2cAddress, PCF8563_REG_VL_SECONDS, &vlSeconds, 0x01);
  //Serial.print("vlSeconds: "); Serial.println(vlSeconds, BIN);
  return !(vlSeconds & PCF8563_VL);
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
  __SUPPRESS_WARNING_UNUSED(_softTWI);
  __SUPPRESS_WARNING_UNUSED(_i2cAddress);

  //int8_t rc = RESULT_IS_FAIL;
  int8_t rc = RESULT_IS_OK;

  //uint8_t control[0x02] = {0x00, 0x00};
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

  //if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, PCF8563_REG_CONTROL_STATUS_1, control, 0x02)) { goto finish; }
  //Serial.print("control1: "); Serial.println(control[0], BIN);
  //Serial.print("control2: "); Serial.println(control[1], BIN);

  // !!! All settings is OK on start by default
  //rc = RESULT_IS_OK;

  //finish:
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
  int8_t rc = RESULT_IS_FAIL;
  uint8_t rawData[PCF8563_REG_TIMEDATE_SIZE];
  uint8_t centuryFlag;
  tm dateTime;

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish;  }

  // Convert Y2K timestamp to _tm_ structure (refer to <time.h>)
  gmtime_r(&_y2Ktimestamp, &dateTime);

  dateTime.tm_mon++; // tm_mon - months since January [0 to 11], but PCF8563 wants [1 to 12]

  centuryFlag = 0x00;
  if (dateTime.tm_year >= 100) {
     dateTime.tm_year -= 100;
     centuryFlag = PCF8563_CENTURY_FLAG;
  }

  rawData[0x00] = Uint8ToBcd(dateTime.tm_sec);
  rawData[0x01] = Uint8ToBcd(dateTime.tm_min);
  rawData[0x02] = Uint8ToBcd(dateTime.tm_hour);
  rawData[0x03] = Uint8ToBcd(dateTime.tm_mday);
  rawData[0x04] = Uint8ToBcd(dateTime.tm_wday);
  rawData[0x05] = Uint8ToBcd(dateTime.tm_mon) | centuryFlag;
  rawData[0x06] = Uint8ToBcd(dateTime.tm_year);

  // set the date time and return OK if Wire.endTransmission called from writeBytesToI2C() returns 0
  if (sizeof(rawData) == writeBytesToI2C(_softTWI, _i2cAddress, PCF8563_REG_VL_SECONDS, rawData, sizeof(rawData))) { rc = RESULT_IS_OK; }

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
  int8_t rc = RESULT_IS_FAIL;
  uint8_t centuryFlag;
  uint8_t rawData[PCF8563_REG_TIMEDATE_SIZE];
  tm dateTime;
                          
  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

  if (!isPCF8563DateTimeValid(_softTWI, _i2cAddress)) { goto finish; } // rc already init as RESULT_IS_FAIL

  if (sizeof(rawData) == readBytesFromI2C(_softTWI, _i2cAddress, PCF8563_REG_VL_SECONDS, rawData, sizeof(rawData))) {

     dateTime.tm_sec   = BcdToUint8(rawData[0x00] & 0x7F);
     dateTime.tm_min   = BcdToUint8(rawData[0x01] & 0x7F);
     dateTime.tm_hour  = BcdToUint8(rawData[0x02] & 0x3F);
     dateTime.tm_mday  = BcdToUint8(rawData[0x03] & 0x3F);
     dateTime.tm_wday  = BcdToUint8(rawData[0x04] & 0x7F);
     dateTime.tm_mon   = rawData[0x05];
     centuryFlag       = dateTime.tm_mon & PCF8563_CENTURY_FLAG;
     dateTime.tm_mon   = BcdToUint8(dateTime.tm_mon & 0x1F);
     dateTime.tm_year  = BcdToUint8(rawData[0x06]);
   
     // century wrap flag
     if (centuryFlag) { dateTime.tm_year += 100; }
   
     dateTime.tm_mon--; // tm_mon - months since January [0 to 11], but PCF8563 returns [1 to 12]
   
     // Make timestamp
     *_timestamp = mk_gmtime(&dateTime);
     rc = RESULT_IS_OK; 
  }

  finish:
  return rc;
}
  
