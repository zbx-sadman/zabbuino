#pragma once

/*

Based on 

*/


//PCF8563 Register Addresses
#define PCF8563_REG_CONTROL_STATUS_1  (0x00)
#define PCF8563_REG_CONTROL_STATUS_2  (0x01)
                                      
#define PCF8563_REG_VL_SECONDS        (0x02)
#define PCF8563_REG_MINUTES           (0x03)
#define PCF8563_REG_HOURS             (0x04)
#define PCF8563_REG_DAYS              (0x05)
#define PCF8563_REG_WEEKDAYS          (0x06)
#define PCF8563_REG_CENTURY_MONTHS    (0x07)
#define PCF8563_REG_YEARS             (0x08)
                                           
#define PCF8563_REG_MINUTE_ALARM      (0x09)
#define PCF8563_REG_HOUR_ALARM        (0x0A)
#define PCF8563_REG_DAY_ALARM         (0x0B)
#define PCF8563_REG_WEEKDAY_ALARM     (0x0C)

#define PCF8563_REG_CLOUT_CONTROL     (0x0D)
#define PCF8563_REG_TIMER_CONTROL     (0x0E)
#define PCF8563_REG_TIMER             (0x0F)

//PCF8563 Register Data Size if not just 1
#define PCF8563_REG_TIMEDATE_SIZE     (7)

// PCF8563 Control Status 1 Register Bits
#define PCF8563_STOP                  (5)

// PCF8563 VL_SECONDS register masks
#define PCF8563_VL                    (0x80)

// PCF8563 VL_SECONDS register masks
#define PCF8563_CENTURY_FLAG          (0x80)

/*****************************************************************************************************************************
*
*   Init PCF8563 RTC 
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t initPCF8563(SoftwareTWI*, const uint8_t);

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
int8_t savePCF8563Time(SoftwareTWI*, uint8_t, time_t);

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
int8_t readPCF8563Time(SoftwareTWI*, uint8_t, time_t*);


