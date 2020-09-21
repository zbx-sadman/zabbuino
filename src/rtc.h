#pragma once

#include "i2c_bus.h"

/*****************************************************************************************************************************
*
*   Convert the number from uint8_t to BCD format
*
*   Returns: 
*     - BCD value
*
*****************************************************************************************************************************/
uint8_t Uint8ToBcd(const uint8_t);

/*****************************************************************************************************************************
*
*   Convert the number from BCD format to uint8_t
*
*   Returns: 
*     - unit8_t value
*
*****************************************************************************************************************************/
uint8_t BcdToUint8(const uint8_t);

/*****************************************************************************************************************************
*
*   Init RTC 
*
*   Returns: 
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on write error
*
*****************************************************************************************************************************/
int8_t initRTC(SoftwareTWI*);

/*****************************************************************************************************************************
*
*   Set UTC time using Y2K timestamp
*
*   Returns: 
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setY2KTime(SoftwareTWI*, const time_t);

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
int8_t getY2KTime(SoftwareTWI*, time_t*);

/*****************************************************************************************************************************
*
*   Set UTC time using Unix timestamp
*
*   Returns: 
*     - RESULT_IS_OK         on success
*     - RESULT_IS_FAIL       on write error
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t setUnixTime(SoftwareTWI*, const uint32_t);

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
int8_t getUnixTime(SoftwareTWI*, uint32_t*);

