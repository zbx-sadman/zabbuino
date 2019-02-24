#pragma once

// Conversion Time (Note 2) 0.17 0.22 s
// Note 2: Guaranteed by design. Not production tested. 
// (C) MAX6675 datasheet
#define MAX6675_CONVERSION_TIME                               (220UL) //ms

#define MAX6675_BITMASK_ID                                    (0x02)
#define MAX6675_BITMASK_TERMOCOUPLE_INPUT                     (0x04)

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getMAX6675Metric(const uint8_t, const uint8_t, const uint8_t, const uint8_t, uint32_t*);

int8_t getMAX6675Metric(const uint8_t, const uint8_t, const uint8_t, const uint8_t, char*);

/*****************************************************************************************************************************
*
*   Read specified metric's value of the MAX6675 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on termocouple connection error
*     - DEVICE_ERROR_WRONG_ID on wrong value of MAX6675's ID bit detection
*     - RESULT_IS_FAIL on other fails
*
*****************************************************************************************************************************/
int8_t getMAX6675Metric(const uint8_t, const uint8_t, const uint8_t, const uint8_t, char*, uint32_t*, const uint8_t);

