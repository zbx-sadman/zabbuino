#pragma once

/*

Based on: https://github.com/RobTillaart/Arduino/tree/master/libraries/DHTstable
version 0.1.13 is used

*/

#define DHT11_ID                                                (11)
#define DHT21_ID                                                (21)
#define DHT22_ID                                                (22)
#define DHT33_ID                                                (33)
#define DHT44_ID                                                (44)
#define AM2301_ID                                               (21)
#define AM2302_ID                                               (22)


// max timeout is 100usec.
// For a 16Mhz proc that is max 1600 clock cycles
// loops using TIMEOUT use at least 4 clock cycli
// so 100 us takes max 400 loops
// so by dividing F_CPU by 40000 we "fail" as fast as possible
#ifndef F_CPU
  #define DHTLIB_TIMEOUT                                        (1000)  // ahould be approx. clock/40000
#else
  #define DHTLIB_TIMEOUT                                        (F_CPU/40000)
#endif

#define DHTLIB_DHT11_WAKEUP                                     (18)
#define DHTLIB_DHT_WAKEUP                                       (1)

/*****************************************************************************************************************************
*
*  Read specified metric's value of the AM/DHT sensor, put it to specified variable's address on success.
*
*  Returns: 
*     - RESULT_IS_FLOAT_01_DIGIT    on success
*     - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*     - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*     - DEVICE_ERROR_CHECKSUM       on detect data corruption
*     - DEVICE_ERROR_ACK_L
*     - DEVICE_ERROR_ACK_H
*
*****************************************************************************************************************************/
int8_t getDHTMetric(const uint8_t, const uint8_t, const uint8_t, int32_t*);
