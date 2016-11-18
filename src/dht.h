#ifndef ZabbuinoDHT_h
#define ZabbuinoDHT_h

/*
Based on: http://playground.arduino.cc/Main/DHTLib
version 0.1.13 is used

*/

#include <Arduino.h>
#include "defaults.h"
#include "service.h"
#include "../zabbuino.h"


#define DHT11_ID                                                11
#define DHT21_ID                                                21
#define DHT22_ID                                                22
#define DHT33_ID                                                33
#define DHT44_ID                                                44
#define AM2301_ID                                               21
#define AM2302_ID                                               22


#ifndef F_CPU
  #define DHTLIB_TIMEOUT                                        1000  // ahould be approx. clock/40000
#else
  #define DHTLIB_TIMEOUT                                        (F_CPU/40000)
#endif

#define DHTLIB_DHT11_WAKEUP                                     18
#define DHTLIB_DHT_WAKEUP                                       1

#define DHTLIB_DHT11_LEADING_ZEROS                              1
#define DHTLIB_DHT_LEADING_ZEROS                                6

/*****************************************************************************************************************************
*
*  Read specified metric's value of the AM/DHT sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_CONNECT on connection error
*     - DEVICE_ERROR_ACK_L
*     - DEVICE_ERROR_ACK_H
*     - DEVICE_ERROR_TIMEOUT if sensor stops answer to the request
*
*****************************************************************************************************************************/
int8_t getDHTMetric(const uint8_t _pin, const uint8_t _sensorModel, const uint8_t _metric, char *_dst);

#endif // #ifndef ZabbuinoDHT_h