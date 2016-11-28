#ifndef ZabbuinoUART_BUS_h
#define ZabbuinoUART_BUS_h

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "../zabbuino.h"
#include "defaults.h"
#include "service.h"
#include "system.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                      COMMON UART SECTION

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
#define UART_SLOW_MODE                                        true
#define UART_STOP_ON_CHAR                                     true


/*****************************************************************************************************************************
*
*   Flush the SoftSerial's buffer
*
*   Returns: 
*     - always true
*
*****************************************************************************************************************************/
uint8_t serialRXFlush(SoftwareSerial *_swSerial, const uint8_t _slowMode);

/*****************************************************************************************************************************
*
*   Read data from the SoftSerial's buffer
*
*   Returns: 
*     - The number of the readed bytes
*
*****************************************************************************************************************************/
uint8_t serialRecive(SoftwareSerial *_swSerial, uint8_t *_src, const uint8_t _size, const uint32_t _readTimeout, const uint8_t _stopOnChar, const uint8_t _stopChar, const uint8_t _slowMode);

/*****************************************************************************************************************************
*
*   Write data to the SoftSerial's buffer
*
*   Returns: 
*     - none, because wrapped SoftwareSerial(HardwareSerial)'s write() sub always return 1
*
*****************************************************************************************************************************/
void serialSend(SoftwareSerial *_swSerial, const uint8_t *_src, const uint8_t _size, const uint8_t _slowMode);

#endif // #ifndef ZabbuinoUART_BUS_h