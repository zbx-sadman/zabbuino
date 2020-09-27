#pragma once

#if defined(ARDUINO_ARCH_AVR)
  #include <SoftwareSerial.h>
#elif defined(ARDUINO_ARCH_ESP8266) || defined (ARDUINO_ARCH_ESP32)
  #include "EspSoftwareSerial/SoftwareSerial.h"
#endif

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
//uint8_t serialRXFlush(Stream*, const uint8_t);

/*****************************************************************************************************************************
*
*   Read data from the SoftSerial's buffer
*
*   Returns: 
*     - The number of the readed bytes
*
*****************************************************************************************************************************/
uint8_t serialRecive(Stream*, uint8_t*, const uint8_t, const uint32_t, const uint8_t, const uint8_t, const uint8_t);

/*****************************************************************************************************************************
*
*   Write data to the SoftSerial's buffer
*
*   Returns: 
*     - none, because wrapped SoftwareSerial(HardwareSerial)'s write() sub always return 1
*
*****************************************************************************************************************************/
void serialSend(Stream*, uint8_t*, uint8_t, const uint8_t);

