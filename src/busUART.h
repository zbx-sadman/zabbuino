#ifndef ZabbuinoBUSUART_h
#define ZabbuinoBUSUART_h

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "../zabbuino.h"
#include "service.h"
#include "system.h"
#include "defaults.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                      COMMON UART SECTION
*/

uint8_t serialRXFlush(SoftwareSerial* _swSerial, const uint8_t _slowMode);
uint8_t serialRecive(SoftwareSerial* _swSerial, uint8_t* _src, const uint8_t _size, const uint32_t _readTimeout, const uint8_t _stopOn, const uint8_t _slowMode);
uint8_t serialSend(SoftwareSerial* _swSerial, const uint8_t* _src, const uint8_t _size, const uint8_t _slowMode);
/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           Megatec protocol compatible UPS SECTION

   http://networkupstools.org/protocols/megatec.html
*/
#define MEGATEC_UPS_UART_SPEED                2400 // Megatec-compatible UPS works on 2400 baud speed
#define MEGATEC_MAX_ANSWER_LENGTH             50   // Read no more 50 chars from UPS
#define MEGATEC_DEFAULT_READ_TIMEOUT          1000L

int8_t getMegatecUPSMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _command, uint8_t _fieldNumber, uint8_t* _dst);

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           APC SMART UPS SECTION

   Despite the lack of official information from APC, this table has been constructed. It’s standard RS-232 serial communications at 2400 bps/8N1. 
   Don’t rush the UPS while transmitting or it may stop talking to you. This isn’t a problem with the normal single character queries, but it really 
   does matter for multi-char things like "@000". Sprinkle a few calls to usleep() in your code and everything will work a lot better.
   http://networkupstools.org/protocols/apcsmart.html
*/

#define APC_UPS_UART_SPEED                2400 // APC UPS works on 2400 baud speed
#define APC_MAX_ANSWER_LENGTH             30   // Read no more 30 chars from UPS
#define APC_DEFAULT_READ_TIMEOUT          1000L

int8_t getAPCSmartUPSMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _command, uint8_t _commandLen,  uint8_t* _dst);

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           PZEM-004 SECTION
*/

/*
Based on: https://github.com/olehs/PZEM004T
version 1.0 is used

*/

#define PZEM_UART_SPEED                    9600 // baud

#define PZEM_VOLTAGE                       0xB0
#define PZEM_CURRENT                       0xB1
#define PZEM_POWER                         0xB2
#define PZEM_ENERGY                        0xB3
#define PZEM_PACKET_SIZE                   0x07
#define PZEM_DEFAULT_READ_TIMEOUT          1000L

uint8_t crcPZEM004(uint8_t* _data, uint8_t _size);
int8_t getPZEM004Metric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const char* _ip, uint8_t* _dst);

#endif