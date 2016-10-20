#ifndef ZabbuinoUART_PZEM_h
#define ZabbuinoUART_PZEM_h

#include "uart_bus.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

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