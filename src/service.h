#ifndef ZabbuinoSERVICE_h
#define ZabbuinoSERVICE_h

#include <Arduino.h>
#include "defaults.h"
#include "../zabbuino.h"
	
void setConfigDefaults(netconfig_t* _configStruct);

/* ****************************************************************************************************************************
*
*   Convert int32_t _number to char[]  with decimal point on _num_after_dot position 
*   _number / (10 * _num_after_dot position) => char[]
*
**************************************************************************************************************************** */
void ltoaf(const int32_t _number, char* _dst, const uint8_t _num_after_dot);

/* ****************************************************************************************************************************
*
*   Convert _Qm.n_ float number (int64_t) to char[] 
*
**************************************************************************************************************************** */
uint8_t qtoaf(const int64_t _number, char* _dst, uint8_t _fracBits);

/* ****************************************************************************************************************************
*
*  Convert _len chars (exclude 0x prefix) of hex string to byte array
*
**************************************************************************************************************************** */
uint8_t hstoba(uint8_t* _dst, const char* _src, uint8_t _len);


/* ****************************************************************************************************************************
*
*   Compute a Dallas Semiconductor 8 bit CRC directly. This is much slower, but much smaller, than the lookup table.
*
*   This function placed here to aviod compilation error when OneWire library is not #included
*
**************************************************************************************************************************** */
uint8_t dallas_crc8(const uint8_t *addr, uint8_t len);

/* ****************************************************************************************************************************
*
*  Print string stored in PROGMEM to Serial 
*
**************************************************************************************************************************** */
void SerialPrint_P (const char* _src);

/* ****************************************************************************************************************************
*
*  Print string stored in PROGMEM to Serial + Line Feed
*
**************************************************************************************************************************** */
void SerialPrintln_P (const char* _src);

/* ****************************************************************************************************************************
*
*   Convert array of char with given lenght to uint32_t variable
*
**************************************************************************************************************************** */
uint32_t pl_atol(char const *_src, uint8_t _len);

/* ****************************************************************************************************************************
*
*  Print array to Serial as MAC or IP or other string with sign-separated parts
*
**************************************************************************************************************************** */
void printArray(uint8_t *_src, uint8_t _len, const uint8_t _type);

void blinkMore(const uint8_t _times, const uint16_t _onTime, const uint16_t _offTime);

#endif