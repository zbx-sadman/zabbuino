#ifndef ZabbuinoSHIFTOUT_h
#define ZabbuinoSHIFTOUT_h

#include <Arduino.h>
#include "../zabbuino.h"
#include "defaults.h"


/* ****************************************************************************************************************************
*
*  Advanced shiftOut()
*  Can get HEX-string as the data to shift out
*
**************************************************************************************************************************** */
void shiftOutAdvanced(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _bitOrder, char* _dataBuffer);

/* ****************************************************************************************************************************
*
*  Push bitstream to WS2812 chip(s)
*
**************************************************************************************************************************** */
void WS2812Out(const uint8_t _dataPin, char* _dataBuffer);

/* ****************************************************************************************************************************
*
*  Prepare buffer for fast shiftOut.
*  Convert byte to two nibble if need to push DEC-value
*  Convert HEX-string to array of nibbles
*  Reverse array of nibbles (and reverse bit order in nibbles) if bitOrder is LSBFIRST
*
**************************************************************************************************************************** */
uint16_t prepareBufferForAdvShiftout(const uint8_t _bitOrder, char* _dataBuffer);


#endif