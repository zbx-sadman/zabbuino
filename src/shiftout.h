#pragma once

/*****************************************************************************************************************************
*
*  Do bit-bang on the specified pin. Sub can get HEX-string as the data to shift out
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
int8_t shiftOutAdvanced(const uint8_t, const uint8_t, const uint8_t, const uint8_t, uint8_t*);

/*****************************************************************************************************************************
*
*  Push bitstream to WS2812 chip(s). Sub can get HEX-string as the data to shift out
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
int8_t WS2812Out(const uint8_t, const uint8_t, uint8_t*, uint16_t = 0x00);

/*****************************************************************************************************************************
*
*  Prepare buffer for fast bit-banging. Convert byte to two nibble if need to push DEC-value. 
*  Convert HEX-string to array of nibbles. Reverse array of nibbles (and reverse bit order in nibbles) if bitOrder is LSBFIRST.
*
*  Returns: 
*    - number of bytes in the prepared data buffer
*
*****************************************************************************************************************************/
int16_t prepareBufferForAdvShiftout(const uint8_t, const uint8_t, uint8_t*);

