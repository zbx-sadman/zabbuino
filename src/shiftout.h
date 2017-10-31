#ifndef _ZABBUINO_SHIFTOUT_H_
#define _ZABBUINO_SHIFTOUT_H_


/*****************************************************************************************************************************
*
*  Do bit-bang on the specified pin. Sub can get HEX-string as the data to shift out
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
int8_t shiftOutAdvanced(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _bitOrder, const uint8_t _compressionType, uint8_t* _src);

/*****************************************************************************************************************************
*
*  Push bitstream to WS2812 chip(s). Sub can get HEX-string as the data to shift out
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
<<<<<<< HEAD
int8_t WS2812Out(const uint8_t _dataPin, const uint8_t _compressionType, uint8_t* _src, uint16_t _len = 0);
=======
int8_t WS2812Out(const uint8_t _dataPin, const uint8_t _compressionType, uint8_t* _src);
>>>>>>> origin/experimental

/*****************************************************************************************************************************
*
*  Prepare buffer for fast bit-banging. Convert byte to two nibble if need to push DEC-value. 
*  Convert HEX-string to array of nibbles. Reverse array of nibbles (and reverse bit order in nibbles) if bitOrder is LSBFIRST.
*
*  Returns: 
*    - number of bytes in the prepared data buffer
*
*****************************************************************************************************************************/
int16_t prepareBufferForAdvShiftout(const uint8_t _bitOrder, const uint8_t _compressionType, uint8_t* _src);

#endif // #ifndef _ZABBUINO_SHIFTOUT_H_