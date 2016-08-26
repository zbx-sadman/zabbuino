/* ****************************************************************************************************************************
*
*  Advanced shiftOut()
*  Can get HEX-string as the data to shift out
*
**************************************************************************************************************************** */
void shiftOutAdvanced(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _bitOrder, char* _dataBuffer)
{
  uint16_t lenOfBuffer = 0;
  uint8_t dataPinBit, clockPinBit;
  volatile uint8_t *dataPortOutputRegister, *clockPortOutputRegister;
  uint8_t oldSREG, i, currByte;

  dataPinBit = digitalPinToBitMask(_dataPin);
  clockPinBit = digitalPinToBitMask(_clockPin);
  dataPortOutputRegister = portOutputRegister(digitalPinToPort(_dataPin));
  clockPortOutputRegister = portOutputRegister(digitalPinToPort(_clockPin));

  /* from wiring_digital.c
  byte dataPinTimer = digitalPinToTimer(_dataPin);
  byte clockPinTimer = digitalPinToTimer(_clockPin);
  if (clockPinTimer != NOT_ON_TIMER) turnOffPWM(clockPinTimer);
  if (dataPinTimer != NOT_ON_TIMER) turnOffPWM(dataPinTimer);
  */

  // Prepare the buffer for burst bit-banging
  lenOfBuffer = prepareBufferForAdvShiftout(_bitOrder, _dataBuffer);
  // Focus on bit-banging
  noInterrupts();
  // Walk over the buffer to end ('\0' char)
  while (lenOfBuffer) {
     // Push 4 bit to Shift Register using direct port manipulation.
     i = 4;    
     while (i)   {
       // Test 4-th but for HIGH/LOW state
       if (*_dataBuffer & 0x08) {
         // Set _dataPin to HIGH
         *dataPortOutputRegister |= dataPinBit;
       } else  {
         // Set _dataPin to LOW
          *dataPortOutputRegister &= ~dataPinBit;
       }
       // blink by clockPin
       *clockPortOutputRegister &= ~clockPinBit;
       *clockPortOutputRegister |= clockPinBit;
       // shift pushed value to left to test previous bit
       *_dataBuffer = *_dataBuffer << 1;
       // bit counter increase
       i--;
      }
      // Move pointer to next value
      _dataBuffer++;
      lenOfBuffer--;
    }
    interrupts();
}

/* ****************************************************************************************************************************
*
*  Push bitstream to WS2812 chip(s)
*
**************************************************************************************************************************** */
void WS2812Out(const uint8_t _dataPin, char* _dataBuffer) 
{
  volatile uint8_t  *port;         // Output PORT register
  uint8_t pinMask;       // Output PORT bitmask
  volatile uint16_t i, lenOfBuffer;  // = numBytes; // Loop counter
  volatile uint8_t
                   *ptr, // = pixels,   // Pointer to next byte
                   b, //   = *ptr++,   // Current byte value
                   hi,             // PORT w/output bit set high
                   lo;             // PORT w/output bit set low
  volatile uint8_t next, bit;
   

  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, LOW);
  port    = portOutputRegister(digitalPinToPort(_dataPin));
  pinMask = digitalPinToBitMask(_dataPin);

  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  next = lo;
  // push by nibbles (4 bits pieces)
  bit  = 4;

  // Prepare the buffer for burst bit-banging
  i = prepareBufferForAdvShiftout(MSBFIRST, _dataBuffer);

  ptr = (uint8_t *) _dataBuffer;
  b = *ptr++;

  noInterrupts(); // Need 100% focus on instruction timing
  // WS2811 and WS2812 have different hi/lo duty cycles; this is
  // similar but NOT an exact copy of the prior 400-on-8 code.

  // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
  // ST instructions:         ^   ^        ^       (T=0,5,13)

    asm volatile(
     "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte],  3"         "\n\t" // 1-2  if(b & 8)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  4)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
      "st   %a[port],  %[next]"  "\n\t" // 2    PORT = next   (T =  7)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
      "breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
      "nop"                      "\n\t" // 1    nop           (T = 13)
      "st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
      "nop"                      "\n\t" // 1    nop           (T = 16)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
      "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
     "nextbyte20:"               "\n\t" //                    (T = 10)
      "ldi  %[bit]  ,  4"        "\n\t" // 1    bit = 4       (T = 11)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
      "st   %a[port], %[lo]"     "\n\t" // 2    PORT = lo     (T = 15)
      "nop"                      "\n\t" // 1    nop           (T = 16)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
       "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)
      : [port]  "+e" (port),
        [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
      : [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo)
    );

   interrupts(); 
}

/* ****************************************************************************************************************************
*
*  Prepare buffer for fast shiftOut.
*  Convert byte to two nibble if need to push DEC-value
*  Convert HEX-string to array of nibbles
*  Reverse array of nibbles (and reverse bit order in nibbles) if bitOrder is LSBFIRST
*
**************************************************************************************************************************** */
uint16_t prepareBufferForAdvShiftout(const uint8_t _bitOrder, char* _dataBuffer)
{
  static const uint8_t bitReverseTable16[] = {
   // 0x00  0x01  0x02  0x03  0x04  0x05  0x06  0x07  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F
   // B0000 B0001 B0010 B0011 B0100 B0101 B0110 B0111 B1000 B1001 B1010 B1011 B1100 B1101 B1110 B1111
   // B0000 B1000 B0100 B1100 B0010 B1010 B0110 B1110 B0001 B1001 B0101 B1101 B0011 B1011 B0111 B1111
      0x00, 0x08, 0x04, 0x0C, 0x02, 0x0A, 0x06, 0x0E, 0x01, 0x09, 0x05, 0x0D, 0x03, 0x0B, 0x07, 0x0F
  };
  uint16_t dataBufferPosition, dataBufferSwapPosition, halfLenOfBuffer, lenOfBuffer;
  uint8_t tmpVal;
  
  // Is HEX-string specified?
  if (haveHexPrefix(_dataBuffer)) {
     // Skip prefix
     dataBufferPosition = 2;
     // Walk over buffer and convert HEX do DEC
     while (_dataBuffer[dataBufferPosition]) {
        _dataBuffer[dataBufferPosition - 2] = htod(_dataBuffer[dataBufferPosition]);
        dataBufferPosition++;
     }
     // Correct position for taking buffer new lenght
     dataBufferPosition -= 2;
  } else {
     // Is not HEX, probally DEC
     dataBufferPosition = 1;
     tmpVal = atoi(_dataBuffer);
     // Write first nibble to buffer[0] 
     _dataBuffer[0] = tmpVal & 0x0F;
     // Write second nibble to buffer[1] if need
     if (tmpVal > 0x0F) {
        _dataBuffer[1] = tmpVal >> 4;
        // Correct position for taking buffer new lenght
        dataBufferPosition = 2;
     }
  }
  // lenght must be saved and used later because any HEX '0' will be converted to '\0' and stops while(*_dataBuffer) processing 
  lenOfBuffer = dataBufferPosition;
  // if reverse bit order specified - all bit chain must be reversed
  if (LSBFIRST == _bitOrder) {
     halfLenOfBuffer = lenOfBuffer >> 1;
     dataBufferSwapPosition = 0;
     dataBufferPosition--;
     // Going over half of buffer
     while (halfLenOfBuffer){
       // swap buffer items 
       tmpVal = _dataBuffer[dataBufferSwapPosition];
       _dataBuffer[dataBufferSwapPosition] = _dataBuffer[dataBufferPosition];
       _dataBuffer[dataBufferPosition] = tmpVal;
       // shrink swapping area
       dataBufferPosition--;
       dataBufferSwapPosition++;
       halfLenOfBuffer--;
     } 
     // Make fast bit reversing for all items
     // That procedure is stand separately because one central item not processeed on previous stage if buffer length is odd
     dataBufferPosition = lenOfBuffer;
     while (dataBufferPosition){
       _dataBuffer[dataBufferPosition] = bitReverseTable16[_dataBuffer[dataBufferPosition]];
       dataBufferPosition--;
     }
  }
  return lenOfBuffer;
}


