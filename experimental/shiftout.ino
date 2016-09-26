/* ****************************************************************************************************************************
*
*  Advanced shiftOut()
*  Can get HEX-string as the data to shift out
*
**************************************************************************************************************************** */
void shiftOutAdvanced(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _bitOrder, char* _src)
{
  uint16_t lenOfBuffer = 0;
  uint8_t dataPinBit, clockPinBit;
  volatile uint8_t *dataPortOutputRegister, *clockPortOutputRegister;
  uint8_t i, currByte;

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
  lenOfBuffer = prepareBufferForAdvShiftout(_bitOrder, _src);
  // Focus on bit-banging
  noInterrupts();
  // Walk over the buffer 
  while (lenOfBuffer) {
     // Push 4 bit to Shift Register using direct port manipulation.
     i = 4;    
     while (i)   {
       // Test 4-th but for HIGH/LOW state
       if (*_src & 0x08) {
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
       *_src = *_src << 1;
       // bit counter increase
       i--;
      }
      // Move pointer to next value
      _src++;
      lenOfBuffer--;
    }
    interrupts();
}

/* ****************************************************************************************************************************
*
*  Push bitstream to WS2812 chip(s)
*
**************************************************************************************************************************** */
void WS2812Out(const uint8_t _dataPin, char* _src) 
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
  i = prepareBufferForAdvShiftout(MSBFIRST, _src);

  ptr = (uint8_t *) _src;
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
uint16_t prepareBufferForAdvShiftout(const uint8_t _bitOrder, char* _src)
{
  static const uint8_t bitReverseTable16[] = {
   // 0x00  0x01  0x02  0x03  0x04  0x05  0x06  0x07  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F       <- number
   // B0000 B0001 B0010 B0011 B0100 B0101 B0110 B0111 B1000 B1001 B1010 B1011 B1100 B1101 B1110 B1111      <- number in binary
   // B0000 B1000 B0100 B1100 B0010 B1010 B0110 B1110 B0001 B1001 B0101 B1101 B0011 B1011 B0111 B1111      <- number in binary reversed
      0x00, 0x08, 0x04, 0x0C, 0x02, 0x0A, 0x06, 0x0E, 0x01, 0x09, 0x05, 0x0D, 0x03, 0x0B, 0x07, 0x0F  //   <- number reversed
  };
  uint16_t dataBufferPosition, dataBufferSwapPosition, halfLenOfBuffer, lenOfBuffer;
  uint8_t tmpVal;
  
  dataBufferPosition = 2;
  // Is HEX-string incoming?
  if (haveHexPrefix(_src)) {
     // Skip prefix
     // Walk over buffer, convert HEX do DEC and shift data to the left (destroy '0x' gap)
     while (_src[dataBufferPosition]) {
        _src[dataBufferPosition - 2] = htod(_src[dataBufferPosition]);
        dataBufferPosition++;
     }
     // Correct position to prefix length for taking buffer new lenght
     dataBufferPosition -= 2;
  } else {
     // Is not HEX, probally its DEC
     tmpVal = atoi(_src);
     // Write first nibble to buffer[0]
     _src[0] = tmpVal >> 4;
     // Write last nibble to buffer[1] 
     _src[1] = tmpVal & 0x0F;
  }
  // lenght must be saved and used later because any HEX '0' will be converted to '\0' and stops while(*_src) processing 
  lenOfBuffer = dataBufferPosition;
  //    <---  LSB  ----
  //    0 0 0 1 0 0 1 0     <= 18 (0x12)
  //    ----  MSB  --->
  // The expressions Most Significant Bit First and Least Significant Bit First are indications on the ordering of the sequence of the bits in the bytes sent over a wire in a transmission protocol or in a stream (e.g. an audio stream).
  // Most Significant Bit First means that the most significant bit will arrive first: 
  //      hence e.g. the hexadecimal number 0x12, 00010010 in binary representation, will arrive as the sequence 0 0 0 1 0 0 1 0 .
  // Least Significant Bit First means that the least significant bit will arrive first: 
  //      hence e.g. the same hexadecimal number 0x12, again 00010010 in binary representation, will arrive as the (reversed) sequence 0 1 0 0 1 0 0 0.
  //
  // if reverse bit order specified - all bit chain must be reversed
  if (LSBFIRST == _bitOrder) {
     halfLenOfBuffer = lenOfBuffer >> 1;
     dataBufferSwapPosition = 0;
     dataBufferPosition--;
     // Going over half of buffer
     while (halfLenOfBuffer){
       // swap buffer items 
       tmpVal = _src[dataBufferSwapPosition];
       _src[dataBufferSwapPosition] = _src[dataBufferPosition];
       _src[dataBufferPosition] = tmpVal;
       // shrink swapping area
       dataBufferPosition--;
       dataBufferSwapPosition++;
       halfLenOfBuffer--;
     } 
     // Make fast bit reversing for all items
     // That procedure is stand separately because one central item not processeed on previous stage if buffer length is odd
     // usung for() _here_ give more pgmspace that using while() 
     for (dataBufferPosition = 0; dataBufferPosition <= lenOfBuffer; dataBufferPosition++){
       _src[dataBufferPosition] = bitReverseTable16[_src[dataBufferPosition]];
     }
  }
  return lenOfBuffer;
}


