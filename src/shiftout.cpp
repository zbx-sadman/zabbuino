// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "shiftout.h"

/*****************************************************************************************************************************
*
*  Do bit-bang on the specified pin. Sub can get HEX-string as the data to shift out
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
// in prepareBufferForAdvShiftout => _src[dataBufferPosition] = bitReverseTable16[(uint8_t) _src[dataBufferPosition]];
int8_t shiftOutAdvanced(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _bitOrder, const uint8_t _compressionType, uint8_t* _src) {
  int16_t lenOfBuffer;
  uint8_t dataPinBit, clockPinBit;
  volatile uint8_t *dataPortOutputRegister, *clockPortOutputRegister;
  uint8_t i;
  pinMode(_dataPin,OUTPUT);
  pinMode(_clockPin,OUTPUT);
  // Prepare the buffer for burst bit-banging
  lenOfBuffer = prepareBufferForAdvShiftout(_bitOrder, _compressionType, _src);
  if (lenOfBuffer <= 0) { return RESULT_IS_FAIL; }

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

  // Focus on bit-banging
  //noInterrupts();
  // Walk over the buffer 
  while (lenOfBuffer) {
     // Push 8 bit to Shift Register using direct port manipulation.
     i = 8;
     while (i)   {
       // Test 4-th but for HIGH/LOW state
       // if (*_src & 0x08) {
       if (*_src & 0x80) {
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
       // *_src = *_src << 1;
       *_src = *_src << 1;
       // bit counter increase
       i--;
     }        
     _src++;
     lenOfBuffer--;
  }

  //interrupts();
  gatherSystemMetrics(); // Measure memory consumption
  return RESULT_IS_OK;
}

/*****************************************************************************************************************************
*
*  Push bitstream to WS2812 chip(s). Sub can get HEX-string as the data to shift out
*
*   Returns: 
*     - none
*
* IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across pixel power leads, add 300 - 500 Ohm resistor on 
*            first pixel's data input and minimize distance between Arduino and first pixel.  
*            Avoid connecting on a live circuit...if you must, connect GND first.
*****************************************************************************************************************************/
int8_t WS2812Out(const uint8_t _dataPin, const uint8_t _compressionType, uint8_t* _src, uint16_t _len) 
{
  volatile uint8_t  *port;         // Output PORT register
  uint8_t pinMask;                 // Output PORT bitmask
  volatile uint16_t i;             // Loop counter
  volatile uint8_t
                   *ptr,           // Pointer to next byte
                   b,              // Current byte value
                   hi,             // PORT w/output bit set high
                   lo;             // PORT w/output bit set low
  volatile uint8_t next, bit;

  i = (0 >= _len) ? prepareBufferForAdvShiftout(MSBFIRST, _compressionType, _src) : _len; 

  if (i <= 0) {
     return RESULT_IS_FAIL;
  }
 
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, LOW);
  port    = portOutputRegister(digitalPinToPort(_dataPin));
  pinMask = digitalPinToBitMask(_dataPin);

  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  next = lo;
  // push by nibbles (4 bits pieces)
  bit  = 8;

  ptr = (uint8_t *) _src;
  b = *ptr++;

  noInterrupts(); // Need 100% focus on instruction timing

  // 16 MHz(ish) AVR ; (F_CPU >= 15400000UL) && (F_CPU <= 19000000L)

  // WS2811 and WS2812 have different hi/lo duty cycles; this is
  // similar but NOT an exact copy of the prior 400-on-8 code.
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)

  //  
  // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
  // ST instructions:         ^   ^        ^       (T=0,5,13)

   asm volatile (
     "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if(b & 128)
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
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
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
   return RESULT_IS_OK;
}

/*****************************************************************************************************************************
*
*  Prepare buffer for fast bit-banging. Convert byte to two nibble if need to push DEC-value. 
*  Convert HEX-string to array of nibbles. Reverse array of nibbles (and reverse bit order in nibbles) if bitOrder is LSBFIRST.
*
*  Returns: 
*    - number of bytes in the prepared data buffer
*
*****************************************************************************************************************************/
int16_t prepareBufferForAdvShiftout(const uint8_t _bitOrder, const uint8_t compressionType, uint8_t* _src)
{
  static const uint8_t bitReverseTable16[] = {
   // 0x00  0x01  0x02  0x03  0x04  0x05  0x06  0x07  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F       <- number
   // B0000 B0001 B0010 B0011 B0100 B0101 B0110 B0111 B1000 B1001 B1010 B1011 B1100 B1101 B1110 B1111      <- number in binary
   // B0000 B1000 B0100 B1100 B0010 B1010 B0110 B1110 B0001 B1001 B0101 B1101 B0011 B1011 B0111 B1111      <- number in binary reversed
      0x00, 0x08, 0x04, 0x0C, 0x02, 0x0A, 0x06, 0x0E, 0x01, 0x09, 0x05, 0x0D, 0x03, 0x0B, 0x07, 0x0F  //   <- number reversed
  };
  uint16_t dataBufferReadPosition, dataBufferWritePosition, dataBufferSwapPosition, halfLenOfBuffer, lenOfBuffer;
  uint8_t tmpVal;
  
  dataBufferReadPosition = 2;
  // Is HEX-string incoming?
  if (haveHexPrefix(_src)) {
     // Skip prefix
     // Walk over buffer, convert HEX do DEC and shift data to the left (destroy '0x' gap)
     while (_src[dataBufferReadPosition]) {
        _src[dataBufferReadPosition - 2] = htod(_src[dataBufferReadPosition]);
        dataBufferReadPosition++;
     }
     // Correct position to prefix length for taking buffer new lenght
     dataBufferReadPosition -= 2;
  } else {
     // Is not HEX, probally its DEC
     tmpVal = atoi((char*) _src);
     // Write first nibble to buffer[0]
     _src[0] = (tmpVal >> 4) & 0x0F;
     // Write last nibble to buffer[1] 
     _src[1] = tmpVal & 0x0F;
  }
  // lenght must be saved and used later because any HEX '0' will be converted to '\0' and stops while(*_src) processing 
  lenOfBuffer = dataBufferReadPosition;

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
     dataBufferReadPosition--;
     // Going over half of buffer
     while (halfLenOfBuffer){
       // swap buffer items 
       tmpVal = _src[dataBufferSwapPosition];
       _src[dataBufferSwapPosition] = _src[dataBufferReadPosition];
       _src[dataBufferReadPosition] = tmpVal;
       // shrink swapping area
       dataBufferReadPosition--;
       dataBufferSwapPosition++;
       halfLenOfBuffer--;
     } 
     // Make fast bit reversing for all items
     // That procedure is stand separately because one central item not processeed on previous stage if buffer length is odd
     // Using for() _here_ spend more pgmspace that using while() 
     for (dataBufferReadPosition = 0; dataBufferReadPosition < lenOfBuffer; dataBufferReadPosition++){
       _src[dataBufferReadPosition] = bitReverseTable16[(uint8_t) _src[dataBufferReadPosition]];
     }
  }

  // Prepared byte array must be transformed from HEX-string: two HEX-nibble joined to one byte
  switch (compressionType) {
      // Data will be 'decompressed' from HEX string by repeat every nibble twice: FD3 => FFDD33
      // This compression method allow to minimize len of recieved HEX-string when it used for color coding (RGB pixel leds and etc)
      case 0x01:
        __DMLD( DEBUG_PORT.println(F("'repeat' type compression ")); )
        for (dataBufferReadPosition = 0; dataBufferReadPosition <= lenOfBuffer; dataBufferReadPosition++) {
            _src[dataBufferReadPosition] = (_src[dataBufferReadPosition] << 4) | _src[dataBufferReadPosition];
        }
        break;  
      // Data is not compressed
      case 0x00:
      default:
        dataBufferReadPosition = dataBufferWritePosition=0x00;
        while (dataBufferReadPosition < lenOfBuffer)  {
          // On case of one nibble found at forward - just move nibble to high, correct the buffer length and jump out
          if ((dataBufferReadPosition + 2) > lenOfBuffer) {
             _src[dataBufferWritePosition] = (_src[dataBufferReadPosition] << 4);
             lenOfBuffer++; 
             break;
          // Two nibble available - make one byte
          } else {
             _src[dataBufferWritePosition] = ((_src[dataBufferReadPosition] << 4) | _src[dataBufferReadPosition + 1]);
          }
          dataBufferReadPosition += 2;
          dataBufferWritePosition++;
        }
        // after nibbles joining length of array is the half 
        lenOfBuffer = lenOfBuffer >> 1;  
        break;  
  }
  return lenOfBuffer;
}


