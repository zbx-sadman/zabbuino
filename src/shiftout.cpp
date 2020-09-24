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
  volatile ioRegister_t *dataPortOutputRegister, *clockPortOutputRegister;
  uint8_t i;
  pinMode(_dataPin,OUTPUT);
  pinMode(_clockPin,OUTPUT);
  // Prepare the buffer for burst bit-banging
  lenOfBuffer = prepareBufferForAdvShiftout(_bitOrder, _compressionType, _src);
  if (0x00 >= lenOfBuffer) { return RESULT_IS_FAIL; }

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
     i = 0x08;
     while (i)   {
       yield();
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
       *_src = *_src << 0x01;
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
#if defined(ARDUINO_ARCH_ESP8266)  
static uint32_t _getCycleCount(void) __attribute__((always_inline));
static inline uint32_t _getCycleCount(void) {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount":"=a" (ccount));
  return ccount;
}
#endif


int8_t WS281xOut(const uint8_t _dataPin, const uint8_t _bitstream800KHz, const uint8_t _compressionType, uint8_t* _src, uint16_t _len)  {

  volatile ioRegister_t  *port;         // Output PORT register
  ioRegister_t pinMask;                 // Output PORT bitmask

  volatile uint16_t dataSize;           // Loop counter
  volatile uint8_t *ptrData;            // Pointer to next byte
  volatile uint8_t currentByte, currentBit;

  dataSize = (0x00 >= _len) ? prepareBufferForAdvShiftout(MSBFIRST, _compressionType, _src) : _len; 

  if (0x00  >= dataSize ) {
     return RESULT_IS_FAIL;
  }
 
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, LOW);

  port    = portOutputRegister(digitalPinToPort(_dataPin));
  pinMask = digitalPinToBitMask(_dataPin);

  ptrData = _src;
  currentByte = *ptrData++;

#if defined(ARDUINO_ARCH_AVR)
  volatile uint8_t 
           hi = *port |  pinMask,             // PORT w/output bit set high
           lo = *port & ~pinMask;             // PORT w/output bit set low
  volatile uint8_t next = lo;

  currentBit = 0x08;

  noInterrupts(); // Need 100% focus on instruction timing

  // 16 MHz(ish) AVR ; (F_CPU >= 15400000UL) && (F_CPU <= 19000000L)

  // WS2811 and WS2812 have different hi/lo duty cycles; this is
  // similar but NOT an exact copy of the prior 400-on-8 code.
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)

  //  
  // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
  // ST instructions:         ^   ^        ^       (T=0,5,13)

  if (_bitstream800KHz) {

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
        [byte]  "+r" (currentByte),
        [bit]   "+r" (currentBit),
        [next]  "+r" (next),
        [count] "+w" (dataSize)
      : [ptr]    "e" (ptrData),
        [hi]     "r" (hi),
        [lo]     "r" (lo)
   );

} else {

    // The 400 KHz clock on 16 MHz MCU is the most 'relaxed' version.

    // 40 inst. clocks per bit: HHHHHHHHxxxxxxxxxxxxLLLLLLLLLLLLLLLLLLLL
    // ST instructions:         ^       ^           ^         (T=0,8,20)

    asm volatile(
     "head40:"                  "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port], %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 128)
       "mov  %[next] , %[hi]"   "\n\t" // 0-1   next = hi    (T =  4)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T =  6)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T =  8)
      "st   %a[port], %[next]"  "\n\t" // 2    PORT = next   (T = 10)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 12)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 14)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 16)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 18)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 20)
      "st   %a[port], %[lo]"    "\n\t" // 2    PORT = lo     (T = 22)
      "nop"                     "\n\t" // 1    nop           (T = 23)
      "mov  %[next] , %[lo]"    "\n\t" // 1    next = lo     (T = 24)
      "dec  %[bit]"             "\n\t" // 1    bit--         (T = 25)
      "breq nextbyte40"         "\n\t" // 1-2  if(bit == 0)
      "rol  %[byte]"            "\n\t" // 1    b <<= 1       (T = 27)
      "nop"                     "\n\t" // 1    nop           (T = 28)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 30)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 32)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 34)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 36)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 38)
      "rjmp head40"             "\n\t" // 2    -> head40 (next bit out)
     "nextbyte40:"              "\n\t" //                    (T = 27)
      "ldi  %[bit]  , 8"        "\n\t" // 1    bit = 8       (T = 28)
      "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 30)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 32)
      "st   %a[port], %[lo]"    "\n\t" // 2    PORT = lo     (T = 34)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 36)
      "sbiw %[count], 1"        "\n\t" // 2    i--           (T = 38)
      "brne head40"             "\n"   // 1-2  if(i != 0) -> (next byte)
      : [port]  "+e" (port),
        [byte]  "+r" (currentByte),
        [bit]   "+r" (currentBit),
        [next]  "+r" (next),
        [count] "+w" (dataSize)
      : [ptr]    "e" (ptrData),
        [hi]     "r" (hi),
        [lo]     "r" (lo));

}

   interrupts(); 
#elif defined(ARDUINO_ARCH_ESP8266)  

  uint8_t *endData = _src + dataSize;
  uint32_t t, time0, time1, period, c, startTime;

  currentBit = 0x80;

  startTime = 0x00;

  if (_bitstream800KHz) {
    time0  = CYCLES_800_T0H;
    time1  = CYCLES_800_T1H;
    period = CYCLES_800;
  } else { // 400 KHz bitstream
    time0  = CYCLES_400_T0H;
    time1  = CYCLES_400_T1H;
    period = CYCLES_400;
  }

  for (t = time0;; t = time0) {
    if (currentByte & currentBit) t = time1;               // Bit high duration
    while (((c = _getCycleCount()) - startTime) < period); // Wait for bit start
    *port |= pinMask;                                      // Pin HIGH
    startTime = c;                                         // Save start time
    while (((c = _getCycleCount()) - startTime) < t);      // Wait high duration
    *port &= ~pinMask;                                     // Pin LOW
    if (!(currentBit >>= 1)) {                             // Next bit/byte
      if (ptrData >= endData) {
        break;
      }
      currentByte = *ptrData++;
      currentBit = 0x80;
    }
  }
  while ((_getCycleCount() - startTime) < period);          // Wait for last bit

#endif

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
  
  dataBufferReadPosition = 0x02;
  // Is HEX-string incoming?
  if (haveHexPrefix(_src)) {
     // Skip prefix
     // Walk over buffer, convert HEX do DEC and shift data to the left (destroy '0x' gap)
     while (_src[dataBufferReadPosition]) {
        yield();
        _src[dataBufferReadPosition - 0x02] = htod(_src[dataBufferReadPosition]);
        dataBufferReadPosition++;
     }
     // Correct position to prefix length for taking buffer new lenght
     dataBufferReadPosition -= 0x02;
  } else {
     // Is not HEX, probally its DEC
     tmpVal = atoi((char*) _src);
     // Write first nibble to buffer[0]
     _src[0x00] = (tmpVal >> 0x04) & 0x0F;
     // Write last nibble to buffer[1] 
     _src[0x01] = tmpVal & 0x0F;
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
     halfLenOfBuffer = lenOfBuffer >> 0x01;
     dataBufferSwapPosition = 0x00;
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
     for (dataBufferReadPosition = 0x00; dataBufferReadPosition < lenOfBuffer; dataBufferReadPosition++){
       yield();
       _src[dataBufferReadPosition] = bitReverseTable16[(uint8_t) _src[dataBufferReadPosition]];
     }
  }

  // Prepared byte array must be transformed from HEX-string: two HEX-nibble joined to one byte
  switch (compressionType) {
      // Data will be 'decompressed' from HEX string by repeat every nibble twice: FD3 => FFDD33
      // This compression method allow to minimize len of recieved HEX-string when it used for color coding (RGB pixel leds and etc)
      case 0x01:
        __DMLD( DEBUG_PORT.println(F("'repeat' type compression ")); )
        for (dataBufferReadPosition = 0x00; dataBufferReadPosition <= lenOfBuffer; dataBufferReadPosition++) {
            yield();
            _src[dataBufferReadPosition] = (_src[dataBufferReadPosition] << 0x04) | _src[dataBufferReadPosition];
        }
        break;  
      // Data is not compressed
      case 0x00:
      default:
        dataBufferReadPosition = dataBufferWritePosition=0x00;
        while (dataBufferReadPosition < lenOfBuffer)  {
          yield();
          // On case of one nibble found at forward - just move nibble to high, correct the buffer length and jump out
          if ((dataBufferReadPosition + 2) > lenOfBuffer) {
             _src[dataBufferWritePosition] = (_src[dataBufferReadPosition] << 0x04);
             lenOfBuffer++; 
             break;
          // Two nibble available - make one byte
          } else {
             _src[dataBufferWritePosition] = ((_src[dataBufferReadPosition] << 0x04) | _src[dataBufferReadPosition + 0x01]);
          }
          dataBufferReadPosition += 0x02;
          dataBufferWritePosition++;
        }
        // after nibbles joining length of array is the half 
        lenOfBuffer = lenOfBuffer >> 0x01;  
        break;  
  }
  return lenOfBuffer;
}


