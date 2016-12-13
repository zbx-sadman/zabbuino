/* 
The original code was written for the Wiring board by:
 * Nicholas Zambetti and Dave Mellis /Interaction Design Institute Ivrea /Dec 2004
 * http://www.potemkin.org/uploads/Wiring/MAX7219.txt
 
First modification by:
 * Marcus Hannerstig/  K3, malmö högskola /2006
 * http://www.xlab.se | http://arduino.berlios.de
 
Second modification is by:
 * tomek ness /FH-Potsdam / Feb 2007
 * http://design.fh-potsdam.de/

*/
#include "busMicrowire.h"

/*****************************************************************************************************************************
*
*  Send one byte to MAX7219 controller
*
*  Returns: 
*    - none
*
*****************************************************************************************************************************/
static void writeByteToMAX7219(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _data) 
{
  int8_t i = 7;
  while(i >= 0) {
    digitalWrite(_clockPin, LOW);                     // tick
    digitalWrite(_dataPin, !!(_data & (0x01 << i)));  // send i-th bit value 
    digitalWrite(_clockPin, HIGH);                    // tock
    --i;                                              // move to lesser bit
  }
}

/*****************************************************************************************************************************
*
*  Push one byte of data to MAX7219 controller
*
*  Returns: 
*    - none
*
*****************************************************************************************************************************/
static void pushDataToMAX7219(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _loadPin, const uint8_t _register, const uint8_t _data) {    
  digitalWrite(_loadPin, LOW);
  // specify register or column
  writeByteToMAX7219(_dataPin, _clockPin, _register);   
  // put data  
  writeByteToMAX7219(_dataPin, _clockPin, _data);
  // show it
  digitalWrite(_loadPin, LOW);
  digitalWrite(_loadPin,HIGH);
}

/*****************************************************************************************************************************
*
*  Print incoming data on MAX7219 based digital led indicator or draw on led matrix.
*
*  Returns: 
*    - none
*
*****************************************************************************************************************************/
void writeToMAX7219(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _loadPin, const uint8_t _intensity, char* _src) {    
  uint8_t col, currByte = 0,  isHexString = false;
  // Init the module 
  // Mark all columns as active
  pushDataToMAX7219(_dataPin, _clockPin, _loadPin, MAX7219_REGISTER_SCANLIMIT, 0x07);      
  // No decode digits - led matrix mode, define active led segsments manually
  pushDataToMAX7219(_dataPin, _clockPin, _loadPin, MAX7219_REGISTER_DECODE_MODE, 0x00);
  // Switch on IC
  pushDataToMAX7219(_dataPin, _clockPin, _loadPin, MAX7219_REGISTER_SHUTDOWN, 0x01);
  // Switch off display test
  pushDataToMAX7219(_dataPin, _clockPin, _loadPin, MAX7219_REGISTER_DISPLAYTEST, 0x00); // no display test
  // Set intensity
  pushDataToMAX7219(_dataPin, _clockPin, _loadPin, MAX7219_REGISTER_INTENSITY, _intensity & 0x0F);    // the first 0x0f is the value you can set

  // Draw line by line from first column...
  col = 1;
  
  // HEX strings must be processeed specially
  if (haveHexPrefix(_src)) {
     // Skip "0x"
     _src += 2;
     isHexString = true;
  }
  
  while (*_src) {
    // HEX processing
    if (isHexString) {
       // Make first four bits of byte to push from HEX.
       currByte = htod(*_src); _src++;
       // Move first nibble to high
       currByte <<= 4;
       // Check for second nibble existience
       if (*_src) {
          // Add its to byte if HEX not '\0'
          currByte |= htod(*_src);
       }
#ifndef NO_ASCII_SUPPORT
    } else {
      //
      //  ASCII processing
      //
      // currByte '1111110' =>  LED SEG 'ABCDEFG' , if DP must be fired up - currByte |= 0x80
      //    AAAA
      //   F    B     
      //   F    B    
      //    GGGG
      //   E    C
      //   E    C
      //    DDDD   DP
      //
      char currChar=(char) *_src;
      switch (currChar) {
         case '0':
            currByte = B1111110;
            break;
         case '1':
            currByte = B0110000;
            break;
         case '2':
            currByte = B1101101;
            break;
         case '3':
            currByte = B1111001;
            break;
         case '4':
            currByte = B0110011;
            break;
         case '5':
            currByte = B1011011;
            break;
         case '6':
            currByte = B1011111;
            break;
         case '7':
            currByte = B1110000;
            break;
         case '8':
            currByte = B1111111;
            break;
         case '9':
            currByte = B1111011;
            break;
         case 0x20:
            currByte = B0000000;
            break;
         case '-':
            currByte = B0000001;
            break;
         case 'C':
            currByte = B1001110;
            break;
         case 'c':
            currByte = B0001101;
            break;
         case 'H':
            currByte = B0110111;
            break;
         case 'h':
            currByte = B0010110;
            break;
         case 'E':
            currByte = B1001111;
            break;
         case 'L':
            currByte = B0001110;
            break;
         case 'l':
            currByte = B0000110;
            break;
         case 'P':
            currByte = B1100111;
            break;
         case 'n':
            currByte = B0010101;
            break;
         case 'o':
            currByte = B0011101;
            break;
         case 'r':
            currByte = B0000101;
            break;
         default:
            currByte = B0000000;
            break;
     }
      // 'dot' sign is next? 
      if ('.' == ((char) *(_src+1))) {
         currByte |= 0x80;
         _src++;
      }
#endif
    }
    _src++;
    // Pushing byte to column
     pushDataToMAX7219(_dataPin, _clockPin, _loadPin, col, currByte);
    col++;
    // only 8 columns must be processeed, comment its if need more
    if (0x08 < col) { break; }
  }
  gatherSystemMetrics(); // Measure memory consumption
  
}


