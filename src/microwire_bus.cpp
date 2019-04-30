/*

old:
Sketch uses 18,492 bytes (57%) of program storage space. Maximum is 32,256 bytes.
Global variables use 826 bytes (40%) of dynamic memory, leaving 1,222 bytes for local variables. Maximum is 2,048 bytes.

progmemed:
Sketch uses 18,320 bytes (56%) of program storage space. Maximum is 32,256 bytes.
Global variables use 826 bytes (40%) of dynamic memory, leaving 1,222 bytes for local variables. Maximum is 2,048 bytes.


'max7219.write[5,6,7,1,"0 .0 .0 .0 .0 .0"]'
Spended (ms): 4
Spended (us): 4168

Spended (us): 1456


zabbix_get -s 172.16.100.206 -k 'max7219.write[5,6,7,1,"UuUuUuUuUuU"]'
Spended (us): 3856
Spended (us): 1176


Sketch uses 18,820 bytes (58%) of program storage space. Maximum is 32,256 bytes.
Global variables use 878 bytes (42%) of dynamic memory, leaving 1,170 bytes for local variables. Maximum is 2,048 bytes.
*/
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

// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "microwire_bus.h"

/*****************************************************************************************************************************
*
*  Send one byte to MAX7219 controller
*
*  Returns: 
*    - none
*
*****************************************************************************************************************************/
static void writeByteToMAX7219(volatile uint8_t *_dataPinPOR, const uint8_t _dataPinBit, volatile uint8_t *_clockPinPOR, const uint8_t _clockPinBit, const uint8_t _data) 
{
  int8_t i = 0x08;
  while(i) {
    i--;                                                      // move to lesser bit
    *_clockPinPOR &= ~_clockPinBit;                             // tick
    // send i-th bit value 
    (_data & (0x01 << i)) ? *_dataPinPOR |= _dataPinBit : *_dataPinPOR &= ~_dataPinBit;
    *_clockPinPOR |= _clockPinBit;                               // tock
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
static void pushDataToMAX7219(volatile uint8_t *_dataPinPOR, const uint8_t _dataPinBit, volatile uint8_t *_clockPinPOR, const uint8_t _clockPinBit, volatile uint8_t *_loadPinPOR, const uint8_t _loadPinBit, const uint8_t _register, const uint8_t _data) {    
  *_loadPinPOR &= ~_loadPinBit;
  // specify register or column
  writeByteToMAX7219(_dataPinPOR, _dataPinBit, _clockPinPOR, _clockPinBit, _register);   
  // put data  
  writeByteToMAX7219(_dataPinPOR, _dataPinBit, _clockPinPOR, _clockPinBit, _data);
  // show it
  *_loadPinPOR |= _loadPinBit;
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
//  uint32_t startTime = micros();

  uint8_t dataPinBit, clockPinBit, loadPinBit;
  volatile uint8_t *dataPinPOR, *clockPinPOR, *loadPinPOR;

  dataPinBit  = digitalPinToBitMask(_dataPin);
  dataPinPOR  = portOutputRegister(digitalPinToPort(_dataPin));
  
  clockPinBit = digitalPinToBitMask(_clockPin); 
  clockPinPOR = portOutputRegister(digitalPinToPort(_clockPin));
  
  loadPinBit  = digitalPinToBitMask(_loadPin); 
  loadPinPOR  = portOutputRegister(digitalPinToPort(_loadPin));
  
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
  pinMode(_loadPin, OUTPUT);

  // Init the module 
  // Mark all columns as active
  pushDataToMAX7219(dataPinPOR, dataPinBit, clockPinPOR, clockPinBit, loadPinPOR, loadPinBit, MAX7219_REGISTER_SCANLIMIT, 0x07);      
  // No decode digits - led matrix mode, define active led segsments manually
  pushDataToMAX7219(dataPinPOR, dataPinBit, clockPinPOR, clockPinBit, loadPinPOR, loadPinBit, MAX7219_REGISTER_DECODE_MODE, 0x00);
  // Switch on IC
  pushDataToMAX7219(dataPinPOR, dataPinBit, clockPinPOR, clockPinBit, loadPinPOR, loadPinBit, MAX7219_REGISTER_SHUTDOWN, 0x01);
  // Switch off display test
  pushDataToMAX7219(dataPinPOR, dataPinBit, clockPinPOR, clockPinBit, loadPinPOR, loadPinBit, MAX7219_REGISTER_DISPLAYTEST, 0x00); // no display test
  // Set intensity
  pushDataToMAX7219(dataPinPOR, dataPinBit, clockPinPOR, clockPinBit, loadPinPOR, loadPinBit, MAX7219_REGISTER_INTENSITY, _intensity & 0x0F);    // the first 0x0f is the value you can set

  // Draw line by line from first column...
  col = 0x01;
  // MAX7219.write[5,6,7,1,"1.2.3.4.5.6.7.8"]
  // MAX7219.write[5,6,7,1,"    1.25"]
  // MAX7219.write[15,16,17,1,"Hc  -1.25"]
  // HEX strings must be processeed specially
  if (haveHexPrefix(_src)) {
     // Skip "0x"
     _src += 2;
     isHexString = true;
  } else {

    uint8_t dataBufferSwapPosition = 0x00;
    uint8_t lenOfBuffer = 0x00;
    while (_src[lenOfBuffer]) { lenOfBuffer++; }
    uint8_t dataBufferPosition = lenOfBuffer-1;
    lenOfBuffer = lenOfBuffer >> 1;
    // 
    while (lenOfBuffer){
       // swap buffer items 
       char tmpVal = _src[dataBufferSwapPosition];
       _src[dataBufferSwapPosition] = _src[dataBufferPosition];
       _src[dataBufferPosition] = tmpVal;
       // shrink swapping area
       dataBufferPosition--;
       dataBufferSwapPosition++;
       lenOfBuffer--;
     } 
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
      uint8_t letterDrawingsNo = arraySize(letterDrawings)+1;


/*
      //Serial.print("letterDrawingsNo: "); Serial.print(letterDrawingsNo); Serial.print(", sign: "); Serial.print (sign); 
      while (letterDrawingsNo) {
        letterDrawingsNo--;
        char sign = (char) pgm_read_byte(&(letterDrawings[letterDrawingsNo].sign));
           //Serial.print("letterDrawingsNo: "); Serial.print(letterDrawingsNo); Serial.print(", sign: "); Serial.print (sign); 
        if (sign == *_src) { break; }
      }
*/
      // Search the drawing No into reference array
      do { letterDrawingsNo--; } while (letterDrawingsNo && *_src != pgm_read_byte(&(letterDrawings[letterDrawingsNo].sign)));
      currByte = pgm_read_byte(&(letterDrawings[letterDrawingsNo].draw));
      //Serial.print(", currByte: "); Serial.println(currByte);

      // dot just skipped and processed on next step
      if ('.' == *_src) { goto next; }
      // 'dot' sign is prev? 
      if ('.' == ((char) *(_src-1))) { currByte |= 0x80; }
#endif
    }
    // Pushing byte to column
    pushDataToMAX7219(dataPinPOR, dataPinBit, clockPinPOR, clockPinBit, loadPinPOR, loadPinBit, col, currByte);

    // only 8 columns must be processeed, comment its if need more
    col++;
    if (0x08 < col) { break; }
    next:
    _src++;

  }
//  Serial.print("Spended (us): "); Serial.println(micros()-startTime);

  gatherSystemMetrics(); // Measure memory consumption
    
}


