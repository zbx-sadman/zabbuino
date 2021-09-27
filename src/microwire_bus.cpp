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

#include "spi_bus.h"
#include "microwire_bus.h"

/*****************************************************************************************************************************
*
*  Push one byte of data to MAX7219 controller
*
*  Returns: 
*    - none
*
*****************************************************************************************************************************/
static void pushDataToMAX7219(const uint8_t spiType, volatile ioRegister_t* _mosiPinPOR, const uint8_t _mosiPinBit, volatile ioRegister_t* _sclkPinPOR, const uint8_t _sclkPinBit, volatile ioRegister_t* _loadPinPOR, const uint8_t _loadPinBit, const uint8_t _register, const uint8_t _data) {
  *_loadPinPOR &= ~_loadPinBit;
  // specify register or column
  spiWriteByte(spiType, _sclkPinPOR, _sclkPinBit, _mosiPinPOR, _mosiPinBit, _register);   
  // put data  
  spiWriteByte(spiType, _sclkPinPOR, _sclkPinBit, _mosiPinPOR,  _mosiPinBit, _data);
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
void writeToMAX7219(const uint8_t _mosiPin, const uint8_t _sclkPin, const uint8_t _loadPin, const uint8_t _intensity, uint8_t* _src) {
  uint8_t dataSize = 0x00, spiType = SPI_TYPE_SOFTWARE;

  ioRegister_t mosiPinBit = 0x00, sclkPinBit = 0x00, loadPinBit = 0x00;
  volatile ioRegister_t *mosiPinPOR = nullptr, *sclkPinPOR = nullptr, *loadPinPOR = nullptr;

#if defined(ARDUINO_ARCH_AVR)
  if (MOSI == _mosiPin && SCK == _sclkPin) {
     spiType = SPI_TYPE_HARDWARE;
  } else {
#endif
     mosiPinBit  = digitalPinToBitMask(_mosiPin);
     mosiPinPOR  = portOutputRegister(digitalPinToPort(_mosiPin));
 
     sclkPinBit = digitalPinToBitMask(_sclkPin); 
     sclkPinPOR = portOutputRegister(digitalPinToPort(_sclkPin));

     pinMode(_mosiPin, OUTPUT);
     pinMode(_sclkPin, OUTPUT);
#if defined(ARDUINO_ARCH_AVR)
  }  
#endif

  loadPinBit  = digitalPinToBitMask(_loadPin); 
  loadPinPOR  = portOutputRegister(digitalPinToPort(_loadPin));

  pinMode(_loadPin, OUTPUT);

  // Init the module 
  // Mark all columns as active
  pushDataToMAX7219(spiType, mosiPinPOR, mosiPinBit, sclkPinPOR, sclkPinBit, loadPinPOR, loadPinBit, MAX7219_REGISTER_SCANLIMIT, 0x07);      
  // No decode digits - led matrix mode, define active led segsments manually
  pushDataToMAX7219(spiType, mosiPinPOR, mosiPinBit, sclkPinPOR, sclkPinBit, loadPinPOR, loadPinBit, MAX7219_REGISTER_DECODE_MODE, 0x00);
  // Switch on IC
  pushDataToMAX7219(spiType, mosiPinPOR, mosiPinBit, sclkPinPOR, sclkPinBit, loadPinPOR, loadPinBit, MAX7219_REGISTER_SHUTDOWN, 0x01);
  // Switch off display test
  pushDataToMAX7219(spiType, mosiPinPOR, mosiPinBit, sclkPinPOR, sclkPinBit, loadPinPOR, loadPinBit, MAX7219_REGISTER_DISPLAYTEST, 0x00); // no display test
  // Set intensity
  pushDataToMAX7219(spiType, mosiPinPOR, mosiPinBit, sclkPinPOR, sclkPinBit, loadPinPOR, loadPinBit, MAX7219_REGISTER_INTENSITY, _intensity & 0x0F);    // the first 0x0f is the value you can set

  // HEX strings must be processeed specially
  if (haveHexPrefix(_src)) {
     uint8_t* ptrWritePosition = _src;
     // Skip "0x"
     uint8_t* ptrReadPosition  = _src + 0x02;

     // HEX processing
     while (*ptrReadPosition) {
       // Make first four bits of byte to push from HEX.
       *ptrWritePosition = htod(*ptrReadPosition); ptrReadPosition++;
       // Check for second nibble existience
       if (*ptrReadPosition) {
          // Move first nibble to high
          *ptrWritePosition <<= 4;
          // Add its to byte if HEX not '\0'
          *ptrWritePosition |= htod(*ptrReadPosition);
       }
     }   
     // it's ok for length calculate?
     dataSize = ptrReadPosition - _src;
     dataSize--;
  } else {

#ifndef NO_ASCII_SUPPORT
     uint8_t letterDrawingsNo;
     uint8_t* dotOwner         = nullptr;
     uint8_t* ptrWritePosition = _src;
     uint8_t* ptrReadPosition  = _src;

     while (*ptrReadPosition) {
       // dot just skipped and processed on next step
       if ('.' == *ptrReadPosition) { 
          // dotOwner is not nullptr if dot sign is not first char in the string
          if (dotOwner) { *dotOwner |= 0x80; }
       } else {
          letterDrawingsNo = arraySize(letterDrawings); // + 0x01;
          // Search the drawing No into reference array
          do { letterDrawingsNo--; } while (letterDrawingsNo && *ptrReadPosition != pgm_read_byte(&(letterDrawings[letterDrawingsNo].sign)));
          *ptrWritePosition = pgm_read_byte(&(letterDrawings[letterDrawingsNo].draw));
          dotOwner = ptrWritePosition;
          ptrWritePosition++;
          dataSize++;
       }
       ptrReadPosition++;
     }

     // start read from the end and write to the begin (ptrWritePosition and ptrReadPosition just reused)
     ptrWritePosition--;
     ptrReadPosition = _src;

     while (ptrWritePosition > ptrReadPosition){

       // swap buffer items 
       char tmpVal = *ptrReadPosition; *ptrReadPosition = *ptrWritePosition; *ptrWritePosition = tmpVal;
       // shrink swapping area
       ptrWritePosition--;
       ptrReadPosition++;
     } 
#endif
  }

  
  //!!! calc runtime !!!  
  // Draw line by line from first column...
  uint8_t col = 0x01;
  for (uint8_t i = 0x00; i < dataSize; i++) {  
    // Pushing byte to column
    pushDataToMAX7219(spiType, mosiPinPOR, mosiPinBit, sclkPinPOR, sclkPinBit, loadPinPOR, loadPinBit, col, _src[i]);  
    // only 8 columns must be processeed, comment its if need more
    col++;
    if (0x08 < col) { break; }
  }
  gatherSystemMetrics(); // Measure memory consumption
    
}


