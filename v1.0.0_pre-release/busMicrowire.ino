/*
Code History:
 --------------
 
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

#define MAX7219_NOOP               0x00
#define MAX7219_DIGIT_0            0x01
#define MAX7219_DIGIT_1            0x02
#define MAX7219_DIGIT_2            0x03
#define MAX7219_DIGIT_3            0x04
#define MAX7219_DIGIT_4            0x05
#define MAX7219_DIGIT_5            0x06
#define MAX7219_DIGIT_6            0x07
#define MAX7219_DIGIT_7            0x08
#define MAX7219_DECODE_MODE        0x09
#define MAX7219_INTENSITY          0x0a
#define MAX7219_SCANLIMIT          0x0b
#define MAX7219_SHUTDOWN           0x0c
#define MAX7219_DISPLAYTEST        0x0f


void max7219WriteByte(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _data) 
{
  int8_t i = 7;
  while(i >= 0) {
    digitalWrite(_clockPin, LOW);                     // tick
    digitalWrite(_dataPin, !!(_data & (0x01 << i)));  // send i-th bit value 
    digitalWrite(_clockPin, HIGH);                    // tock
    --i;                                              // move to lesser bit
  }
}

void max7219PushData(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _loadPin, const uint8_t _register, const uint8_t _data) {    
  digitalWrite(_loadPin, LOW);
  // specify register or column
  max7219WriteByte(_dataPin, _clockPin, _register);   
  // put data  
  max7219WriteByte(_dataPin, _clockPin, _data);
  // show it
  digitalWrite(_loadPin, LOW);
  digitalWrite(_loadPin,HIGH);
}

void max7219DrawOn8x8(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _loadPin, const uint8_t _intensity, char* _dataBuffer) {    
  uint8_t col, dataByte;
  // Init the module 
  // Mark all columns as active
  max7219PushData(_dataPin, _clockPin, _loadPin, MAX7219_SCANLIMIT, 0x07);      
  // No decode digits - led matrix mode
  max7219PushData(_dataPin, _clockPin, _loadPin, MAX7219_DECODE_MODE, 0x00);
  // Switch on IC
  max7219PushData(_dataPin, _clockPin, _loadPin, MAX7219_SHUTDOWN, 0x01);
  // Switch off display test
  max7219PushData(_dataPin, _clockPin, _loadPin, MAX7219_DISPLAYTEST, 0x00); // no display test
  // Set intensity
  max7219PushData(_dataPin, _clockPin, _loadPin, MAX7219_INTENSITY, _intensity & 0x0f);    // the first 0x0f is the value you can set

  // Draw line by line from first column...
  col = 1;
  // Only HEX strings is processeed
  if (haveHexPrefix(_dataBuffer)) {
    // Skip "0x"
    _dataBuffer += 2;
    while (*_dataBuffer) {
      // Make first four bits of byte to push from HEX.
      dataByte = htod(*_dataBuffer); _dataBuffer++;
      // Move first nibble to high
      dataByte <<= 4;
      // Check for second nibble existience
      if (*_dataBuffer) {
         // Add its to byte if HEX not '\0'
         dataByte |= htod(*_dataBuffer);
         _dataBuffer++;
      }
      // Pushing byte to column
      max7219PushData(_dataPin, _clockPin, _loadPin, col, dataByte);
      col++;
      // only 8 columns must be processeed
      if (0x08 < col) { break; }
    }
  }
}


