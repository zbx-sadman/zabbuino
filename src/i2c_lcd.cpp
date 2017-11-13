// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_lcd.h"


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

Original code on: https://github.com/marcoschwartz/LiquidCrystal_I2C
version 1.1.2 is used

*/

/*****************************************************************************************************************************
*
*  Make pulse action for HD44780
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void pulseEnableOnLCD(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _data)
{
  uint8_t sendByte;
  sendByte = _data | _BV(LCD_E);
  writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, &sendByte, 1);	// 'Enable' high
  delayMicroseconds(1);		                        // enable pulse must be >450ns
  sendByte = _data | (_data & ~_BV(LCD_E));
  writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, &sendByte, 1);	// 'Enable' low
  delayMicroseconds(50);	                        // commands need > 37us to settle
} 

/*****************************************************************************************************************************
*
*  Write four bits to I2C extender (if HD44780 controller is used in 4-bit mode)
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void write4bitsToLCD(SoftwareWire* _softTWI, const uint8_t _i2cAddress, uint8_t _data) 
{
  writeBytesToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, &_data, 1);
  pulseEnableOnLCD(_softTWI, _i2cAddress, _data);
}

/*****************************************************************************************************************************
*
*  Send data to I2C extender
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void sendToLCD(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint8_t _data, const uint8_t _mode)
{
  // Send first nibble (first four bits) into high byte area
  write4bitsToLCD(_softTWI, _i2cAddress, (_data & 0xF0) | _mode);
  // Send second nibble (second four bits) into high byte area
  write4bitsToLCD(_softTWI, _i2cAddress, ((_data << 4) & 0xF0) | _mode);
}

/*****************************************************************************************************************************
*
*  Interprets and print incoming data to LCD which used HD44780 controller
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - DEVICE_ERROR_CONNECT on test connection error
*
*****************************************************************************************************************************/
int8_t printToPCF8574LCD(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _lcdBacklight, const uint16_t _lcdType, const char *_src, const uint8_t forceInit)
{
  uint8_t displayFunction, lastLine, currLine, currChar, i, isHexString;
  uint8_t rowOffsets[] = { 0x00, 0x40, 0x14, 0x54 };
  static uint8_t lcdInited = false;
  

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) {return DEVICE_ERROR_CONNECT; }

  switch (_lcdType) {
    case LCD_TYPE_801:
    case LCD_TYPE_1601:
      displayFunction = LCD_1LINE;
      // 0 = 1 line
      lastLine = 0;
      break;

    case LCD_TYPE_802:
    case LCD_TYPE_1602:
    case LCD_TYPE_2002:               // Tested on WinStar WH-2002A LCD
    case LCD_TYPE_2402:
    case LCD_TYPE_4002:
      displayFunction = LCD_2LINE;
      // 0, 1 = 2 line
      lastLine = 1;
      break;
    case LCD_TYPE_1604:
    case LCD_TYPE_2004:               // Tested on generic 2004A LCD
    case LCD_TYPE_4004:
      displayFunction = LCD_2LINE;
      // 0, 1, 2, 3 = 4 line
      lastLine = 3;
      break;
    default:
      return false;
  }
      displayFunction = LCD_2LINE;

  _lcdBacklight = _lcdBacklight ? _BV(LCD_BL) : 0;

  // just tooggle backlight and go back if no data given 
  writeByteToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, _lcdBacklight);
  if (! *_src) {return RESULT_IS_OK; }

  if (forceInit || !lcdInited) {
     lcdInited = true;
     // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
     // according to datasheet, we need at least 40ms after power rises above 2.7V
     // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
     delay(50); 

     // Now we pull both RS and R/W low to begin commands
     // this is according to the hitachi HD44780 datasheet
     // figure 24, pg 46
     
     // we start in 8bit mode, try to set 4 bit mode
     write4bitsToLCD(_softTWI, _i2cAddress, (0x03 << 4) | _lcdBacklight);
     delayMicroseconds(4500); // wait min 4.1ms
      
     // second try
     write4bitsToLCD(_softTWI, _i2cAddress, (0x03 << 4) | _lcdBacklight);
     delayMicroseconds(4500); // wait min 4.1ms
      
     // third go!
     write4bitsToLCD(_softTWI, _i2cAddress, (0x03 << 4) | _lcdBacklight);
     delayMicroseconds(150);
      
     // finally, set to 4-bit interface
     write4bitsToLCD(_softTWI, _i2cAddress, (0x02 << 4) | _lcdBacklight);
     
     // set # lines, font size, etc.
     sendToLCD(_softTWI, _i2cAddress, (LCD_FUNCTIONSET | displayFunction), 0 | _lcdBacklight);
     sendToLCD(_softTWI, _i2cAddress, (LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF), 0 | _lcdBacklight);
  
  }
  // Always begin from 0,0
  currLine = 0; 
   // HEX strings must be processeed specially
  isHexString = false;

  // TODO: use hstoba() to preconvert hex-string
  if (haveHexPrefix(_src)) {
     // Skip "0x"
     _src += 2;
     isHexString = true;
  }
  // Walk over all string
  while(*_src) {
    //
    // HEX processing
    // 
    if (isHexString) {
       // restore full byte (ASCII char) from HEX nibbles
       // Make first four bits of byte to push from HEX.
       currChar = htod(*_src); _src++;
       // Move first nibble to high
       currChar <<= 4;
       // Check for second nibble existience
       if (*_src) {
          // Add its to byte if HEX not '\0'
          currChar |= htod(*_src);
       }
     } else {
       //
       //  ASCII processing
       //
       currChar = *_src;
       // Escape sign detected
       if ('\\' == currChar) {
         // See for next char
         switch (*(_src+1)) {
           // Self-escaping, just skip next char and make no transformation for current char
           case '\\':
             break;
           // \t - horizontal tabulation
           case 't':
             currChar = LCD_CMD_HT;
             break;
           // \n - new line
           case 'n':
             currChar = LCD_CMD_LF;
             break;
           // form feed
           case 'f':
             currChar = LCD_CMD_RETURNHOME;
             break;
           // \xHH - HEX byte HH, for example \x0C => LCD_CMD_CURSOROFF
           case 'x':
             // take next two char (+2 & +3) and move pointer 
             if (*(_src+2) && *(_src+3)) { 
                currChar = (htod((char) tolower(*(_src+2))) << 4) + htod((char) tolower(*(_src+3))); 
                _src += 2; 
             }
             break;
         } // switch (*(_src+1))
         _src++;
      } // if ('\\' == currChar)
    } // if (isHexString) ... else 

//    Serial.print("currChar: 0x"); Serial.println(currChar, HEX);
#ifndef LCD_MELT_CODEPAGE_COMPABILITY
    if (currChar > 0x7F && currChar < 0x9F) {
       // Jump to position 
       //Serial.print("Jump to:"); Serial.println(int (currChar - 0x80));
       sendToLCD(_softTWI, _i2cAddress, LCD_SETDDRAMADDR | ((currChar - 0x80) + rowOffsets[currLine]), 0 | _lcdBacklight);
      } else {
#endif
       // Analyze character
       switch (currChar) {
         // clear display, set cursor position to zero
         case LCD_CMD_RETURNHOME:
         case LCD_CMD_CLEARDISPLAY:
           sendToLCD(_softTWI, _i2cAddress, currChar, 0 | _lcdBacklight);
           delayMicroseconds(1700);  
           break;

         // Blink with backlight
         case LCD_CMD_BACKLIGHT_BLINK:
           for (i = 0; i < LCD_BLINK_TIMES; i++) {
              // _lcdBacklight is not false/true, is 0x00 / 0x08
              _lcdBacklight = _lcdBacklight ? 0x00 : _BV(LCD_BL);
              writeByteToI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, _lcdBacklight);
              delay (LCD_BLINK_DUTY_CYCLE);
            } 
            break;
          
         // HD44780 control commands
         case LCD_CMD_DISPLAYOFF:
         case LCD_CMD_ENTRYMODE_RIGHTTOLEFT:
         case LCD_CMD_ENTRYMODE_RIGHTTOLEFT_SCREENSHIFT:
         case LCD_CMD_ENTRYMODE_LEFTTORIGHT:
         case LCD_CMD_ENTRYMODE_LEFTTORIGHT_SCREENSHIFT:
         case LCD_CMD_BLANKSCREEN:
         case LCD_CMD_CURSOROFF:
         case LCD_CMD_UNDERLINECURSORON:
         case LCD_CMD_BLINKINGBLOCKCURSORON:
         case LCD_CMD_CURSORMOVELEFT:
         case LCD_CMD_CURSORMOVERIGHT:
         case LCD_CMD_SCREENSHIFTLEFT:
         case LCD_CMD_SCREENSHIFTRIGHT:
           sendToLCD(_softTWI, _i2cAddress, currChar, 0 | _lcdBacklight);
           delayMicroseconds(42); 
           break;

         // Print 'Tab'
         case LCD_CMD_HT:
           // Space is 0x20 ASCII
           for (i = 0; i < LCD_TAB_SIZE; i++) { sendToLCD(_softTWI, _i2cAddress, 0x20, 0 | _BV(LCD_RS) | _lcdBacklight); }
           break;
       
         // Go to new line
         case LCD_CMD_LF:
           if (lastLine > currLine) { currLine++; }
           sendToLCD(_softTWI, _i2cAddress, (LCD_SETDDRAMADDR | rowOffsets[currLine]), 0 | _lcdBacklight);
           break;

         // Otherwise - print the char
         default:
           sendToLCD(_softTWI, _i2cAddress, currChar , 0 | _BV(LCD_RS) | _lcdBacklight);    
       } //switch (currChar) {
#ifndef LCD_MELT_CODEPAGE_COMPABILITY
    } // if (currChar > 0x7F && currChar < 0x9F) .. else 
#endif
    // move pointer to next char
    _src++;
  } // while(*_src)
  gatherSystemMetrics(); // Measure memory consumption
  return RESULT_IS_OK;
}

