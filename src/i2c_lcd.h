#ifndef _ZABBUINO_I2C_LCD_H_
#define _ZABBUINO_I2C_LCD_H_

#include "i2c_bus.h"


// Some pin mappings not used at presently
/* LCD functional pin             ===>                   PCF8574 port (bit # in byte which send to I2C expander) */

#define LCD_RS                                                  0x00      // P0 (pin 4) on PCF8574 - expander pin #4  (used)
#define LCD_RW                                                  0x01      // P1 (pin 5) on PCF8574 - expander pin #5
#define LCD_E                                                   0x02      // P2 (pin 6) on PCF8574 - expander pin #6  (used)
#define LCD_BL                                                  0x03      // P3 (pin 7) on PCF8574 - expander pin #7  (used)

#define LCD_D4                                                  0x04      // P4 (pin 9) on PCF8574  - expander pin #11
#define LCD_D5                                                  0x05      // P5 (pin 10) on PCF8574 - expander pin #12
#define LCD_D6                                                  0x06      // P6 (pin 11) on PCF8574 - expander pin #13
#define LCD_D7                                                  0x07      // P7 (pin 12) on PCF8574 - expander pin #14

// LCD types.  1602 => 16 chars * 2 rows
#define LCD_TYPE_801                                            801
#define LCD_TYPE_1601                                           1601
                            
#define LCD_TYPE_802                                            802 
#define LCD_TYPE_1202                                           1202 
#define LCD_TYPE_1602                                           1602 
#define LCD_TYPE_2002                                           2002
#define LCD_TYPE_2402                                           2402
#define LCD_TYPE_4002                                           4002

#define LCD_TYPE_1604                                           1604
#define LCD_TYPE_2004                                           2004
#define LCD_TYPE_4004                                           4004

// LCD control codes
#define LCD_TAB_SIZE                                            0x04   // 4 space
#define LCD_BLINK_DUTY_CYCLE                                    250    // 250 ms
#define LCD_BLINK_TIMES                                         0x04   // in cycle of 4 times - 2 off state & 2 on state. Need use even numbers for save previous backlight state on cycle finish

// HD44780-compatible commands
#define LCD_CMD_BACKLIGHT_BLINK                                 0x03   // ASCII 03 - backlight blink
#define LCD_CMD_HT                                              0x09   // ASCII 09 - horizontal tabulation
#define LCD_CMD_LF                                              0x0A   // ASCII 10 - line feed

#define LCD_CMD_DISPLAYOFF                                      0x00
#define LCD_CMD_CLEARDISPLAY                                    0x01
#define LCD_CMD_RETURNHOME                                      0x02
#define LCD_CMD_ENTRYMODE_RIGHTTOLEFT                           0x04
#define LCD_CMD_ENTRYMODE_RIGHTTOLEFT_SCREENSHIFT               0x05
#define LCD_CMD_ENTRYMODE_LEFTTORIGHT                           0x06
#define LCD_CMD_ENTRYMODE_LEFTTORIGHT_SCREENSHIFT               0x07
#define LCD_CMD_BLANKSCREEN                                     0x08
#define LCD_CMD_CURSOROFF                                       0x0C
#define LCD_CMD_UNDERLINECURSORON                               0x0E
#define LCD_CMD_BLINKINGBLOCKCURSORON                           0x0F
#define LCD_CMD_CURSORMOVELEFT                                  0x10
#define LCD_CMD_CURSORMOVERIGHT                                 0x14
#define LCD_CMD_SCREENSHIFTLEFT                                 0x18
#define LCD_CMD_SCREENSHIFTRIGHT                                0x1E


#define LCD_DISPLAYCONTROL                                      0x08
#define LCD_CURSORSHIFT                                         0x10
#define LCD_FUNCTIONSET                                         0x20
#define LCD_SETDDRAMADDR                                        0x80

// flags for display on/off control
#define LCD_DISPLAYON                                           0x04
#define LCD_DISPLAYOFF                                          0x00
#define LCD_CURSORON                                            0x02
#define LCD_CURSOROFF                                           0x00
#define LCD_BLINKON                                             0x01
#define LCD_BLINKOFF                                            0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE                                         0x08
#define LCD_CURSORMOVE                                          0x00
#define LCD_MOVERIGHT                                           0x04
#define LCD_MOVELEFT                                            0x00

// flags for function set
#define LCD_8BITMODE                                            0x10
#define LCD_4BITMODE                                            0x00
#define LCD_2LINE                                               0x08
#define LCD_1LINE                                               0x00


/*****************************************************************************************************************************
*
*  Interprets and print incoming data to LCD which used HD44780 controller
*
*   Returns: 
*     - RESULT_IS_OK on success
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/
int8_t printToPCF8574LCD(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _lcdBacklight, const uint16_t _lcdType, const char *_src);

#endif // #ifndef _ZABBUINO_I2C_LCD_H_