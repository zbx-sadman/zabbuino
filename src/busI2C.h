#ifndef ZabbuinoBUSI2C_h
#define ZabbuinoBUSI2C_h

#include <Arduino.h>
// Wire lib for I2C sensors
#include <Wire.h>
#include "defaults.h"
#include "../zabbuino.h"
#include "service.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                      COMMON I2C SECTION
*/

#define WireToU8(_source)  ((uint8_t) _source[0])
#define WireToS8(_source)  ((int8_t) _source[0])

#define WireToU16(_source)  ((uint16_t) ( ((uint16_t) _source[0] << 8)| _source[1]))
#define WireToS16(_source)  ((int16_t) ( ((uint16_t) _source[0] << 8)| _source[1]))

#define WireToU16LE(_source)  ((uint16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))
#define WireToS16LE(_source)  ((int16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))

#define WireToU24(_source)  ((uint32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))
#define WireToS24(_source)  ((int32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))

int8_t scanI2C(EthernetClient *_ethClient);

uint8_t writeByteToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _data);
uint8_t writeBytesToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t* _data, uint8_t _len);
uint8_t readBytesFromi2C(const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t _buff[], const uint8_t _len);
uint8_t inline isI2CDeviceReady(uint8_t _i2cAddress);

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           SHT2x SECTION
*/

#define SHT2X_I2C_ADDRESS                                       0x40
#define SHT2X_CMD_GETTEMP_HOLD                                  0xE3
#define SHT2X_CMD_GETHUMD_HOLD                                  0xE5

uint16_t getRawDataFromSHT2X(const uint8_t _i2cAddress, const uint8_t _command);
int8_t getSHT2XMetric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, const uint8_t _metric, char* _dst);

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     BMP SECTION
*/

#define BMP180_I2C_ADDRESS                                      0x77  // I2C address of BMP085/180 sensor

#define BMP280_I2C_ADDRESS_1                                    0x76  // I2C address of BMP280/BME280 sensor
#define BMP280_I2C_ADDRESS_2                                    0x77


#define BMP_REGISTER_CHIPID                                     0xD0

#define BMP_REGISTER_RESET                                      0xE0 // for all BMP or not?
#define BMP_REGISTER_CONTROL                                    0xF4 


#define BMP085_CHIPID                                           0x55
#define BMP180_CHIPID                                           0x55
#define BMP280_CHIPID_1                                         0x56 // Only for engeneering samples (BMP180 datasheet, page 24)
#define BMP280_CHIPID_2                                         0x57 // Only for engeneering samples
#define BMP280_CHIPID_3                                         0x58 // Mass production ID

#define BME280_CHIPID                                           0x60 // 

#define BMP180_ULTRALOWPOWER                                    0x00
#define BMP180_STANDARD                                         0x01
#define BMP180_HIGHRES                                          0x02
#define BMP180_ULTRAHIGHRES                                     0x03

#define BMP180_REGISTER_CAL_AC1                                 0xAA  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC2                                 0xAC  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC3                                 0xAE  // R   Calibration data (16 bits)    
#define BMP180_REGISTER_CAL_AC4                                 0xB0  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC5                                 0xB2  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC6                                 0xB4  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_B1                                  0xB6  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_B2                                  0xB8  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_MB                                  0xBA  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_MC                                  0xBC  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_MD                                  0xBE  // R   Calibration data (16 bits)

#define BMP180_REGISTER_TEMPDATA                                0xF6
#define BMP180_REGISTER_PRESSUREDATA                            0xF6
#define BMP180_CMD_READTEMP                                     0x2E
#define BMP180_CMD_READPRESSURE                                 0x34

#define BMP180_READY_MASK                                       0x20  // Sco (register F4h <5>): Start of conversion. 
                                                                      // The value of this bit stays “1” during conversion and is reset to “0” after conversion is 
                                                                      // complete (data registers are filled). 

#define BMP280_REGISTER_DIG_T1                                  0x88
#define BMP280_REGISTER_DIG_T2                                  0x8A
#define BMP280_REGISTER_DIG_T3                                  0x8C

#define BMP280_REGISTER_DIG_P1                                  0x8E
#define BMP280_REGISTER_DIG_P2                                  0x90
#define BMP280_REGISTER_DIG_P3                                  0x92
#define BMP280_REGISTER_DIG_P4                                  0x94
#define BMP280_REGISTER_DIG_P5                                  0x96
#define BMP280_REGISTER_DIG_P6                                  0x98
#define BMP280_REGISTER_DIG_P7                                  0x9A
#define BMP280_REGISTER_DIG_P8                                  0x9C
#define BMP280_REGISTER_DIG_P9                                  0x9E

#define BMP280_REGISTER_CAL26                                   0xE1  // R calibration stored in 0xE1-0xF0

#define BMP280_REGISTER_STATUS                                  0xF3
#define BMP280_REGISTER_CONTROL                                 0xF4
#define BMP280_REGISTER_CONFIG                                  0xF5
#define BMP280_REGISTER_PRESSUREDATA                            0xF7
#define BMP280_REGISTER_TEMPDATA                                0xFA

#define BMP280_OVERSAMP_SKIPPED          			0x00
#define BMP280_OVERSAMP_1X               			0x01
#define BMP280_OVERSAMP_2X               			0x02
#define BMP280_OVERSAMP_4X               			0x03
#define BMP280_OVERSAMP_8X               			0x04
#define BMP280_OVERSAMP_16X              			0x05

#define BMP280_ULTRALOWPOWER            			0x00
#define BMP280_LOWPOWER	                 			0x01
#define BMP280_STANDARD      	                		0x02
#define BMP280_HIGHRES          		        	0x03
#define BMP280_ULTRAHIGHRES    			                0x04

#define BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE          	BMP280_OVERSAMP_1X
#define BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE       	BMP280_OVERSAMP_1X

#define BMP280_LOWPOWER_OVERSAMP_PRESSURE	         	BMP280_OVERSAMP_2X
#define BMP280_LOWPOWER_OVERSAMP_TEMPERATURE	         	BMP280_OVERSAMP_1X

#define BMP280_STANDARD_OVERSAMP_PRESSURE     	                BMP280_OVERSAMP_4X
#define BMP280_STANDARD_OVERSAMP_TEMPERATURE  	                BMP280_OVERSAMP_1X

#define BMP280_HIGHRES_OVERSAMP_PRESSURE         	        BMP280_OVERSAMP_8X
#define BMP280_HIGHRES_OVERSAMP_TEMPERATURE      	        BMP280_OVERSAMP_1X

#define BMP280_ULTRAHIGHRES_OVERSAMP_PRESSURE       	        BMP280_OVERSAMP_16X
#define BMP280_ULTRAHIGHRES_OVERSAMP_TEMPERATURE    	        BMP280_OVERSAMP_2X

#define BMP280_SLEEP_MODE                    			0x00
#define BMP280_FORCED_MODE                   			0x01
#define BMP280_NORMAL_MODE                   			0x03

#define BMP280_FILTER_COEFF_OFF                                 0x00
#define BMP280_FILTER_COEFF_2                                   0x01
#define BMP280_FILTER_COEFF_4                                   0x02
#define BMP280_FILTER_COEFF_8                                   0x03
#define BMP280_FILTER_COEFF_16                                  0x04

#define BMP280_STANDBY_TIME_1_MS                                0x00
#define BMP280_STANDBY_TIME_63_MS                               0x01
#define BMP280_STANDBY_TIME_125_MS                              0x02
#define BMP280_STANDBY_TIME_250_MS                              0x03
#define BMP280_STANDBY_TIME_500_MS                              0x04
#define BMP280_STANDBY_TIME_1000_MS                             0x05
#define BMP280_STANDBY_TIME_2000_MS                             0x06
#define BMP280_STANDBY_TIME_4000_MS                             0x07

#define BMP280_READY_MASK                                       0x09 // Byte 0 + Byte 3 must be equial 0 if BMP280 do not busy

// BME280 additional registers and constants
#define BME280_STANDARD_OVERSAMP_HUMIDITY 	                BMP280_OVERSAMP_1X
#define BME280_REGISTER_DIG_H1                                  0xA1
#define BME280_REGISTER_DIG_H2                                  0xE1
#define BME280_REGISTER_DIG_H3                                  0xE3
#define BME280_REGISTER_DIG_H4                                  0xE4
#define BME280_REGISTER_DIG_H5                                  0xE5
#define BME280_REGISTER_DIG_H6                                  0xE7

#define BME280_REGISTER_CONTROLHUMID                            0xF2
#define BME280_REGISTER_HUMIDDATA                               0xFD

#define T_INIT_MAX						20   // 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX					37   // 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX					10   // 10/16 = 0.625 ms
#define T_SETUP_HUMIDITY_MAX					10   // 10/16 = 0.625 ms


uint8_t waitToBMPReady(const uint8_t _i2cAddress, const int16_t _registerAddress, const int16_t _mask, const uint16_t _timeout);
int8_t getBMPMetric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, const uint8_t _overSampling, const uint8_t _filterCoef, const uint8_t _metric, char* _dst);
int8_t getBMP280Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, const uint8_t _overSampling, uint8_t _filterCoef, const uint8_t _metric, char* _dst);
int8_t getBMP180Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _overSampling, const uint8_t _metric, char* _dst);

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     LCD SECTION

*/
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


void sendToLCD(const uint8_t _i2cAddress, const uint8_t _data, const uint8_t _mode);
void write4bitsToLCD(const uint8_t _i2cAddress, uint8_t _data);
void pulseEnableOnLCD(const uint8_t _i2cAddress, const uint8_t _data);
int8_t printToPCF8574LCD(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _lcdBacklight, const uint16_t _lcdType, const char *_src);

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     BH1750 SECTION
*/


#define BH1750_I2C_FIRST_ADDRESS                                0x23
#define BH1750_I2C_SECOND_ADDRESS                               0x5C
/*

  Datasheet: http://rohmfs.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1750fvi-e.pdf
*/

// No active state
#define BH1750_CMD_POWERDOWN                                    0x00
// Wating for measurment command
#define BH1750_CMD_POWERON                                      0x01
// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_CMD_RESET                                        0x07

// Start measurement at 1 lux resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGHRES                               0x10
// Start measurement at 0.5 lux resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGHRES_2                             0x11
// Start measurement at 4 lux resolution. Measurement time is approx 16ms.
#define BH1750_CONTINUOUS_LOWRES                                0x13
// Start measurement at 1 lux resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_HIGHRES                                  0x20
// Start measurement at 0.5 lux resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_HIGHRES_2                                0x21
// Start measurement at 4 lux resolution. Measurement time is approx 16ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_LOWRES                                   0x23


int8_t getBH1750Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _metric, char* _dst);

#endif