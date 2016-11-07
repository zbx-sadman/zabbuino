#ifndef ZabbuinoDEFAULTS_h
#define ZabbuinoDEFAULTS_h

#if defined(FEATURE_I2C_ENABLE) || defined(FEATURE_BMP_ENABLE) || defined(FEATURE_BH1750_ENABLE) || defined (FEATURE_PCF8574_LCD_ENABLE) || defined (FEATURE_SHT2X_ENABLE)
   #define LIBWIRE_USE
#endif

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 HEADERS SECTION
*/

#include <Arduino.h>
#include <IPAddress.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/boot.h>
// used by interrupts-related macroses
#include <wiring_private.h>


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                               ALARM SECTION 
*/


#define ERROR_NONE                 	                        0x00
#define ERROR_NET                 	                        0x01
#define ERROR_DHCP                 	                        0x02
// Turn off state LED blink (no errors found)
// State LED no blink type
#define BLINK_NOPE                 	                        0x00
// State LED blink type with DHCP problem reached (no renew lease or more)
#define BLINK_DHCP_PROBLEM         	                        150 // ~150ms on, ~850ms off
// State LED blink type with Network activity problem (no packets processed for NET_IDLE_TIMEOUT)
#define BLINK_NET_PROBLEM          	                        500 // ~500ms on, ~500ms off


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            NETWORK MODULE SECTION 

       Note, that changing MAC or IP-address separately may cause "strange" network errors until the moment when the router delete old ARP-records from the cache.

*/


// How often do ENC28J60 module reinit for more stable network
#define NET_ENC28J60_REINIT_PERIOD  	                        10000UL  // 10 sec

// These macros used only to debug process on ENC28J60-powered systems 
// ENC28J60 bank registers
#define NET_ENC28J60_EIR                                        0x1C
#define NET_ENC28J60_ESTAT                                      0x1D
// ENC28J60 EIE Register Bit Definitions
#define NET_ENC28J60_EIE_INTIE                                  0x80
#define NET_ENC28J60_EIE_PKTIE                                  0x40
#define NET_ENC28J60_EIE_DMAIE                                  0x20
#define NET_ENC28J60_EIE_LINKIE                                 0x10
#define NET_ENC28J60_EIE_TXIE                                   0x08
#define NET_ENC28J60_EIE_WOLIE                                  0x04
#define NET_ENC28J60_EIE_TXERIE                                 0x02
#define NET_ENC28J60_EIE_RXERIE                                 0x01
// ENC28J60 EIR Register Bit Definitions
#define NET_ENC28J60_EIR_PKTIF                                  0x40
#define NET_ENC28J60_EIR_DMAIF                                  0x20
#define NET_ENC28J60_EIR_LINKIF                                 0x10
#define NET_ENC28J60_EIR_TXIF                                   0x08
#define NET_ENC28J60_EIR_WOLIF                                  0x04
#define NET_ENC28J60_EIR_TXERIF                                 0x02
#define NET_ENC28J60_EIR_RXERIF                                 0x01
// ENC28J60 ESTAT Register Bit Definitions
#define NET_ENC28J60_ESTAT_INT                                  0x80
#define NET_ENC28J60_ESTAT_LATECOL                              0x10
#define NET_ENC28J60_ESTAT_RXBUSY                               0x04
#define NET_ENC28J60_ESTAT_TXABRT                               0x02
#define NET_ENC28J60_ESTAT_CLKRDY                               0x01


// Network activity timeout (for which no packets processed or no DHCP lease renews finished with success)
#define NET_IDLE_TIMEOUT            	                        60000UL  // 60 sec

// How long active client can transmit packets
#ifdef FEATURE_DEBUG_TO_SERIAL
  // on debug serial's output can make network processing slow
  #define NET_ACTIVE_CLIENT_CONNECTION_TIMEOUT                    5000UL  // 5 sec
#else
  #define NET_ACTIVE_CLIENT_CONNECTION_TIMEOUT                    1000UL  // 1 sec
#endif

// How often do renew DHCP lease
#define NET_DHCP_RENEW_PERIOD       	                        30000UL  // 30 sec
// How often do renew DHCP lease
#define NET_STABILIZATION_DELAY     	                        100L     // 0.1 sec


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         SYSTEM CONFIGURATION SECTION 
*/

#define HOLD_TIME_TO_FACTORY_RESET  	                        5000L // 5 seconds

// How many secs device may be stay in infinitibe loop before reboot
// Also you can use:
// WDTO_1S
// WDTO_2S
// WDTO_4S - not for all controllers (please reference to avr/wdt.h)
// WDTO_8S - not for all controllers (please reference to avr/wdt.h)
#define WTD_TIMEOUT                   	                        WDTO_8S 

#define SYS_METRIC_RENEW_PERIOD        	                        1000L // 1 sec

// Number of expected arguments of the command
#define ARGS_MAX                    	                        6
// Size of buffer's argument part. All separators and delimiters must be taken into account. See note to BUFFER_SIZE macro too
#define ARGS_PART_SIZE         	                                100
// Size of buffer's command part
#define CMD_PART_SIZE          	                                25

// ***NOTE****    
// Total buffer size cannot be so small, because many subroutines use its too.
// getMegatecUPSMetric() can write to its up to MEGATEC_MAX_ANSWER_LENGTH bytes, for example
// The total size of the buffer. 
#define BUFFER_SIZE                   	                        CMD_PART_SIZE + ARGS_PART_SIZE

#define SYS_MCU_ID_LEN                                          20

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          AGENT CONFIGURATION SECTION 
*/

// How much bytes will be allocated to hostname store
// sizeof() is not used here to get constant memory allocation due set.hostname() can operate longer strings
#define ZBX_AGENT_HOSTNAME_MAXLEN   	                        32  // MCU ID as hostname take 20 chars

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          I/O PORTS/PINS PRE-CONFIGURATION SECTION 
*/

// see below: const uint8_t port_protect[PORTS_NUM] = {...}
#if defined (ARDUINO_AVR_DUEMILANOVE) || defined (ARDUINO_AVR_MINI) || defined (ARDUINO_AVR_NANO) || defined (ARDUINO_AVR_NG) || defined (ARDUINO_AVR_PRO) || defined (ARDUINO_AVR_ETHERNET)
#define PORTS_NUM                                               0x05
#elif defined(ARDUINO_AVR_LEONARDO) || defined (ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_ROBOT_CONTROL) || defined(ARDUINO_AVR_ROBOT_MOTOR) || defined (ARDUINO_AVR_YUN)
#define PORTS_NUM                                               0x07
#elif defined (ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
#define PORTS_NUM                                               0x0D
#else // Unknow boards equal to "Arduino Duemilanove"
#define PORTS_NUM 						0x05
#define ARDUINO_AVR_DUEMILANOVE
#endif

const uint8_t port_protect[PORTS_NUM] = {
/*

  All bits equal '0' cause setting corresponding pin to non-protection mode
  
  All bits equal '1' cause setting corresponding pin to protection mode (it's will be not affected by portWrite[], digitalWrite[] and other commands)
  
*/
#if defined (ARDUINO_AVR_DUEMILANOVE) || defined (ARDUINO_AVR_MINI) || defined (ARDUINO_AVR_NANO) || defined (ARDUINO_AVR_NG) || defined (ARDUINO_AVR_PRO) || defined (ARDUINO_AVR_ETHERNET)
  B00000000, // not a port
  B00000000, // not a port
  // Pins D10, D11, D12, D13 is protected by setting 2, 3, 4, 5 bits, due its used to SPI (ethernet shield)
  // Pin D9 is used by Timer1 and can't be used for PWM (analogWrite) when system metric was gathered on Timer1 interrupt
  // Bits 6, 7 have not correspondented pins in Arduino Mini Pro / Freeduino 2009
  B11111110, /*     PORTB        
D13 -^    ^- D8    <- pins   */
  B00000000, /*     PORTC 
   ^-A7   ^-A0   <- pins    */
  // Pins D0, D1 is protected by settings 0, 1 bits, due its used to RX/TX lines of UART and make it possible to transmit data to Serial Monitor  
  B00000000  /*     PORTD 
   ^-D7   ^-D0   <- pins    */
#elif defined(ARDUINO_AVR_LEONARDO) || defined (ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_ROBOT_CONTROL) || defined(ARDUINO_AVR_ROBOT_MOTOR) || defined (ARDUINO_AVR_YUN)
  // check ports settings for your platform
  B00000000, // not a port
  B00000000, // not a port
  B00000000, // PORTB
  B00000000, // PORTC
  B00000000, // PORTD
  B00000000, // PORTE
  B00000000  // PORTF
#elif defined (ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
  // check ports settings for your platform
  B00000000, // not a port
  B00000000, // PORTA
  B00000000, // PORTB
  B00000000, // PORTC
  B00000000, // PORTD
  B00000000, // PORTE
  B00000000, // PORTF
  B00000000, // PORTG
  B00000000, // PORTH
  B00000000, // not a port
  B00000000, // PORTJ
  B00000000, // PORTK
  B00000000  // PORTL
#endif
};



const uint8_t port_mode[PORTS_NUM] = {
/*
 
  All bits equal '0' cause setting corresponding pin to INPUT mode
  
  All bits equal '1' cause setting corresponding pin to OUTPUT mode
  
*/
 
#if defined (ARDUINO_AVR_DUEMILANOVE) || defined (ARDUINO_AVR_MINI) || defined (ARDUINO_AVR_NANO) || defined (ARDUINO_AVR_NG) || defined (ARDUINO_AVR_PRO) || defined (ARDUINO_AVR_ETHERNET)
  B00000000, // not a port
  B00000000, // not a port
  // Bits 6, 7 have not correspondented pins in Arduino Mini Pro / Freeduino 2009
  B00111110, /*     PORTB        
D13 -^    ^- D8    <- pins   */
  B11111100, /*     PORTC 
   ^-A7   ^-A0   <- pins    */
  B11111111  /*     PORTD 
   ^-D7   ^-D0   <- pins    */
#elif defined(ARDUINO_AVR_LEONARDO) || defined (ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_ROBOT_CONTROL) || defined(ARDUINO_AVR_ROBOT_MOTOR) || defined (ARDUINO_AVR_YUN)
  // check ports settings for your platform
  B00000000, // not a port
  B00000000, // not a port
  B11111111, // PORTB
  B11111111, // PORTC
  B11111111, // PORTD
  B11111111, // PORTE
  B11111111  // PORTF
#elif defined (ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
  // check ports settings for your platform
  B00000000, // not a port
  B11111111, // PORTA
  B11111111, // PORTB
  B11111111, // PORTC
  B11111111, // PORTD
  B11111111, // PORTE
  B11111111, // PORTF
  B11111111, // PORTG
  B11111111, // PORTH
  B00000000, // not a port
  B11111111, // PORTJ
  B11111111, // PORTK
  B11111111  // PORTL
#endif
};


const uint8_t port_pullup[PORTS_NUM] = {
/*
   All bits equal '0' cause do not pull-up corresponding pin

   All bits equal '1' cause pull-up corresponding pin

*/

#if defined (ARDUINO_AVR_DUEMILANOVE) || defined (ARDUINO_AVR_MINI) || defined (ARDUINO_AVR_NANO) || defined (ARDUINO_AVR_NG) || defined (ARDUINO_AVR_PRO) || defined (ARDUINO_AVR_ETHERNET)
  B00000000, // not a port
  B00000000, // not a port
  // Bits 6, 7 have not correspondented pins in Arduino Mini Pro / Freeduino 2009
  B00111101, /*     PORTB        
D13 -^    ^- D8    <- pins   */
  B00000000, /*     PORTC 
   ^-A7   ^-A0   <- pins    */
  B00000011  /*     PORTD 
   ^-D7   ^-D0   <- pins    */
#elif defined(ARDUINO_AVR_LEONARDO) || defined (ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_ROBOT_CONTROL) || defined(ARDUINO_AVR_ROBOT_MOTOR) || defined (ARDUINO_AVR_YUN)
  // check ports settings for your platform
  B00000000, // not a port
  B00000000, // not a port
  B00000000, // PORTB
  B00000000, // PORTC
  B00000000, // PORTD
  B00000000, // PORTE
  B00000000  // PORTF
#elif defined (ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
  // check ports settings for your platform
  B00000000, // not a port
  B00000000, // PORTA
  B00000000, // PORTB
  B00000000, // PORTC
  B00000000, // PORTD
  B00000000, // PORTE
  B00000000, // PORTF
  B00000000, // PORTG
  B00000000, // PORTH
  B00000000, // not a port
  B00000000, // PORTJ
  B00000000, // PORTK
  B00000000  // PORTL
#endif
};


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            COMMAND NAMES SECTION 
*/
// Increase this if add new command 
//#define CMD_MAX                                                 0x3F

// Add command macro with new sequental number
#define CMD_ZBX_NOPE                                            0x00
#define CMD_ZBX_AGENT_PING                                      0x01
#define CMD_ZBX_AGENT_HOSTNAME                                  0x02
#define CMD_ZBX_AGENT_VERSION                                   0x03
#define CMD_SYS_UPTIME                                          0x04
                                                                
#define CMD_ARDUINO_ANALOGWRITE                                 0x05
#define CMD_ARDUINO_ANALOGREAD                                  0x06
#define CMD_ARDUINO_ANALOGREFERENCE                             0x07
#define CMD_ARDUINO_DELAY                                       0x08
#define CMD_ARDUINO_DIGITALWRITE                                0x09
#define CMD_ARDUINO_DIGITALREAD                                 0x0A
#define CMD_ARDUINO_TONE                                        0x0B
#define CMD_ARDUINO_NOTONE                                      0x0C
#define CMD_ARDUINO_RANDOMSEED                                  0x0D
#define CMD_ARDUINO_RANDOM                                      0x0E

#define CMD_SET_HOSTNAME                                        0x0F
#define CMD_SET_PASSWORD                                        0x10
#define CMD_SET_SYSPROTECT                                      0x11
#define CMD_SET_NETWORK                                         0x12

#define CMD_SYS_PORTWRITE                                       0x13
#define CMD_SYS_SHIFTOUT                                        0x14
#define CMD_SYS_REBOOT                                          0x15

#define CMD_SYS_MCU_NAME                                        0x16
#define CMD_SYS_NET_MODULE                                      0x17

#define CMD_SYS_CMD_COUNT                                       0x18
#define CMD_SYS_CMD_TIMEMAX                                     0x19
#define CMD_SYS_CMD_TIMEMAX_N                                   0x1A

#define CMD_SYS_RAM_FREE                                        0x1B
#define CMD_SYS_RAM_FREEMIN                                     0x1C

#define CMD_SYS_VCC                                             0x1D
#define CMD_SYS_VCCMIN                                          0x1E
#define CMD_SYS_VCCMAX                                          0x1F

#define CMD_EXTINT_COUNT                                        0x20
#define CMD_INCENC_VALUE                                        0x21

#define CMD_OW_SCAN                                             0x22

#define CMD_I2C_SCAN                                            0x23
#define CMD_I2C_WRITE                                           0x24
#define CMD_I2C_READ                                            0x25
#define CMD_I2C_BITWRITE                                        0x26
#define CMD_I2C_BITREAD                                         0x27

#define CMD_DS18X20_TEMPERATURE                                 0x28

#define CMD_DHT_HUMIDITY                                        0x29
#define CMD_DHT_TEMPERATURE                                     0x2A

#define CMD_BMP_PRESSURE                                        0x2B 
#define CMD_BMP_TEMPERATURE                                     0x2C
#define CMD_BME_HUMIDITY                                        0x2D

#define CMD_BH1750_LIGHT                                        0x2E

#define CMD_MAX7219_WRITE                                       0x2F

#define CMD_PCF8574_LCDPRINT                                    0x30

#define CMD_SHT2X_HUMIDITY                                      0x31
#define CMD_SHT2X_TEMPERATURE                                   0x32

#define CMD_ACS7XX_ZC                                           0x33
#define CMD_ACS7XX_AC                                           0x34
#define CMD_ACS7XX_DC                                           0x35

#define CMD_ULTRASONIC_DISTANCE                                 0x36

#define CMD_IR_SEND                                             0x37
#define CMD_IR_SENDRAW                                          0x38

#define CMD_WS2812_SENDRAW                                      0x39

#define CMD_SYS_MCU_ID                                          0x3A
#define CMD_SYS_MCU_SIGN                                        0x3B

#define CMD_PZEM004_CURRENT                                     0x3C 
#define CMD_PZEM004_VOLTAGE                                     0x3D 
#define CMD_PZEM004_POWER                                       0x3E  
#define CMD_PZEM004_ENERGY                                      0x3F
 
#define CMD_UPS_APCSMART                                        0x40
#define CMD_UPS_MEGATEC                                         0x41
#define CMD_SYSTEM_RUN                                          0x42


// add new command as "const char command_<COMMAND_MACRO> PROGMEM". Only 'const' push string to PROGMEM. Tanx, Arduino.
// command_* values must be in lower case
const char command_CMD_ZBX_NOPE[]                               PROGMEM = "";
const char command_CMD_ZBX_AGENT_PING[]                         PROGMEM = "agent.ping";
const char command_CMD_ZBX_AGENT_HOSTNAME[]                     PROGMEM = "agent.hostname";
const char command_CMD_ZBX_AGENT_VERSION[]                      PROGMEM = "agent.version";
const char command_CMD_SYS_UPTIME[]                             PROGMEM = "sys.uptime";

const char command_CMD_ARDUINO_ANALOGWRITE[]                    PROGMEM = "analogwrite";         
const char command_CMD_ARDUINO_ANALOGREAD[]                     PROGMEM = "analogread";
const char command_CMD_ARDUINO_ANALOGREFERENCE[]                PROGMEM = "analogreference";
const char command_CMD_ARDUINO_DELAY[]                          PROGMEM = "delay";
const char command_CMD_ARDUINO_DIGITALWRITE[]                   PROGMEM = "digitalwrite";
const char command_CMD_ARDUINO_DIGITALREAD[]                    PROGMEM = "digitalread"; 

const char command_CMD_ARDUINO_TONE[]                           PROGMEM = "tone";
const char command_CMD_ARDUINO_NOTONE[]                         PROGMEM = "notone";

const char command_CMD_ARDUINO_RANDOMSEED[]                     PROGMEM = "randomseed";
const char command_CMD_ARDUINO_RANDOM[]                         PROGMEM = "random";

const char command_CMD_SET_HOSTNAME[]                           PROGMEM = "set.hostname";
const char command_CMD_SET_PASSWORD[]                           PROGMEM = "set.password";
const char command_CMD_SET_SYSPROTECT[]                         PROGMEM = "set.sysprotect";
const char command_CMD_SET_NETWORK[]                            PROGMEM = "set.network";

const char command_CMD_SYS_PORTWRITE[]                          PROGMEM = "portwrite";

const char command_CMD_SYS_SHIFTOUT[]                           PROGMEM = "shiftout";

const char command_CMD_SYS_REBOOT[]                             PROGMEM = "reboot";              

const char command_CMD_SYS_MCU_NAME[]                           PROGMEM = "sys.mcu.name";
const char command_CMD_SYS_MCU_ID[]                             PROGMEM = "sys.mcu.id";
const char command_CMD_SYS_MCU_SIGN[]                           PROGMEM = "sys.mcu.sign";
const char command_CMD_SYS_NET_MODULE[]                         PROGMEM = "sys.net.module";

const char command_CMD_SYS_CMD_COUNT[]                          PROGMEM = "sys.cmd.count";
const char command_CMD_SYS_CMD_TIMEMAX[]                        PROGMEM = "sys.cmd.timemax";
const char command_CMD_SYS_CMD_TIMEMAX_N[]                      PROGMEM = "sys.cmd.timemax.n";

const char command_CMD_SYS_RAM_FREE[]                           PROGMEM = "sys.ram.free";
const char command_CMD_SYS_RAM_FREEMIN[]                        PROGMEM = "sys.ram.freemin";

const char command_CMD_SYS_VCC[]                                PROGMEM = "sys.vcc";
const char command_CMD_SYS_VCCMIN[]                             PROGMEM = "sys.vccmin";
const char command_CMD_SYS_VCCMAX[]                             PROGMEM = "sys.vccmax";

const char command_CMD_EXTINT_COUNT[]                           PROGMEM = "extint.count";
const char command_CMD_INCENC_VALUE[]                           PROGMEM = "incenc.value";

const char command_CMD_OW_SCAN[]                                PROGMEM = "ow.scan";

const char command_CMD_I2C_SCAN[]                               PROGMEM = "i2c.scan";
const char command_CMD_I2C_WRITE[]                              PROGMEM = "i2c.write";
const char command_CMD_I2C_READ[]                               PROGMEM = "i2c.read";
const char command_CMD_I2C_BITWRITE[]                           PROGMEM = "i2c.bitwrite";
const char command_CMD_I2C_BITREAD[]                            PROGMEM = "i2c.bitread";

const char command_CMD_DS18X20_TEMPERATURE[]                    PROGMEM = "ds18x20.temperature";

const char command_CMD_DHT_HUMIDITY[]                           PROGMEM = "dht.humidity";
const char command_CMD_DHT_TEMPERATURE[]                        PROGMEM = "dht.temperature";

const char command_CMD_BMP_PRESSURE[]                           PROGMEM = "bmp.pressure";
const char command_CMD_BMP_TEMPERATURE[]                        PROGMEM = "bmp.temperature";
const char command_CMD_BME_HUMIDITY[]                           PROGMEM = "bme.humidity";

const char command_CMD_BH1750_LIGHT[]                           PROGMEM = "bh1750.light";

const char command_CMD_MAX7219_WRITE[]                          PROGMEM = "max7219.write";

const char command_CMD_PCF8574_LCDPRINT[]                       PROGMEM = "pcf8574.lcdprint";

const char command_CMD_SHT2X_HUMIDITY[]                         PROGMEM = "sht2x.humidity";
const char command_CMD_SHT2X_TEMPERATURE[]                      PROGMEM = "sht2x.temperature";

const char command_CMD_ACS7XX_ZC[]                              PROGMEM = "acs7xx.zc";
const char command_CMD_ACS7XX_AC[]                              PROGMEM = "acs7xx.ac";
const char command_CMD_ACS7XX_DC[]                              PROGMEM = "acs7xx.dc";

const char command_CMD_ULTRASONIC_DISTANCE[]                    PROGMEM = "ultrasonic.distance";
const char command_CMD_IR_SEND[]                                PROGMEM = "ir.send";
const char command_CMD_IR_SENDRAW[]                             PROGMEM = "ir.sendraw";

const char command_CMD_WS2812_SENDRAW[]                         PROGMEM = "ws2812.sendraw";

const char command_CMD_PZEM004_CURRENT[]                        PROGMEM = "pzem004.current";
const char command_CMD_PZEM004_VOLTAGE[]                        PROGMEM = "pzem004.voltage";
const char command_CMD_PZEM004_POWER[]                          PROGMEM = "pzem004.power";
const char command_CMD_PZEM004_ENERGY[]                         PROGMEM = "pzem004.energy";

const char command_CMD_UPS_APCSMART[]                           PROGMEM = "ups.apcsmart";
const char command_CMD_UPS_MEGATEC[]                            PROGMEM = "ups.megatec";

const char command_CMD_SYSTEM_RUN[]                             PROGMEM = "system.run";

// do not insert new command to any position without syncing indexes. Tanx, Arduino and AVR, for this method of string array pushing to PROGMEM
// ~300 bytes of PROGMEM space can be saved with crazy "#ifdef-#else-#endif" dance
const char* const commands[] PROGMEM = {
  command_CMD_ZBX_NOPE,

  command_CMD_ZBX_AGENT_PING,
  command_CMD_ZBX_AGENT_HOSTNAME,
  command_CMD_ZBX_AGENT_VERSION,
  command_CMD_SYS_UPTIME,

  command_CMD_ARDUINO_ANALOGWRITE,
  command_CMD_ARDUINO_ANALOGREAD,

#ifdef FEATURE_AREF_ENABLE
  command_CMD_ARDUINO_ANALOGREFERENCE,
#else
  command_CMD_ZBX_NOPE,
#endif

  command_CMD_ARDUINO_DELAY,
  command_CMD_ARDUINO_DIGITALWRITE,
  command_CMD_ARDUINO_DIGITALREAD,

#ifdef FEATURE_TONE_ENABLE
  command_CMD_ARDUINO_TONE,
  command_CMD_ARDUINO_NOTONE,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_RANDOM_ENABLE
  command_CMD_ARDUINO_RANDOMSEED,
  command_CMD_ARDUINO_RANDOM,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_EEPROM_ENABLE
  command_CMD_SET_HOSTNAME,
  command_CMD_SET_PASSWORD,
  command_CMD_SET_SYSPROTECT,
  command_CMD_SET_NETWORK,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

  command_CMD_SYS_PORTWRITE,

#ifdef FEATURE_SHIFTOUT_ENABLE 
  command_CMD_SYS_SHIFTOUT,
#else
  command_CMD_ZBX_NOPE,
#endif

  command_CMD_SYS_REBOOT,

#ifdef FEATURE_DEBUG_COMMANDS_ENABLE
  command_CMD_SYS_MCU_NAME,
  command_CMD_SYS_NET_MODULE,
  command_CMD_SYS_CMD_COUNT,
  command_CMD_SYS_CMD_TIMEMAX,
  command_CMD_SYS_CMD_TIMEMAX_N,
  
  command_CMD_SYS_RAM_FREE,
  command_CMD_SYS_RAM_FREEMIN,
#else 
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif
  
  command_CMD_SYS_VCC,
  command_CMD_SYS_VCCMIN,
  command_CMD_SYS_VCCMAX,

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
  command_CMD_EXTINT_COUNT,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE
  command_CMD_INCENC_VALUE,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_OW_ENABLE
  command_CMD_OW_SCAN,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_I2C_ENABLE
  command_CMD_I2C_SCAN,
  command_CMD_I2C_WRITE,
  command_CMD_I2C_READ,
  command_CMD_I2C_BITWRITE,
  command_CMD_I2C_BITREAD,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_DS18X20_ENABLE
  command_CMD_DS18X20_TEMPERATURE,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_DHT_ENABLE
  command_CMD_DHT_HUMIDITY,
  command_CMD_DHT_TEMPERATURE,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_BMP_ENABLE
  command_CMD_BMP_PRESSURE,
  command_CMD_BMP_TEMPERATURE,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef SUPPORT_BME280_INCLUDE
  command_CMD_BME_HUMIDITY,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_BH1750_ENABLE
  command_CMD_BH1750_LIGHT,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_MAX7219_ENABLE
  command_CMD_MAX7219_WRITE,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_PCF8574_LCD_ENABLE
  command_CMD_PCF8574_LCDPRINT,
#else
  command_CMD_ZBX_NOPE,
#endif
  
#ifdef FEATURE_SHT2X_ENABLE
  command_CMD_SHT2X_HUMIDITY,
  command_CMD_SHT2X_TEMPERATURE,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif
  
#ifdef FEATURE_ACS7XX_ENABLE
  command_CMD_ACS7XX_ZC,
  command_CMD_ACS7XX_AC,
  command_CMD_ACS7XX_DC,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_ULTRASONIC_ENABLE
  command_CMD_ULTRASONIC_DISTANCE,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_IR_ENABLE
  command_CMD_IR_SEND,
  command_CMD_IR_SENDRAW,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_WS2812_ENABLE
  command_CMD_WS2812_SENDRAW,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_DEBUG_COMMANDS_ENABLE
  command_CMD_SYS_MCU_ID,
  command_CMD_SYS_MCU_SIGN,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_PZEM004_ENABLE
  command_CMD_PZEM004_CURRENT,
  command_CMD_PZEM004_VOLTAGE,
  command_CMD_PZEM004_POWER,
  command_CMD_PZEM004_ENERGY,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_UPS_APCSMART_ENABLE
  command_CMD_UPS_APCSMART,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_UPS_MEGATEC_ENABLE
  command_CMD_UPS_MEGATEC,
#else
  command_CMD_ZBX_NOPE,
#endif

command_CMD_SYSTEM_RUN,

};

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                             MESSAGES SECTION 
*/

#define MSG_ZBX_NOTSUPPORTED          	                      "ZBX_NOTSUPPORTED"

#define USE_TEXT_ERROR_MESSAGES

#ifdef USE_TEXT_ERROR_MESSAGES
  #define MSG_DEVICE_ERROR_CONNECT                              "Device not conected"
  #define MSG_DEVICE_ERROR_ACK_L                                "ACK (L) error"
  #define MSG_DEVICE_ERROR_ACK_H                                "ACK (H) error"
  #define MSG_DEVICE_ERROR_CHECKSUM                             "Wrong checksum"
  #define MSG_DEVICE_ERROR_TIMEOUT                              "Timeout error"
  #define MSG_DEVICE_ERROR_WRONG_ANSWER                         "Wrong answer recieved"
#else
  #define MSG_DEVICE_ERROR_CONNECT                              "-131"
  #define MSG_DEVICE_ERROR_ACK_L                                "-132"
  #define MSG_DEVICE_ERROR_ACK_H                                "-133"
  #define MSG_DEVICE_ERROR_CHECKSUM                             "-134"
  #define MSG_DEVICE_ERROR_TIMEOUT                              "-135"
  #define MSG_DEVICE_ERROR_WRONG_ANSWER                         "-136"
#endif

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           VARIOUS DEFINES SECTION 
*/

#define IDX_METRICS_LAST                                        8
#define IDX_METRICS_FIRST_CRONNED                               3

#define IDX_METRIC_SYS_CMD_COUNT                                0
#define IDX_METRIC_SYS_CMD_LAST                                 1
#define IDX_METRIC_SYS_CMD_TIMEMAX                              2
#define IDX_METRIC_SYS_CMD_TIMEMAX_N                            3

#define IDX_METRIC_SYS_VCC                                      4
#define IDX_METRIC_SYS_VCCMIN                                   5
#define IDX_METRIC_SYS_VCCMAX                                   6
#define IDX_METRIC_SYS_RAM_FREE                                 7
#define IDX_METRIC_SYS_RAM_FREEMIN                              8

// Zabbix v2.x header prefix ('ZBXD\x01')
#define ZBX_HEADER_PREFIX                                       "zbxd\1"
// sizeof() returns wrong result -> 6
#define ZBX_HEADER_PREFIX_LENGTH                                5
// Zabbix v2.x header length
#define ZBX_HEADER_LENGTH                                       12

#define SENS_READ_TEMP                                          0x01
#define SENS_READ_HUMD                                          0x02
#define SENS_READ_PRSS                                          0x03
#define SENS_READ_LUX                                           0x04

#define SENS_READ_ZC                                            0x08
#define SENS_READ_AC                                            0x09
#define SENS_READ_DC                                            0x0A

#define SENS_READ_VOLTAGE                                       0x0B
#define SENS_READ_POWER                                         0x0C
#define SENS_READ_ENERGY                                        0x0D

#define SENS_READ_RAW                                           0xFF

#define RESULT_IS_FAIL                                          false
#define RESULT_IS_OK                                            true
#define RESULT_IN_BUFFER                                        0x02
#define RESULT_IS_PRINTED                                       0x04
#define RESULT_IN_LONGVAR                                       0x08
#define RESULT_IN_ULONGVAR                                      0x10
#define RUN_NEW_COMMAND                                         -0x2

// Error Codes
//#define DEVICE_DISCONNECTED_C         	-127
#define DEVICE_ERROR_CONNECT                                    -0x01
#define DEVICE_ERROR_ACK_L                                      -0x02
#define DEVICE_ERROR_ACK_H                                      -0x04
#define DEVICE_ERROR_CHECKSUM                                   -0x08
#define DEVICE_ERROR_TIMEOUT                                    -0x10
#define DEVICE_ERROR_WRONG_ANSWER                               -0x20


/*
ADC channels 

     • Bits 3:0 – MUX[3:0]: Analog Channel Selection Bits
       The value of these bits selects which analog inputs are connected to the ADC. See Table 24-4 for details. If
       these bits are changed during a conversion, the change will not go in effect until this conversion is complete
       (ADIF in ADCSRA is set).

       Table 24-4. Input Channel Selections
       MUX3..0  Single Ended Input
       1110     1.1V (VBG)
       1111     0V (GND)       - noise level measurement possible
*/
#define ANALOG_CHAN_VBG 		                        0x0E // B1110
#define ANALOG_CHAN_GND 		                        0x0F // B1111

#define DBG_PRINT_AS_MAC 		                        0x01
#define DBG_PRINT_AS_IP  		                        0x02

#define ENCODER_STABILIZATION_DELAY                             2000L // 2000 microseconds
#define ADC_STABILIZATION_DELAY                                 1000L // 2000 microseconds
#define I2C_NO_REG_SPECIFIED                                    -0x01 //

// On Leonardo, Micro and other ATmega32u4 boards wait to Serial Monitor ready for 5sec 
#define SERIAL_WAIT_TIMEOUT            	                        5000UL  // 5 sec

//#define INT32_POSITIVE_MAX                                      2147483647

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         PROGRAMM STRUCTURES SECTION 
*/
// Note: netconfig_t size must be no more ___uint8_t___ bytes, because readConfig()'s read cycle use uint8_t counter. 
// Change the index's variable type if bigger size need
typedef struct {
  uint8_t useDHCP;         	 		  // 1 byte
  uint8_t macAddress[6];                          // 6 byte 
  IPAddress ipAddress;     			  // 6 byte (uint8_t[])
  IPAddress ipNetmask;     			  // 6 byte (uint8_t[])
  IPAddress ipGateway;     			  // 6 byte (uint8_t[])
  char hostname[ZBX_AGENT_HOSTNAME_MAXLEN+1];     // +1 for '\0', 255 - (1 + 6*4 + 4 + 1) = 225 bytes max
  // #ifdef ... #elif ... #endif does not work with struct operator
  uint32_t password;                              // 4 byte
  uint8_t useProtection;                          // 1 byte
} netconfig_t ;


typedef struct {                                  // 9 bytes: 
  union {                                         // 4 byte
    uint32_t count;        			  
    int32_t value;        			  
  };
  // mode == -1 => Interrupt is not attached
  int8_t mode;                                    // 1 byte 
  volatile uint8_t *encTerminalAPIR;              // 1 byte
  volatile uint8_t *encTerminalBPIR;              // 1 byte
  uint8_t encTerminalAPinBit;                     // 1 byte
  uint8_t encTerminalBPinBit;                     // 1 byte
} extInterrupt_t ;

typedef union {                                   // 4 byte
  uint32_t ulongvar;
  int32_t  longvar;
} long_ulong_t;


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         INLINE AND "DEFINE" FUNCTIONS SECTION 
*/

// HEX char to number
#define htod(_hex) ( (_hex >= 'a' && _hex <= 'f') ? (10 + _hex - 'a') : ( (_hex >= '0' && _hex <= '9') ? (_hex - '0') : 0)) 

// Convert dec number to hex char
#define dtoh(_dec) ( (_dec > 9) ? ('A' - 10 + _dec) : ('0' + _dec) )

// if _source have hex prefix - return true
#define haveHexPrefix(_source) ( (_source[0] == '0' && _source[1] == 'x') )

#define arraySize(_array) sizeof(_array) / sizeof(*(_array))

/* ****************************************************************************************************************************
*
*  Return "Free" memory size
*
**************************************************************************************************************************** */
inline __attribute__((always_inline)) uint32_t getRamFree(void) {
  extern uint16_t __heap_start, *__brkval;
  uint16_t v;
  return (uint32_t) &v - (__brkval == 0 ? (uint32_t) &__heap_start : (uint32_t) __brkval);
}

/* ****************************************************************************************************************************
*
*   Correct sys.vccmin/sys.vccmax metrics when VCC just taken
*
**************************************************************************************************************************** */
// __attribute__((always_inline)) 
inline void correctVCCMetrics(uint32_t _currVCC) {
  // Global variable from outside
  extern int32_t *sysMetrics;
  if (sysMetrics[IDX_METRIC_SYS_VCCMIN] > _currVCC) { sysMetrics[IDX_METRIC_SYS_VCCMIN] = _currVCC; }
  if (sysMetrics[IDX_METRIC_SYS_VCCMAX] < _currVCC) { sysMetrics[IDX_METRIC_SYS_VCCMAX] = _currVCC; }
}

#endif

