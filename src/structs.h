#ifndef _ZABBUINO_STRUCTS_H_
#define _ZABBUINO_STRUCTS_H_

#include <IPAddress.h>
#include <Arduino.h>
#include "NetworkAddress.h"
#include "../basic.h"
#include "tune.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            ALARM SECTION 
*/

// "No error detected" code
#define ERROR_NONE                 	                        0x00
// "No network activity" error code
#define ERROR_NET                 	                        0x01
// "DHCP problem" error code
#define ERROR_DHCP                 	                        0x02

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                        NETWORK MODULE SECTION 

   Note, that changing MAC or IP-address separately may cause "strange" network errors until the moment when the router delete old ARP-records from the cache.

   // Microchip forgot to step the number on the silcon when they released the revision B7. 6 is now rev B7. We still have
   // to see what they do when they release B8. At the moment there is no B8 out yet

*/

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         PROGRAMM STRUCTURES SECTION 
*/

// Note: netconfig_t size must be no more ___uint8_t___ bytes, because readConfig()'s read cycle use uint8_t counter. 
// Change the index's variable type if bigger size need
#pragma pack(push,1)
typedef struct {
  uint8_t CRC;                                    // 1 byte, CRC stores first, due it's EEPROM cell rewrites everytime on config saving if it changed. 
                                                  //         When the cell was broken, we just shift start config store address to next cell.
  // struct {
  // uint8_t useProtection: 1;                    // 1 bit
  // uint8_t useDHCP: 1;         	 	  // 1 bit
  // }
  uint8_t useProtection;                          // 1 byte
  uint8_t useDHCP;         	 		  // 1 byte
  uint8_t macAddress[6];                          // 6 byte 
  NetworkAddress ipAddress;     		  // 4 byte (uint8_t[])
  NetworkAddress ipNetmask;     		  // 4 byte (uint8_t[])
  NetworkAddress ipGateway;     	          // 4 byte (uint8_t[])
  char hostname[constAgentHostnameMaxLength+1];     // +1 for '\0', 255 - (1 + 1 + 1 + 6 + 4*4 + 1) = 229 bytes max
  // #ifdef ... #elif ... #endif does not work with struct operator
  uint32_t password;                              // 4 byte
  int16_t tzOffset;                               // 2 byte
} netconfig_t ;


typedef struct {
  uint32_t sysCmdCount;                           // Number of executed commands 
  uint8_t  sysCmdLast;                            // Index of last executed command
  uint32_t sysCmdLastExecTime;                    // End time of last executed command
  uint32_t sysCmdTimeMax;                         // Maximum spend time for command execution
  uint8_t  sysCmdTimeMaxN;                        // Index of the slowest command
//  uint16_t sysVCC;
  uint16_t sysVCCMin;                             // Maximum VCC (in mV) from MCU powering on
  uint16_t sysVCCMax;                             // Minimum VCC (in mV) from MCU powering on
  uint32_t sysRamFree;                            // "Current" free memory (in bytes).
  uint32_t sysRamFreeMin;                         // Minimum free memory (in bytes) from MCU powering on
  uint32_t netPHYReinits;                         // PHY reinits number (restarts of network module)
} sysmetrics_t;


typedef struct {                                  // 9 bytes: 
  uint32_t value;        			  
  uint8_t owner;                                  // 1 byte 
  // mode == -1 => Interrupt is not attached
  int8_t mode;                                    // 1 byte 
  volatile uint8_t *encTerminalAPIR;              // 1 byte
  volatile uint8_t *encTerminalBPIR;              // 1 byte
  uint8_t encTerminalAPinBit;                     // 1 byte
  uint8_t encTerminalBPinBit;                     // 1 byte
} extInterrupt_t ;
#pragma pack(pop) 
/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            COMMAND NAMES SECTION 
*/

// Need to add command macro with sequental number
#define CMD_ZBX_NOPE                                            0x00
#define CMD_ZBX_AGENT_PING                                      0x01
#define CMD_ZBX_AGENT_HOSTNAME                                  0x02
#define CMD_ZBX_AGENT_VERSION                                   0x03
#define CMD_SYSTEM_UPTIME                                       0x04
                                                                
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

#define CMD_SYSTEM_HW_CHASSIS                                   0x16
#define CMD_SYSTEM_HW_CPU                                       0x17
#define CMD_NET_PHY_NAME                                        0x18
#define CMD_NET_PHY_REINITS                                     0x19

#define CMD_SYS_CMD_COUNT                                       0x1A
#define CMD_SYS_CMD_TIMEMAX                                     0x1B
#define CMD_SYS_CMD_TIMEMAX_N                                   0x1C

#define CMD_SYS_RAM_FREE                                        0x1D
#define CMD_SYS_RAM_FREEMIN                                     0x1E

#define CMD_SYS_VCC                                             0x1F
#define CMD_SYS_VCCMIN                                          0x20
#define CMD_SYS_VCCMAX                                          0x21

#define CMD_EXTINT_COUNT                                        0x22
#define CMD_INCENC_VALUE                                        0x23

#define CMD_OW_SCAN                                             0x24

#define CMD_I2C_SCAN                                            0x25
#define CMD_I2C_WRITE                                           0x26
#define CMD_I2C_READ                                            0x27
#define CMD_I2C_BITWRITE                                        0x28
#define CMD_I2C_BITREAD                                         0x29

#define CMD_DS18X20_TEMPERATURE                                 0x2A

#define CMD_DHT_HUMIDITY                                        0x2B
#define CMD_DHT_TEMPERATURE                                     0x2C

#define CMD_BMP_PRESSURE                                        0x2D 
#define CMD_BMP_TEMPERATURE                                     0x2E
#define CMD_BME_HUMIDITY                                        0x2F

#define CMD_BH1750_LIGHT                                        0x30

#define CMD_MAX7219_WRITE                                       0x31

#define CMD_PCF8574_LCDPRINT                                    0x32

#define CMD_SHT2X_HUMIDITY                                      0x33
#define CMD_SHT2X_TEMPERATURE                                   0x34

#define CMD_ACS7XX_ZC                                           0x35
#define CMD_ACS7XX_AC                                           0x36
#define CMD_ACS7XX_DC                                           0x37

#define CMD_ULTRASONIC_DISTANCE                                 0x38

#define CMD_IR_SEND                                             0x39
#define CMD_IR_SENDRAW                                          0x3A

#define CMD_WS2812_SENDRAW                                      0x3B

#define CMD_PZEM004_CURRENT                                     0x3C 
#define CMD_PZEM004_VOLTAGE                                     0x3D 
#define CMD_PZEM004_POWER                                       0x3E  
#define CMD_PZEM004_ENERGY                                      0x3F
 
#define CMD_UPS_APCSMART                                        0x40
#define CMD_UPS_MEGATEC                                         0x41

#define CMD_SYSTEM_RUN                                          0x42   // relocate in future

#define CMD_INA219_BUSVOLTAGE                                   0x43
#define CMD_INA219_CURRENT                                      0x44
#define CMD_INA219_POWER                                        0x45

#define CMD_SET_LOCALTIME                                       0x46
#define CMD_SYSTEM_LOCALTIME                                    0x47

#define CMD_AT24CXX_WRITE                                       0x48
#define CMD_AT24CXX_READ                                        0x49

#define CMD_MAX44009_LIGHT                                      0x4A


// add new command as "const char command_<COMMAND_MACRO> PROGMEM". Only 'const' push string to PROGMEM. Tanx, Arduino & AVR.
// command_* values must be in lower case due analyze sub convert all chars to lower
const char command_CMD_ZBX_NOPE[]                               PROGMEM = "\1";
const char command_CMD_ZBX_AGENT_PING[]                         PROGMEM = "agent.ping";
const char command_CMD_ZBX_AGENT_HOSTNAME[]                     PROGMEM = "agent.hostname";
const char command_CMD_ZBX_AGENT_VERSION[]                      PROGMEM = "agent.version";
const char command_CMD_SYSTEM_UPTIME[]                          PROGMEM = "system.uptime";

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
const char command_CMD_SET_LOCALTIME[]                          PROGMEM = "set.localtime";

const char command_CMD_SYS_PORTWRITE[]                          PROGMEM = "portwrite";

const char command_CMD_SYS_SHIFTOUT[]                           PROGMEM = "shiftout";

const char command_CMD_SYS_REBOOT[]                             PROGMEM = "reboot";              

const char command_CMD_SYSTEM_HW_CHASSIS[]                      PROGMEM = "system.hw.chassis";
const char command_CMD_SYSTEM_HW_CPU[]                          PROGMEM = "system.hw.cpu";
const char command_CMD_SYSTEM_LOCALTIME[]                       PROGMEM = "system.localtime";

const char command_CMD_SYSTEM_RUN[]                             PROGMEM = "system.run";

const char command_CMD_NET_PHY_NAME[]                           PROGMEM = "net.phy.name";
const char command_CMD_NET_PHY_REINITS[]                        PROGMEM = "net.phy.reinits";

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

const char command_CMD_INA219_BUSVOLTAGE[]                      PROGMEM = "ina219.busvoltage";
const char command_CMD_INA219_CURRENT[]                         PROGMEM = "ina219.current";
const char command_CMD_INA219_POWER[]                           PROGMEM = "ina219.power";

const char command_CMD_AT24CXX_WRITE[]                          PROGMEM = "at24cxx.write";
const char command_CMD_AT24CXX_READ[]                           PROGMEM = "at24cxx.read";

const char command_CMD_MAX44009_LIGHT[]                         PROGMEM = "max44009.light";


// do not insert new command to any position without syncing indexes. Tanx, Arduino and AVR, for this method of string array pushing to PROGMEM
// ~300 bytes of PROGMEM space can be saved with crazy "#ifdef-#else-#endif" dance
const char* const commands[] PROGMEM = {
  command_CMD_ZBX_NOPE,

  command_CMD_ZBX_AGENT_PING,
  command_CMD_ZBX_AGENT_HOSTNAME,
  command_CMD_ZBX_AGENT_VERSION,
  command_CMD_SYSTEM_UPTIME,

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

#ifdef FEATURE_SYSINFO_ENABLE
  command_CMD_SYSTEM_HW_CHASSIS,
  command_CMD_SYSTEM_HW_CPU,
  command_CMD_NET_PHY_NAME,
  command_CMD_NET_PHY_REINITS,
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

#ifdef FEATURE_REMOTE_COMMANDS_ENABLE
command_CMD_SYSTEM_RUN,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_INA219_ENABLE
  command_CMD_INA219_BUSVOLTAGE,
  command_CMD_INA219_CURRENT,
  command_CMD_INA219_POWER,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_SYSTEM_RTC_ENABLE
  command_CMD_SET_LOCALTIME,
  command_CMD_SYSTEM_LOCALTIME,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_AT24CXX_ENABLE
  command_CMD_AT24CXX_WRITE,
  command_CMD_AT24CXX_READ,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_MAX44009_ENABLE
  command_CMD_MAX44009_LIGHT,
#else
  command_CMD_ZBX_NOPE,
#endif

};

/*
command_t const commands[] PROGMEM = {
  { command_CMD_ZBX_NOPE, CMD_ZBX_NOPE },
  { command_CMD_ZBX_AGENT_PING, CMD_ZBX_AGENT_PING },
  { command_CMD_ZBX_AGENT_HOSTNAME, CMD_ZBX_AGENT_HOSTNAME },
  { command_CMD_ZBX_AGENT_VERSION, CMD_ZBX_AGENT_VERSION },
  { command_CMD_SYS_UPTIME, CMD_SYS_UPTIME },

  { command_CMD_ARDUINO_ANALOGWRITE, CMD_ARDUINO_ANALOGWRITE },
  { command_CMD_ARDUINO_ANALOGREAD,  CMD_ARDUINO_ANALOGREAD }, 

#ifdef FEATURE_AREF_ENABLE
  { command_CMD_ARDUINO_ANALOGREFERENCE, CMD_ARDUINO_ANALOGREFERENCE},
#endif

  { command_CMD_ARDUINO_DELAY,        CMD_ARDUINO_DELAY },        
  { command_CMD_ARDUINO_DIGITALWRITE, CMD_ARDUINO_DIGITALWRITE }, 
  { command_CMD_ARDUINO_DIGITALREAD,  CMD_ARDUINO_DIGITALREAD },  

#ifdef FEATURE_TONE_ENABLE
  { command_CMD_ARDUINO_TONE,   CMD_ARDUINO_TONE },   
  { command_CMD_ARDUINO_NOTONE, CMD_ARDUINO_NOTONE }, 
#endif

#ifdef FEATURE_RANDOM_ENABLE
  { command_CMD_ARDUINO_RANDOMSEED, CMD_ARDUINO_RANDOMSEED },
  { command_CMD_ARDUINO_RANDOM,     CMD_ARDUINO_RANDOM },    
#endif

#ifdef FEATURE_EEPROM_ENABLE
  { command_CMD_SET_HOSTNAME,   CMD_SET_HOSTNAME },   
  { command_CMD_SET_PASSWORD,   CMD_SET_PASSWORD },   
  { command_CMD_SET_SYSPROTECT, CMD_SET_SYSPROTECT }, 
  { command_CMD_SET_NETWORK,    CMD_SET_NETWORK },    
#endif

  { command_CMD_SYS_PORTWRITE, CMD_SYS_PORTWRITE },

#ifdef FEATURE_SHIFTOUT_ENABLE 
  { command_CMD_SYS_SHIFTOUT,  CMD_SYS_SHIFTOUT },
#endif

  { command_CMD_SYS_REBOOT, CMD_SYS_REBOOT },

#ifdef FEATURE_SYSINFO_ENABLE
  { command_CMD_SYSTEM_HW_CPU,      CMD_SYSTEM_HW_CPU },      
  { command_CMD_NET_PHY_MODULE,    CMD_SYS_NET_MODULE },    
  { command_CMD_SYS_CMD_COUNT,     CMD_SYS_CMD_COUNT },     
  { command_CMD_SYS_CMD_TIMEMAX,   CMD_SYS_CMD_TIMEMAX },   
  { command_CMD_SYS_CMD_TIMEMAX_N, CMD_SYS_CMD_TIMEMAX_N }, 
  
  { command_CMD_SYS_RAM_FREE, CMD_SYS_RAM_FREE },
  { command_CMD_SYS_RAM_FREEMIN, CMD_SYS_RAM_FREEMIN },
#endif
  
  { command_CMD_SYS_VCC,    CMD_SYS_VCC },   
  { command_CMD_SYS_VCCMIN, CMD_SYS_VCCMIN },
  { command_CMD_SYS_VCCMAX, CMD_SYS_VCCMAX },

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
  { command_CMD_EXTINT_COUNT, CMD_EXTINT_COUNT },
#endif

#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE
  { command_CMD_INCENC_VALUE, CMD_INCENC_VALUE },
#endif

#ifdef FEATURE_OW_ENABLE
  { command_CMD_OW_SCAN, CMD_OW_SCAN },
#endif

#ifdef FEATURE_I2C_ENABLE
  { command_CMD_I2C_SCAN,     CMD_I2C_SCAN },     
  { command_CMD_I2C_WRITE,    CMD_I2C_WRITE },    
  { command_CMD_I2C_READ,     CMD_I2C_READ },     
  { command_CMD_I2C_BITWRITE, CMD_I2C_BITWRITE },
  { command_CMD_I2C_BITREAD,  CMD_I2C_BITREAD },  
#endif

#ifdef FEATURE_DS18X20_ENABLE
  { command_CMD_DS18X20_TEMPERATURE, CMD_DS18X20_TEMPERATURE },
#endif

#ifdef FEATURE_DHT_ENABLE
  { command_CMD_DHT_HUMIDITY,    CMD_DHT_HUMIDITY },   
  { command_CMD_DHT_TEMPERATURE, CMD_DHT_TEMPERATURE },
#endif

#ifdef FEATURE_BMP_ENABLE
  { command_CMD_BMP_PRESSURE,   CMD_BMP_PRESSURE },
  { command_CMD_BMP_TEMPERATURE, CMD_BMP_TEMPERATURE },
#endif

#ifdef SUPPORT_BME280_INCLUDE
  { command_CMD_BME_HUMIDITY, CMD_BME_HUMIDITY },
#endif

#ifdef FEATURE_BH1750_ENABLE
  { command_CMD_BH1750_LIGHT, CMD_BH1750_LIGHT },
#endif

#ifdef FEATURE_MAX7219_ENABLE
  { command_CMD_MAX7219_WRITE, CMD_MAX7219_WRITE },
#endif

#ifdef FEATURE_PCF8574_LCD_ENABLE
  { command_CMD_PCF8574_LCDPRINT, CMD_PCF8574_LCDPRINT },
#endif
  
#ifdef FEATURE_SHT2X_ENABLE
  { command_CMD_SHT2X_HUMIDITY,    CMD_SHT2X_HUMIDITY },    
  { command_CMD_SHT2X_TEMPERATURE, CMD_SHT2X_TEMPERATURE }, 
#endif
  
#ifdef FEATURE_ACS7XX_ENABLE
  { command_CMD_ACS7XX_ZC, CMD_ACS7XX_ZC }, 
  { command_CMD_ACS7XX_AC, CMD_ACS7XX_AC }, 
  { command_CMD_ACS7XX_DC, CMD_ACS7XX_DC }, 
#endif

#ifdef FEATURE_ULTRASONIC_ENABLE
  { command_CMD_ULTRASONIC_DISTANCE, CMD_ULTRASONIC_DISTANCE },
#endif

#ifdef FEATURE_IR_ENABLE
  { command_CMD_IR_SEND,    CMD_IR_SEND },    
  { command_CMD_IR_SENDRAW, CMD_IR_SENDRAW }, 
#endif

#ifdef FEATURE_WS2812_ENABLE
  { command_CMD_WS2812_SENDRAW, CMD_WS2812_SENDRAW },
#endif

#ifdef FEATURE_SYSINFO_ENABLE
  { command_CMD_SYS_MCU_ID,   CMD_SYS_MCU_ID },   
  { command_CMD_SYS_MCU_SIGN, CMD_SYS_MCU_SIGN }, 
#endif

#ifdef FEATURE_PZEM004_ENABLE
  { command_CMD_PZEM004_CURRENT, CMD_PZEM004_CURRENT }, 
  { command_CMD_PZEM004_VOLTAGE, CMD_PZEM004_VOLTAGE }, 
  { command_CMD_PZEM004_POWER,   CMD_PZEM004_POWER },   
  { command_CMD_PZEM004_ENERGY,  CMD_PZEM004_ENERGY },  
#endif

#ifdef FEATURE_UPS_APCSMART_ENABLE
  { command_CMD_UPS_APCSMART, CMD_UPS_APCSMART },
#endif

#ifdef FEATURE_UPS_MEGATEC_ENABLE
  { command_CMD_UPS_MEGATEC, CMD_UPS_MEGATEC },
#endif

  { command_CMD_SYSTEM_RUN, CMD_SYSTEM_RUN },

#ifdef FEATURE_INA219_ENABLE
  { command_CMD_INA219_BUSVOLTAGE, CMD_INA219_BUSVOLTAGE },
  { command_CMD_INA219_CURRENT,    CMD_INA219_CURRENT },   
  { command_CMD_INA219_POWER,      CMD_INA219_POWER },     
#endif

  { command_CMD_NET_ENC_REINITS,       CMD_NET_ENC_REINITS },       
  { command_CMD_NET_ENC_REINIT_REASON, CMD_NET_ENC_REINIT_REASON }, 
  { command_CMD_NET_ENC_PKTCNT_MAX,    CMD_NET_ENC_PKTCNT_MAX },    
};
*/
/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           VARIOUS DEFINES SECTION 
*/

// Zabbix v2.x header prefix ('ZBXD\x01')
#define ZBX_HEADER_PREFIX                                       "zbxd\1"
// sizeof() returns wrong result -> 6
#define ZBX_HEADER_PREFIX_LENGTH                                4
// Zabbix v2.x header length
#define ZBX_HEADER_LENGTH                                       12

/*

Enum take more progspace on compilation that macro :(
Why? All sources tell me thah enum is preprocessor feature. May be it use 16-bit width of ptr's or so?

typedef enum {
   SENS_READ_RAW,
   SENS_READ_TEMP,
   SENS_READ_HUMD,
   SENS_READ_PRSS,
   SENS_READ_LUX,
   SENS_READ_ZC,
   SENS_READ_AC,
   SENS_READ_DC,
   SENS_READ_VOLTAGE,
   SENS_READ_POWER,
   SENS_READ_ENERGY
} sens_metrics_t;
*/

#define SENS_READ_TEMP                                          0x01
#define SENS_READ_HUMD                                          0x02
#define SENS_READ_PRSS                                          0x03
#define SENS_READ_LUX                                           0x04

#define SENS_READ_ZC                                            0x08
#define SENS_READ_AC                                            0x09
#define SENS_READ_DC                                            0x0A

#define SENS_READ_VOLTAGE                                       0x0B
#define SENS_READ_SHUNT_VOLTAGE                                 0x0C
#define SENS_READ_BUS_VOLTAGE                                   0x0D
#define SENS_READ_POWER                                         0x0E
#define SENS_READ_ENERGY                                        0x0F

#define SENS_READ_RAW                                           0xFF

#define RESULT_IS_FAIL                                          false
#define RESULT_IS_OK                                            true
#define RESULT_IN_BUFFER                                        0x02
#define RESULT_IS_PRINTED                                       0x04
#define RESULT_IS_SIGNED_VALUE                                  0x08
#define RESULT_IS_UNSIGNED_VALUE                                0x10
// RESULT_IS_NEW_COMMAND's value must not equal any command index to avoid incorrect processing
#define RESULT_IS_NEW_COMMAND                                   -0x02

// Error Codes
//#define DEVICE_DISCONNECTED_C         	-127

#define ZBX_NOTSUPPORTED                                        -0x01
#define DEVICE_ERROR_CONNECT                                    -0x02
#define DEVICE_ERROR_ACK_L                                      -0x04
#define DEVICE_ERROR_ACK_H                                      -0x08
#define DEVICE_ERROR_CHECKSUM                                   -0x10
#define DEVICE_ERROR_TIMEOUT                                    -0x20
#define DEVICE_ERROR_WRONG_ID                                   -0x30
#define DEVICE_ERROR_NOT_SUPPORTED                              -0x40
#define DEVICE_ERROR_WRONG_ANSWER                               -0x50
#define DEVICE_ERROR_EEPROM_CORRUPTED                           -0x60
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

#define I2C_NO_REG_SPECIFIED                                    -0x01 //

// Length of netconfig_t's CRC field
#define CONFIG_CRC_LEN                                          0x01
// PoConfig will be stored or loaded on ...
#define CONFIG_STORE_PTR_ADDRESS                                0x01
#define CONFIG_STORE_DEFAULT_START_ADDRESS                      0x02
// one byte used to pointer to EEPROM's start address from which config will saved, max stored pointer value is 255
#define LAST_EEPROM_CELL_ADDRESS                                0xFF 

// Who is use interrupt pin
#define OWNER_IS_NOBODY                                         0x00
#define OWNER_IS_EXTINT                                         0x01
#define OWNER_IS_INCENC                                         0x02

#define NO_REINIT_ANALYZER                                      false
#define REINIT_ANALYZER                                         true

#define WANTS_VALUE_NONE                                        0x00
#define WANTS_VALUE_WHOLE                                       0x01
#define WANTS_VALUE_SCALED                                      0x0F


#endif // _ZABBUINO_STRUCTS_H_

