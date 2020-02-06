#pragma once
#include <Arduino.h>
#include <avr/wdt.h>
#include "sys_macros.h"


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            DISPATCH SECTION 

*/


#define COMPILER_WARNING_UNUSED_VAR_SUPRESSION

#ifdef COMPILER_WARNING_UNUSED_VAR_SUPRESSION
#define __SUPPRESS_WARNING_UNUSED(_code) (void)(_code)
#else
#define __SUPPRESS_WARNING_UNUSED(_code) (void)0
#endif


// Enable LCD support if report screen required
#if defined (FEATURE_SYSTEM_DISPLAY_ENABLE)
    #define FEATURE_PCF8574_LCD_ENABLE
#endif

#if defined (FEATURE_SYSTEM_RTC_DS3231_ENABLE)    || \
    defined (FEATURE_SYSTEM_RTC_PCF8563_ENABLE)
    #define FEATURE_SYSTEM_RTC_ENABLE
#endif

#if defined (FEATURE_BMP180_ENABLE)
    #define FEATURE_BMP_ENABLE
    #define SUPPORT_BMP180_INCLUDE
#endif

#if defined (FEATURE_BMP280_ENABLE)
    #define FEATURE_BMP_ENABLE
    #define SUPPORT_BMP280_INCLUDE
#endif

// Enable BMP280 support if need to use BME280, because BME280 is BMP280+Humidity sensor.
#if defined (FEATURE_BME280_ENABLE)
    #define FEATURE_BMP_ENABLE
    #define SUPPORT_BMP280_INCLUDE
    #define SUPPORT_BME280_INCLUDE
#endif


// Need to use Wire lib if any I2C related feature enabled
#if defined (FEATURE_I2C_ENABLE)                || \
    defined (FEATURE_BMP_ENABLE)                || \
    defined (FEATURE_BH1750_ENABLE)             || \
    defined (FEATURE_PCF8574_LCD_ENABLE)        || \
    defined (FEATURE_SHT2X_ENABLE)              || \
    defined (FEATURE_SYSTEM_RTC_ENABLE)         || \
    defined (FEATURE_MAX44009_ENABLE)           || \
    defined (FEATURE_VEML6070_ENABLE)           || \
    defined (FEATURE_PCA9685_ENABLE)            || \
    defined (FEATURE_ADPS9960_ENABLE)           || \
    defined (FEATURE_TSL2561_ENABLE)            || \
    defined (FEATURE_MLX90614_ENABLE)           || \
    defined (FEATURE_SGP30_ENABLE)              || \
    defined (FEATURE_T67XX_ENABLE)              || \
    defined (FEATURE_WUHAN_CUBIC_PM_I2C_ENABLE) || \
    defined (FEATURE_INA219_ENABLE)
    #define TWI_USE
#endif

// !!! does not work
//#if defined (FEATURE_SERIAL_LISTEN_TOO) && (!defined(FEATURE_DEBUG_MESSAGING_LEVEL) || (FEATURE_DEBUG_MESSAGING_LEVEL < 1))
//    #define FEATURE_DEBUG_MESSAGING_LEVEL  1
//#endif

// Need to use Hardware Serial if any UART related feature enabled
#if defined (FEATURE_DEBUG_TO_SERIAL_DEV)    || \
    defined (FEATURE_SERIAL_LISTEN_TOO)      || \
    defined (FEATURE_NET_DEBUG_TO_SERIAL)    || \
    (FEATURE_DEBUG_MESSAGING_LEVEL > 0)       
    #define SERIAL_USE
#endif

// Need to allow ltoaf() function if these features used
#if defined (FEATURE_DHT_ENABLE)         || \
    defined (FEATURE_BH1750_ENABLE)      || \
    defined (FEATURE_MAX44009_ENABLE)    || \
    defined (FEATURE_MLX90614_ENABLE)    || \
    defined (FEATURE_SHT2X_ENABLE)       || \
    defined (FEATURE_DS18X20_ENABLE)     || \
    defined (FEATURE_MAX6675_ENABLE)     || \
    defined (FEATURE_PZEM004_ENABLE)     || \
    defined (FEATURE_BMP180_ENABLE)      || \
    defined (FEATURE_BMP280_ENABLE)      
    #define FUNCTION_LTOAF_USE
#endif

// Need to allow ltoaf() function if these features used
#if defined (FEATURE_BME280_ENABLE)   
    #define FUNCTION_QTOAF_USE
#endif

// Need to use init Interrupt stuff if any interrupt related feature enabled
#if defined(FEATURE_EXTERNAL_INTERRUPT_ENABLE) || defined(FEATURE_INCREMENTAL_ENCODER_ENABLE)
   #define INTERRUPT_USE
#endif

// Define I/O ports number to reserve port_protect, port_pullup and other arrays size 
// see below: const uint8_t port_protect[PORTS_NUM] = {...}
#if defined (ARDUINO_AVR_DUEMILANOVE)    || \
    defined (ARDUINO_AVR_UNO)            || \
    defined (ARDUINO_AVR_MINI)           || \
    defined (ARDUINO_AVR_NANO)           || \
    defined (ARDUINO_AVR_PRO)            || \
    defined (ARDUINO_AVR_ETHERNET)
    #define PORTS_NUM                                           0x05

#elif defined (ARDUINO_AVR_LEONARDO)     || \
    defined (ARDUINO_AVR_MICRO)          || \
    defined (ARDUINO_AVR_ROBOT_CONTROL)  || \
    defined (ARDUINO_AVR_ROBOT_MOTOR)    || \
    defined (ARDUINO_AVR_YUN)
    #define PORTS_NUM                                           0x07

#elif defined (ARDUINO_AVR_MEGA) || \
    defined (ARDUINO_AVR_MEGA2560)
    #define PORTS_NUM                                           0x0D
#else // Unknow boards equal to "Arduino Duemilanove"
//    #define ARDUINO_AVR_DUEMILANOVE
    #define PORTS_NUM                                           0x05
#endif
/*

#if not defined (ARDUINO_AVR_DUEMILANOVE)    || \
    not defined (ARDUINO_AVR_MINI)           || \
    not defined (ARDUINO_AVR_NANO)           || \
    not defined (ARDUINO_AVR_BT)             || \
    not defined (ARDUINO_AVR_FIO)            || \
    not defined (ARDUINO_AVR_PRO)            || \
    not defined (ARDUINO_AVR_ETHERNET)       || \
    not defined (ARDUINO_AVR_LEONARDO)       || \
    not defined (ARDUINO_AVR_MICRO)          || \
    not defined (ARDUINO_AVR_ROBOT_CONTROL)  || \
    not defined (ARDUINO_AVR_ROBOT_MOTOR)    || \
    not defined (ARDUINO_AVR_YUN)            || \
    not defined (ARDUINO_AVR_MEGA)           || \
    not defined (ARDUINO_AVR_MEGA2560)       

    #define ARDUINO_AVR_UNO

#endif
*/


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            ALARM SECTION 
*/

// Must be in "sys_structs.h", but "sys_structs.h" want "cfg_tune.h", and "cfg_tune.h" want "sys_structs.h" 
#pragma pack(push,1)
typedef struct {
  uint32_t onTime;
  uint32_t allTime;
} blinkSettings_t;
#pragma pack(pop)


const blinkSettings_t blinkSettings[]={
  {000UL, 1000UL}, // ERROR_NONE 0 ms   on, (1000-0) ms   off
  {250UL,  500UL}, // ERROR_NET  250 ms on, (500-250) ms  off
  {750UL, 1500UL}, // ERROR_DHCP 750 ms on, (1500-750) ms off
  {500UL, 1000UL}, // ERROR_NO_NET_ACTIVITY 500 ms on, (1000-500) ms off
};

// Turn off state LED blink (no errors found)
const uint32_t constBlinkNope                                   = 000UL;
// State LED blink type with DHCP problem reached (no renew lease or more)
// ~150ms on, ~850ms off
const uint32_t constBlinkDhcpProblem         	                = 750UL; 
// State LED blink type with Network activity problem (no packets processed for constNetIdleTimeout)
// ~500ms on, ~500ms off
const uint32_t constBlinkNetworkProblem                         = 250UL;
//
const uint32_t constBlinkNoNetActivityProblem                   = 500UL;


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          SYSTEM HARDWARE SECTION 
*/

const uint8_t  constDefaultSDAPin                               = SDA;
const uint8_t  constDefaultSCLPin                               = SCL;

// System RTC module settings (only DS3231 is supported at this time)
const uint8_t  constSystemRtcSDAPin                             = SDA;     // SDA - A4
const uint8_t  constSystemRtcSCLPin                             = SCL;     // SCL - A5
#if defined (FEATURE_SYSTEM_RTC_DS3231_ENABLE)
const uint8_t  constSystemRtcI2CAddress                         = 0x68;   // DS3231 RTC I2C address 
#elif defined (FEATURE_SYSTEM_RTC_PCF8563_ENABLE)
const uint8_t  constSystemRtcI2CAddress                         = 0x51;   // PCF8563 RTC I2C address 
#endif

const uint16_t constUserFunctionCallInterval                    = 1000UL; // 1sec


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            NETWORK MODULE SECTION 

   Note, that changing MAC or IP-address separately may cause "strange" network errors until the moment when the router delete old ARP-records from the cache.

   // Microchip forgot to step the number on the silcon when they released the revision B7. 6 is now rev B7. We still have
   // to see what they do when they release B8. At the moment there is no B8 out yet
*/

// How often do ENC28J60 module reinit for more stable network
// 5 sec
const uint32_t constPHYCheckInterval                            = 10000UL; 

// Network activity timeout (for which no packets processed or no DHCP lease renews finished with success)
// 60 sec
const uint32_t constNetIdleTimeout            	                = 60000UL; 

// How long active client can transmit packets
#ifdef SERIAL_USE
  // on debug serial's output can make network processing slow
  // 5 seconds                                                  
  const uint32_t constNetSessionTimeout                         = 5000UL;
#else
  // 1 second
  const uint32_t constNetSessionTimeout                         = 1000UL; 
#endif

// How often do renew DHCP lease
// 30 sec
//const uint32_t constNetDhcpRenewPeriod                         = 30000UL;
// How often do renew DHCP lease
// 0.1 sec
const uint32_t constNetStabilizationDelay                       = 100UL; 

// How much system wait for Ethernet Shield resetting
// https://arduinodiy.wordpress.com/2017/04/12/the-w5100-bug/ -> Bug 3 – The Funduino Reset Bug
const uint32_t constEthernetShieldInitDelay                     = 250UL;

const uint32_t consNetDebugPrintInterval                        = 5000UL;
/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         SYSTEM CONFIGURATION SECTION 
*/

// How long need to hold button for factory reset
// 5 seconds
const uint32_t constHoldTimeToFactoryReset                      = 5000UL; 

// How many secs device may be stay in infinitibe loop before reboot
// Also you can use:
// WDTO_1S
// WDTO_2S
// WDTO_4S - not for all controllers (please reference to avr/wdt.h)
// WDTO_8S - not for all controllers (please reference to avr/wdt.h)
const uint32_t constWtdTimeout                                  = (WDTO_8S);

// How often need gather system metrics
// 1 sec
const uint32_t constSysMetricGatherPeriod                       = 1000UL; 

// Number of expected arguments of the command (argc)
const uint8_t constArgC                                         = 6;
// Size of buffer's argument part. All separators and delimiters must be taken into account. See note to constBufferSize macro too
//const uint16_t constArgsPartSize                               = 163;
const uint16_t constArgsPartSize                                = 250;
// Size of buffer's command part
const uint8_t constCmdPartSize                                  = 25;


// ***NOTE****    
//                  
// Total buffer size cannot be so small, because many subroutines use its too:
// - you need at least 75 bytes if set.network[] command will be used
// - getMegatecUPSMetric() can write to its up to MEGATEC_MAX_ANSWER_LENGTH bytes, for example
// The total size of the buffer. 
const uint16_t constBufferSize                                  = constCmdPartSize + constArgsPartSize;

// How long the ID of MCU (in bytes)
const uint8_t constMcuIdSize                                    = 10;

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          AGENT CONFIGURATION SECTION 
*/

// How much bytes will be allocated to hostname store
// sizeof() is not used here to get constant memory allocation due set.hostname() can operate longer strings
const uint8_t constAgentHostnameMaxLength                       = 32;  // MCU ID as hostname take 32 chars

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          I/O PORTS/PINS PRE-CONFIGURATION SECTION 
*/

const uint8_t port_protect[] = {
/*

  All bits equal '0' cause setting corresponding pin to non-protection mode
  
  All bits equal '1' cause setting corresponding pin to protection mode (it's will be not affected by portWrite[], digitalWrite[] and other commands)
  
*/
#if (PORTS_NUM >= 0x05)
  B00000000, // not a port
  B00000000, // not a port
  // Pins D10, D11, D12, D13 is protected by setting 2, 3, 4, 5 bits, due its used to SPI (ethernet shield)
  // Pin D9 is used by Timer1 and can't be used for PWM (analogWrite) when system metric was gathered on Timer1 interrupt
  // Bits 6, 7 have not correspondented pins in Arduino Mini Pro / Freeduino 2009
  B11111111, /*     PORTB        
D13 -^    ^- D8    <- pins   */
  B00000000, /*     PORTC 
   ^-A7   ^-A0   <- pins    */
  // Pins D0, D1 is protected by settings 0, 1 bits, due its used to RX/TX lines of UART and make it possible to transmit data to Serial Monitor  
  B00000000  /*     PORTD 
   ^-D7   ^-D0   <- pins    */
#elif (PORTS_NUM >= 0x07)
  // check ports settings for your platform
  B00000000, // not a port
  B00000000, // not a port
  B00000000, // PORTB
  B00000000, // PORTC
  B00000000, // PORTD
  B00000000, // PORTE
  B00000000  // PORTF
#elif (PORTS_NUM >= 0x0D)
  // check ports settings for your platform
  B00000000, // not a port
  B00000000, // PORTA
  B00011111, /* PORTB
  D10-^^- D50 ... D53 (SPI BUS + D10 - Ethernet.h's CS pin) */
  B00000000, // PORTC
  B00000000, // PORTD
  B00000011, /* PORTE
         ^-D1,D0 (USART0) */
  B00000000, // PORTF
  B00000000, // PORTG
  B01100000, /* PORTH
    ^-D9,D8 (Status LED, Factory Reset Button */
  B00000000, // not a port
  B00000000, // PORTJ
  B00000000, // PORTK
  B00000000  // PORTL
#endif
};



const uint8_t port_mode[] PROGMEM = {
//const uint8_t port_mode[PORTS_NUM] = {
/*
 
  All bits equal '0' cause setting corresponding pin to INPUT mode
  
  All bits equal '1' cause setting corresponding pin to OUTPUT mode
  
*/
 
#if (PORTS_NUM >= 0x05)
  B00000000, // not a port
  B00000000, // not a port
  // Bits 6, 7 have not correspondented pins in Arduino Mini Pro / Freeduino 2009
  B00000000, /*     PORTB        
D13 -^    ^- D8    <- pins   */
  B00000000, /*     PORTC 
   ^-A7   ^-A0   <- pins    */
  B00000000  /*     PORTD 
   ^-D7   ^-D0   <- pins    */
#elif (PORTS_NUM >= 0x07)
  // check ports settings for your platform
  B00000000, // not a port
  B00000000, // not a port
  B00000000, // PORTB
  B00000000, // PORTC
  B00000000, // PORTD
  B00000000, // PORTE
  B00000000  // PORTF
#elif (PORTS_NUM >= 0x0D)
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


const uint8_t port_pullup[] PROGMEM = {
//const uint8_t port_pullup[PORTS_NUM] = {
/*
   All bits equal '0' cause do not pull-up corresponding pin

   All bits equal '1' cause pull-up corresponding pin

*/

#if (PORTS_NUM >= 0x05)
  B00000000, // not a port
  B00000000, // not a port
  // Bits 6, 7 have not correspondented pins in Arduino Mini Pro / Freeduino 2009
  B00000000, /*     PORTB        
D13 -^    ^- D8    <- pins   */
  B00000000, /*     PORTC 
   ^-A7   ^-A0   <- pins    */
  B00000000  /*     PORTD 
   ^-D7   ^-D0   <- pins    */
#elif (PORTS_NUM >= 0x07)
  // check ports settings for your platform
  B00000000, // not a port
  B00000000, // not a port
  B00000000, // PORTB
  B00000000, // PORTC
  B00000000, // PORTD
  B00000000, // PORTE
  B00000000  // PORTF
#elif (PORTS_NUM >= 0x0D)
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
                                                             MESSAGES SECTION 
*/

const char MSG_ZBX_NOTSUPPORTED[]                       PROGMEM = "ZBX_NOTSUPPORTED";
const char MSG_ZBX_UNEXPECTED_RC[]                      PROGMEM = "Unexpected retcode";

#ifdef USE_TEXT_ERROR_MESSAGES  
  const char MSG_DEVICE_ERROR_CONNECT[]                 PROGMEM = "Device not conected";
  const char MSG_DEVICE_ERROR_ACK_L[]                   PROGMEM = "ACK (L) error";
  const char MSG_DEVICE_ERROR_ACK_H[]                   PROGMEM = "ACK (H) error";
  const char MSG_DEVICE_ERROR_CHECKSUM[]                PROGMEM = "Wrong checksum";
  const char MSG_DEVICE_ERROR_TIMEOUT[]                 PROGMEM = "Timeout error";
  const char MSG_DEVICE_ERROR_WRONG_ID[]                PROGMEM = "Wrong ID";
  const char MSG_DEVICE_ERROR_NOT_SUPPORTED[]           PROGMEM = "Device not supported";
  const char MSG_DEVICE_ERROR_WRONG_ANSWER[]            PROGMEM = "Wrong answer recieved";
  const char MSG_DEVICE_ERROR_EEPROM[]                  PROGMEM = "Can't save to EEPROM";
#else                                                   
  const char MSG_DEVICE_ERROR_CONNECT[]                 PROGMEM = "-131";
  const char MSG_DEVICE_ERROR_ACK_L[]                   PROGMEM = "-132";
  const char MSG_DEVICE_ERROR_ACK_H[]                   PROGMEM = "-133";
  const char MSG_DEVICE_ERROR_CHECKSUM[]                PROGMEM = "-134";
  const char MSG_DEVICE_ERROR_TIMEOUT[]                 PROGMEM = "-135";
  const char MSG_DEVICE_ERROR_WRONG_ID[]                PROGMEM = "-136";
  const char MSG_DEVICE_ERROR_NOT_SUPPORTED[]           PROGMEM = "-137"; 
  const char MSG_DEVICE_ERROR_WRONG_ANSWER[]            PROGMEM = "-138";
  const char MSG_DEVICE_ERROR_EEPROM[]                  PROGMEM = "-151";
#endif

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           VARIOUS DEFINES SECTION 
*/
// How much need to wait to get encoder stabilization
// 2000 microseconds
const uint32_t constEncoderStabilizationDelay                   = 2000UL; 

// How much need to wait to get ADC stabilization
// 1000 microseconds
const uint32_t constAdcStabilizationDelay                       = 1000UL; 

// On Leonardo, Micro and other ATmega32u4 boards wait to Serial Monitor ready for 5sec 
const uint32_t constSerialWaitTimeout                           = 5000UL;  
 
// 10 bit ADC
const uint16_t constAnalogReadMappingLowValue                   = 0x00;
const uint16_t constAnalogReadMappingHighValue                  = 1023;