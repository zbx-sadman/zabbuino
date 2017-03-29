#ifndef _ZABBUINO_TUNE_CONFIG_H_
#define _ZABBUINO_TUNE_CONFIG_H_

#include <Arduino.h>

#include <avr/wdt.h>

const char mySSID[] = "CheckMe";
const char myPSK[] = "trumptrump";

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            DISPATCH SECTION 

*/
// Enable LCD support if report screen required
#if defined(FEATURE_SYSTEM_DISPLAY_ENABLE)
   #define FEATURE_PCF8574_LCD_ENABLE
#endif

#if defined(FEATURE_SYSTEM_RTC_ENABLE)
   #define FEATURE_I2C_RTC_ENABLE
#endif

#if defined(FEATURE_BMP180_ENABLE)
   #define FEATURE_BMP_ENABLE
   #define SUPPORT_BMP180_INCLUDE
#endif

#if defined(FEATURE_BMP280_ENABLE)
   #define FEATURE_BMP_ENABLE
   #define SUPPORT_BMP280_INCLUDE
#endif

// Enable BMP280 support if need to use BME280, because BME280 is BMP280+Humidity sensor.
#if defined(FEATURE_BME280_ENABLE)
   #define FEATURE_BMP_ENABLE
   #define SUPPORT_BMP280_INCLUDE
   #define SUPPORT_BME280_INCLUDE
#endif


// Need to use Wire lib if any I2C related feature enabled
#if defined(FEATURE_I2C_ENABLE) || defined(FEATURE_BMP_ENABLE) || defined(FEATURE_BH1750_ENABLE) || defined (FEATURE_PCF8574_LCD_ENABLE) || defined (FEATURE_SHT2X_ENABLE) || defined (FEATURE_I2C_RTC_ENABLE) || defined (FEATURE_MAX44009_ENABLE)
   #define TWI_USE
#endif

#if defined(FEATURE_SERIAL_LISTEN_TOO) && !(defined(FEATURE_DEBUG_TO_SERIAL_LOW) || defined(FEATURE_DEBUG_TO_SERIAL_MIDDLE) || defined(FEATURE_DEBUG_TO_SERIAL_HIGH))
   #define FEATURE_DEBUG_TO_SERIAL_LOW
#endif

// Need to use Hardware Serial if any UART related feature enabled
#if defined(FEATURE_DEBUG_TO_SERIAL_DEV) || defined(FEATURE_DEBUG_TO_SERIAL_LOW) || defined(FEATURE_DEBUG_TO_SERIAL_MIDDLE) || defined(FEATURE_DEBUG_TO_SERIAL_HIGH) || defined(FEATURE_SERIAL_LISTEN_TOO) || defined(FEATURE_NET_DEBUG_TO_SERIAL)
   #define SERIAL_USE
#endif

// Need to use init Interrupt stuff if any interrupt related feature enabled
#if defined(FEATURE_EXTERNAL_INTERRUPT_ENABLE) || defined(FEATURE_INCREMENTAL_ENCODER_ENABLE)
   #define INTERRUPT_USE
#endif


// Define I/O ports number to reserve port_protect, port_pullup and other arrays size 
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

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            ALARM SECTION 
*/
// Turn off state LED blink (no errors found)
const uint32_t constBlinkNope                                  = 000UL;
// State LED blink type with DHCP problem reached (no renew lease or more)
// ~150ms on, ~850ms off
const uint32_t constBlinkDhcpProblem         	               = 150UL; 
// State LED blink type with Network activity problem (no packets processed for constNetIdleTimeout)
// ~500ms on, ~500ms off
const uint32_t constBlinkNetworkProblem                        = 500UL;


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          SYSTEM HARDWARE SECTION 
*/

const uint8_t  constDefaultSDAPin                               = A4;
const uint8_t  constDefaultSCLPin                               = A5;

// System RTC module settings (only DS3231 is supported at this time)
const uint8_t  constSystemRtcSDAPin                             = A4;     // SDA - A4
const uint8_t  constSystemRtcSCLPin                             = A5;     // SCL - A5
const uint8_t  constSystemRtcI2CAddress                         = 0x68;   // DS3231 RTC I2C address 

const uint16_t constUserFunctionCallInterval                    = 3000UL; // 3sec


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            NETWORK MODULE SECTION 

   Note, that changing MAC or IP-address separately may cause "strange" network errors until the moment when the router delete old ARP-records from the cache.

   // Microchip forgot to step the number on the silcon when they released the revision B7. 6 is now rev B7. We still have
   // to see what they do when they release B8. At the moment there is no B8 out yet
*/

// How often do ENC28J60 module reinit for more stable network
// 5 sec
const uint32_t constPHYCheckInterval                           = 5000UL; 

// Network activity timeout (for which no packets processed or no DHCP lease renews finished with success)
// 60 sec
const uint32_t constNetIdleTimeout            	               = 60000UL; 

// How long active client can transmit packets
#ifdef SERIAL_USE
  // on debug serial's output can make network processing slow
  // 5 seconds
  const uint32_t constNetSessionTimeout                        = 5000UL;
#else
  // 1 second
  const uint32_t constNetSessionTimeout                        = 1000UL; 
#endif

// How often do renew DHCP lease
// 30 sec
//const uint32_t constNetDhcpRenewPeriod                         = 30000UL;
// How often do renew DHCP lease
// 0.1 sec
const uint32_t constNetStabilizationDelay                      = 100UL; 

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         SYSTEM CONFIGURATION SECTION 
*/

// How long need to hold button for factory reset
// 5 seconds
const uint32_t constHoldTimeToFactoryReset                     = 5000UL; 

// How many secs device may be stay in infinitibe loop before reboot
// Also you can use:
// WDTO_1S
// WDTO_2S
// WDTO_4S - not for all controllers (please reference to avr/wdt.h)
// WDTO_8S - not for all controllers (please reference to avr/wdt.h)
const uint32_t constWtdTimeout                                 = (WDTO_8S);

// How often need gather system metrics
// 1 sec
const uint32_t constSysMetricGatherPeriod                      = 1000UL; 

// Number of expected arguments of the command (argc)
const uint8_t constArgC                                        = 6;
// Size of buffer's argument part. All separators and delimiters must be taken into account. See note to constBufferSize macro too
//const uint16_t constArgsPartSize                               = 163;
const uint16_t constArgsPartSize                               = 25;
// Size of buffer's command part
const uint8_t constCmdPartSize                                 = 100;


// ***NOTE****    
//                  
// Total buffer size cannot be so small, because many subroutines use its too:
// - you need at least 75 bytes if set.network[] command will be used
// - getMegatecUPSMetric() can write to its up to MEGATEC_MAX_ANSWER_LENGTH bytes, for example
// The total size of the buffer. 
const uint16_t constBufferSize                                 = constCmdPartSize + constArgsPartSize;

// How long the ID of MCU
const uint8_t constMcuIdLength                                 = 20;

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          AGENT CONFIGURATION SECTION 
*/

// How much bytes will be allocated to hostname store
// sizeof() is not used here to get constant memory allocation due set.hostname() can operate longer strings
const uint8_t constAgentHostnameMaxLength                      = 32;  // MCU ID as hostname take 32 chars

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          I/O PORTS/PINS PRE-CONFIGURATION SECTION 
*/

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
  B11111111, /*     PORTB        
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



const uint8_t port_mode[PORTS_NUM] PROGMEM = {
//const uint8_t port_mode[PORTS_NUM] = {
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
  B11111111, /*     PORTC 
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


const uint8_t port_pullup[PORTS_NUM] PROGMEM = {
//const uint8_t port_pullup[PORTS_NUM] = {
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


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                             MESSAGES SECTION 
*/

#define MSG_ZBX_NOTSUPPORTED          	                       "ZBX_NOTSUPPORTED"

#ifdef USE_TEXT_ERROR_MESSAGES
  #define MSG_DEVICE_ERROR_CONNECT                              "Device not conected"
  #define MSG_DEVICE_ERROR_ACK_L                                "ACK (L) error"
  #define MSG_DEVICE_ERROR_ACK_H                                "ACK (H) error"
  #define MSG_DEVICE_ERROR_CHECKSUM                             "Wrong checksum"
  #define MSG_DEVICE_ERROR_TIMEOUT                              "Timeout error"
  #define MSG_DEVICE_ERROR_WRONG_ID                             "Wrong ID" 
  #define MSG_DEVICE_ERROR_NOT_SUPPORTED                        "Device not supported" 
  #define MSG_DEVICE_ERROR_WRONG_ANSWER                         "Wrong answer recieved"
  #define MSG_DEVICE_ERROR_EEPROM                               "Can't save to EEPROM"
#else
  #define MSG_DEVICE_ERROR_CONNECT                              "-131"
  #define MSG_DEVICE_ERROR_ACK_L                                "-132"
  #define MSG_DEVICE_ERROR_ACK_H                                "-133"
  #define MSG_DEVICE_ERROR_CHECKSUM                             "-134"
  #define MSG_DEVICE_ERROR_TIMEOUT                              "-135"
  #define MSG_DEVICE_ERROR_WRONG_ID                             "-136" 
  #define MSG_DEVICE_ERROR_NOT_SUPPORTED                        "-137" 
  #define MSG_DEVICE_ERROR_WRONG_ANSWER                         "-138"
  #define MSG_DEVICE_ERROR_EEPROM                               "-151"
#endif

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           VARIOUS DEFINES SECTION 
*/
// How much need to wait to get encoder stabilization
// 2000 microseconds
const uint32_t constEncoderStabilizationDelay                  = 2000UL; 

// How much need to wait to get ADC stabilization
// 1000 microseconds
const uint32_t constAdcStabilizationDelay                      = 1000UL; 

// On Leonardo, Micro and other ATmega32u4 boards wait to Serial Monitor ready for 5sec 
const uint32_t constSerialWaitTimeout                          = 5000UL;  
 

#endif // _ZABBUINO_TUNE_CONFIG_H_

