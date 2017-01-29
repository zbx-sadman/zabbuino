#ifndef _ZABBUINO_BASIC_CONFIG_H_
#define _ZABBUINO_BASIC_CONFIG_H_
#include <Arduino.h>
#include <IPAddress.h>
#define USE_NETWORK_192_168_0_0

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
   
                                                             NETWORK MODULE SECTION
   
   Old releases of Arduino IDE can do not processing #includes inside #if pragmas (see NETWORK MODULE SECTION) and stops on compiling or show errors
   If you use that release - comment all #includes, exclude your's module related block.

                                                                      Wizet users

    WizNet official library: https://github.com/Wiznet/WIZ_Ethernet_Library/tree/master/Arduino%20IDE%201.5.x/Ethernet can be used with Zabbuino
    Unfortunatly network chip selection can't be carried from outside of Wiznet library :(
    You must edit "%Program Files%\Arduino\libraries\Ethernet\src\utility\w5100.h" directly to comment and uncomment the 
    same #defines - W5100_ETHERNET_SHIELD or W5500_ETHERNET_SHIELD or another else.
                                                              
                                                              !!! ENC28J60 users !!!

   1. Please try to use https://github.com/ntruchsess/arduino_uip/tree/fix_errata12 brahch of UIPEthernet if your ENC28J60 seems freeze or loose connection.
   2. Leave as much free memory as possible. If Arduino IDE show to you "Low memory available, stability problems may occur", it means that the chances 
      of sudden and unpredictable hang your device are large. You can get more memory if disable DHCP feature for Zabbuino and disabe UDP protocol support
      for UIPEthernet (uipethernet-conf.h -> #define UIP_CONF_UDP 0). 
      Also, you can increase number of sockets for UIPEthernet (uipethernet-conf.h -> #define #define UIP_CONF_MAX_CONNECTIONS .. ).
   3. Sometime ENC28J60's RX buffer or configuration registry have corrupt (i don't know how, experiments will go on. May be it my algo error) and 
      chip re-init is enough to recovery work state. You can enable USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE declaration to re-intit 
      ENC28J60 when corruption is detected. 
      Note that you need to move the _readReg(uint8_t address)_ declaration from **private** to **public** area in UIPEthernet\utility\Enc28J60Network.h file.

   4. When (1) & (2) & (3) did not help to add stability, you can buy Wiznet 5xxx shield or rewrite the source code.
   
   Tested on UIPEthernet v1.09


*/

//#define W5100_ETHERNET_SHIELD             // Arduino Ethernet Shield and Compatibles ...
#define ENC28J60_ETHERNET_SHIELD          // Microchip __ENC28J60__ network modules
//#define USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE

//#define W5200_ETHERNET_SHIELD             // WIZ820io, W5200 Ethernet Shield , not tested yet
//#define W5500_ETHERNET_SHIELD             // WIZ550io, ioShield series of WIZnet , not tested yet



/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 PROGRAMM FEATURES SECTION

                                  Comment #define's below to save RAM and Flash and uncomment to enable some feature 

*/

/* NEW */
/*/ 
/=/      Enable INA219 Zer0-Drift, Bidirectional Current/Power Monitor With I2C Interface support and commands:
/=/        - INA219.BusVoltage[]
/=/        - INA219.Current[]
/=/        - INA219.Power[]
/*/
//#define FEATURE_EXTERNAL_INTERRUPT_ENABLE
#define FEATURE_INA219_ENABLE
 

/****       Network              ****/
/*/ 
/=/      Obtain an IP-address using DHCP
/=/      
/=/      Note: Do not forget to enable UDP prototol support for network module driver.
/*/
//#define FEATURE_NET_DHCP_ENABLE

/*/ 
/=/      Force obtain IP-address using DHCP even netConfig.useDHCP = false
/*/
//#define FEATURE_NET_DHCP_FORCE

/*/ 
/=/      Use last byte of MCU ID as MAC`s and IP's last byte 
/=/      
/=/      Note: changing MAC or IP-address separately may cause "strange" network errors until the moment when the router delete 
/=/      old ARP-records from the cache.
/*/
//#define FEATURE_NET_USE_MCUID

/****       Arduino wrappers     ****/
  
/*/ 
/=/      Enable commands:
/=/        - Tone[];
/=/        - NoTone[]
/=/      
/*/
//#define FEATURE_TONE_ENABLE


/*/ 
/=/      Enable commands:
/=/        - RandomSeed[]; 
/=/        - Random[]
/*/
//#define FEATURE_RANDOM_ENABLE

/*/ 
/=/      Enable command:
/=/        - ShiftOut[]
/*/
//#define FEATURE_SHIFTOUT_ENABLE

/****       Interrupts related   ****/

/*/ 
/=/      Enable external interrupts handling and commands:
/=/        - ExtInt.Count[]
/*/
//#define FEATURE_EXTERNAL_INTERRUPT_ENABLE

/* 
/=/     Handle incremental encoder that connected to interrupt-pin and enable command:
/=/     - IncEnc.Value[]
/*/
//#define FEATURE_INCREMENTAL_ENCODER_ENABLE

/****      1-Wire bus      ****/

/*/ 
/=/     Enable 1-Wire processing and command:
/=/       - OW.Scan[]
/*/
//#define FEATURE_OW_ENABLE

/*/ 
/=/     Enable Dallas DS18x20 sensors handling and command:
/=/       - DS18x20.Temperature[]
/*/

//#define FEATURE_DS18X20_ENABLE

/****       I2C bus        ****/

/*/ 
/=/     Enable I2C processing and commands:
/=/       - I2C.Scan[];
/=/       - I2C.Write[];
/=/       - I2C.Read[];
/=/       - I2C.BitWrite[];
/=/       - I2C.BitRead[]
/=/
/=/ Note #1: I2C library (Wire.h) takes at least 32bytes of memory for internal buffers
/=/ Note #2: I2C library (Wire.h) activate internal pullups for SDA & SCL pins when Wire.begin() called
/*/
#define FEATURE_I2C_ENABLE

/*/ 
/=/     Enable BOSCH BMP sensors handling and commands:
/=/       - BMP.Pressure[];
/=/       - BMP.Temperature[]
/=/ 
/=/     Note: See below to specify BMP model
/=/
/*/
//#define FEATURE_BMP_ENABLE

/*/ 
/=/     BMP180 / BMP085
/*/
//#define SUPPORT_BMP180_INCLUDE

/*/ 
/=/     BMP280
/*/
//#define SUPPORT_BMP280_INCLUDE

/*/ 
/=/     BME280 and enable command:
/=/       - BME.Humidity[]
/=/
/=/     Note: BME280 is BMP280+Humidity sensor. Temperature and pressure is can be taken with BMP.Temperature[] / BMP.Pressure[] commands
/*/
//#define SUPPORT_BME280_INCLUDE

/*/ 
/=/     Enable ROHM BH1750 ambient light sensor handling and command:
/=/       - BH1750.Light[]
/=/ 
/=/     Note: To BH1750.Light command can be replaced with I2C.Read[....] with result multiplication by 0.83333.. (lux = raw / 1.2). 
/=/     This replacement allow to save a little progmem space
/*/
//#define FEATURE_BH1750_ENABLE

/*/ 
/=/     Enable LCD that connected via PCF8574(A)-based I2C expander and command:
/=/       - PCF8574.LCDPrint[]
/=/ 
/*/
//#define FEATURE_PCF8574_LCD_ENABLE

/*/ 
/=/     Enable Sensirion SHT2x sensors handling and commands:
/=/       - SHT2x.Humidity[];
/=/       - SHT2x.Temperature[]
/*/
//#define FEATURE_SHT2X_ENABLE

/****       MicroWire bus       ****/
/*/ 
/=/     Enable MAX7219 with 8x8 LED matrix handling and command:
/=/       - MAX7219.Write[]
/=/
/*/
//#define FEATURE_MAX7219_ENABLE

/****       UART bus       ****/
/*/ 
/=/     Enable PZEM-004 energy monitor support and commands:
/=/       - pzem004.current[]
/=/       - pzem004.voltage[]
/=/       - pzem004.power[]  
/=/       - pzem004.energy[] 
/*/
//#define FEATURE_PZEM004_ENABLE

/*/ 
/=/     Enable APC SmartUPS protocol support and command:
/=/       - ups.apcsmart[]
/*/
//#define FEATURE_UPS_APCSMART_ENABLE

/*/ 
/=/     Enable Megatec protocol support and command:
/=/       - ups.megatec[]
/=/     
/=/     Note: command is not tested on real hardware. Please, send report to me.
/*/
//#define FEATURE_UPS_MEGATEC_ENABLE

/****       DHT/AM family    ****/

/*/
/=/     Enable DHT/AM humidity sensors handling and commands:
/=/       - DHT.Humidity[];
/=/       - DHT.Temperature[]
/*/
//#define FEATURE_DHT_ENABLE

/****       Ultrasonic    ****/

/*/ 
/=/     Enable HC-SR04 sensor and command:
/=/       - Ultrasonic.Distance[];
/*/
//#define FEATURE_ULTRASONIC_ENABLE

/****       ACS7XX family    ****/

/*/
/=/     Enable Allegro ACS7XX current sensor and commands:
/=/       - ACS7XX.ZC[];
/=/       - ACS7XX.AC[]
/*/
//#define FEATURE_ACS7XX_ENABLE

/****       InfraRed transmitters emulation    ****/
/*/ 
/=/     Enable commands:
/=/       - IR.Send[];
/=/       - IR.SendRAW[]
/=/
/=/     Note: See below to include special signal types supporting
/*/
//#define FEATURE_IR_ENABLE
//#define SUPPORT_IR_RC5
//#define SUPPORT_IR_RC6
//#define SUPPORT_IR_SONY
//#define SUPPORT_IR_NEC
//#define SUPPORT_IR_SAMSUNG
//#define SUPPORT_IR_WHYNTER
//#define SUPPORT_IR_LG
//#define SUPPORT_IR_DISH
//#define SUPPORT_IR_SHARP
//#define SUPPORT_IR_DENON

/****      Led Pixel modules     ****/
/*/ 
/=/     Enable WS2812 led chip support and command:
/=/       - ws2812.sendraw[]
/*/
//#define FEATURE_WS2812_ENABLE


/****       System        ****/

/*/ 
/=/     Enable AVR watchdog
/=/     !!! BEWARE !!!
/=/     NOT ALL BOOTLOADERS HANDLE WATCHDOG PROPERLY: http://forum.arduino.cc/index.php?topic=157406.0 
/=/ 
/=/     Note: OptiBoot is watchdog compatible and use less flash space that stock bootloader.
/=/     Note: watchdog timeout may be vary for many controllers, see comments to macro WTD_TIMEOUT in zabbuino.h
/=/
/*/
//#define FEATURE_WATCHDOG_ENABLE


/*/
/=/     Use analogReference() function in analogread[] and acs7xx.*[] commands
/=/
/=/     Uncomment ***only*** if you know all about AREF pin using ***risks***
/*/
// // // // #define FEATURE_AREF_ENABLE

/*/
/=/     Store runtime settings in EEPROM and use its on start
/*/
//#define FEATURE_EEPROM_ENABLE

/*/
/=/     Force protect (enable even netConfig.useProtection is false) your system from illegal access for change runtime settings and reboots 
/*/
//#define FEATURE_PASSWORD_PROTECTION_FORCE

/*/
/=/     Enable system's command which can be used in system debug process:
/=/       - Sys.MCU.Name[];
/=/       - Sys.MCU.ID[];
/=/       - Sys.Net.Module[];
/=/       - Sys.Cmd.Count[];
/=/       - Sys.Cmd.TimeMax[];
/=/       - Sys.RAM.Free[];
/=/       - Sys.RAM.FreeMin[]
/*/
//#define FEATURE_DEBUG_COMMANDS_ENABLE

/*/
/=/     View the more or less debug messages on the Serial Monitor. Choose one.
/*/
//#define FEATURE_DEBUG_TO_SERIAL_LOW
#define FEATURE_DEBUG_TO_SERIAL_MIDDLE
//#define FEATURE_DEBUG_TO_SERIAL_HIGH
//#define FEATURE_DEBUG_TO_SERIAL_DEV

/*/
/=/     View the additional debug messages on the Serial Monitor when network errors probaly occurs
/*/
//#define FEATURE_NET_DEBUG_TO_SERIAL

/*/
/=/     Recieve command from Serial Monitor too. Do not forget to enable one of FEATURE_DEBUG_TO_SERIAL_* macro 
/*/
//#define FEATURE_SERIAL_LISTEN_TOO

/*/
/=/     Send back to user text messages if error is occurs. Otherwise - send numeric code
/*/
#define USE_TEXT_ERROR_MESSAGES

/*/
/=/     Use interrupt on Timer1 for internal metric gathering
/*/
// Need to try Phase Correct PWM mode - _BV(WGM13);
#define GATHER_METRIC_USING_TIMER_INTERRUPT

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            NETWORK MODULE SECTION 

       Note, that changing MAC or IP-address separately may cause "strange" network errors until the moment when the router delete old ARP-records from the cache.

*/

const uint8_t constNetDefaultUseDHCP = false;

// Zabbuino's IP address
#ifdef USE_NETWORK_192_168_0_0
  #define NET_DEFAULT_MAC_ADDRESS                              {0xDE,0xAD,0xBE,0xEF,0xFE,0xF0}
  #define NET_DEFAULT_IP_ADDRESS                               {192,168,0,220}
  #define NET_DEFAULT_GATEWAY                                  {192,168,0,1}
  #define NET_DEFAULT_NETMASK                                  {255,255,255,0}
#else
  #define NET_DEFAULT_MAC_ADDRESS                              {0xDE,0xAD,0xBE,0xEF,0xFE,0xF0}
  #define NET_DEFAULT_IP_ADDRESS                               {172,16,100,220}
  #define NET_DEFAULT_GATEWAY                                  {172,16,100,254}
  #define NET_DEFAULT_NETMASK                                  {255,255,255,0}
#endif

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                               ALARM SECTION 
*/

// Where is state LED connected
const uint8_t constStateLedPin                                 = 0x09; 
// State LED must blink or just be turned on?
#define ON_ALARM_STATE_BLINK            
// Use more blinks to runtime stage indication
//#define ADVANCED_BLINKING

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         SYSTEM CONFIGURATION SECTION 
*/

// Access password must be used anytime.
const uint8_t constSysDefaultProtection                        = true; 

// It's just number of "long int" type. Surprise!
const uint32_t constSysDefaultPassword                         = 0x000; 

// Digital pin which must shorted on the GND for constHoldTimeToFactoryReset time to copy default system setting into EEPROM
const uint8_t constFactoryResetButtonPin                       = 0x08; 

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          AGENT CONFIGURATION SECTION 
*/

#define ZBX_AGENT_DEFAULT_HOSTNAME                             "zabbuino"

// Domain name used only to make FDQN if FEATURE_NET_USE_MCUID is allowed, and FEATURE_EEPROM_ENABLE is disabled
#define ZBX_AGENT_DEFAULT_DOMAIN                               ".local.net"


#define ZBX_AGENT_VERISON                                      "Zabbuino 1.1.1"
#endif // #ifndef _ZABBUINO_BASIC_CONFIG_H_

