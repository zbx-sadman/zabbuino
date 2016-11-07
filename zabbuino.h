#ifndef Zabbuino_h
#define Zabbuino_h
/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
   
                                                             NETWORK MODULE SECTION
   
   Old releases of Arduino IDE can do not processing #includes inside #if pragmas (see NETWORK MODULE SECTION) and hangs on compiling or show errors
   If you use that release - comment all #includes, exclude your's module related block 

                                                              !!! ENC28J60 users !!!

    Please try to use https://github.com/ntruchsess/arduino_uip/tree/fix_errata12 brahch of UIPEthernet if your ENC28J60 seems freeze or loose connection.
   
    Tested on UIPEthernet v1.09
    
    When UIPEthernet's fix_errata12 brahch did not help to add stability, you can buy W5xxx shield.
    
    Also u can try uncomment USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE declaration below to periodically ENC28J60 re-intit if EIR.TXERIF and EIR.RXERIF == 1
*/

#define W5100_ETHERNET_SHIELD             // Arduino Ethernet Shield and Compatibles ...
//#define ENC28J60_ETHERNET_SHIELD          // Microchip __ENC28J60__ network modules

// not tested yet, but set up to use WizNet official library: https://github.com/Wiznet/WIZ_Ethernet_Library/tree/master/Arduino%20IDE%201.5.x/Ethernet
// Unfortunatly network chip selection can't be carried from outside of Wiznet library :(
// You must edit "%Program Files%\Arduino\libraries\Ethernet\src\utility\w5100.h" directly to comment 
//     and uncomment the same #defines - W5100_ETHERNET_SHIELD or W5500_ETHERNET_SHIELD or another else
//#define W5200_ETHERNET_SHIELD             // WIZ820io, W5200 Ethernet Shield 
//#define W5500_ETHERNET_SHIELD             // WIZ550io, ioShield series of WIZnet



/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 PROGRAMM FEATURES SECTION

                                  Comment #define's below to save RAM and Flash and uncomment to enable some feature 

*/

/****       New              ****/
// WS2812 led stripe support
//#define FEATURE_WS2812_ENABLE

// PZEM-004 energy monitor support
//#define FEATURE_PZEM004_ENABLE

// 
//#define FEATURE_UPS_APCSMART_ENABLE
//#define FEATURE_UPS_MEGATEC_ENABLE


/****       Network              ****/
/*/ 
/=/      Obtain an IP-address using DHCP
/*/
//#define FEATURE_NET_DHCP_ENABLE

/*/ 
/=/      Force obtain IP-address using DHCP even netConfig.useDHCP = false
/*/
//#define FEATURE_NET_DHCP_FORCE

/*/ 
/=/      Use last byte of MCU ID as MAC`s and IP's last byte 
/=/      Note, that changing MAC or IP-address separately may cause "strange" network errors until the moment when the router delete old ARP-records from the cache.
/*/
// need to test for ip rewriting
//#define FEATURE_NET_USE_MCUID

/****       Arduino wrappers     ****/
  
/*/ 
/=/      Enable commands:
/=/        - Tone[];
/=/        - NoTone[]
/=/      
/=/      Note, gatherSystemMetrics() subroutine that called by Timer1 interrupt if GATHER_METRIC_USING_TIMER_INTERRUPT macro is enabled conflicts with the tone() function -
/=/      the buzz stops for a short while. 
/=/      The reason for this seems is delayed Timer0(Timer2) interrupts that service tone() due getADCVoltage() subroutine that called from gatherSystemMetrics() run so long.
/=/      If you need loud and clear buzz's, at this time - you must disable GATHER_METRIC_USING_TIMER_INTERRUPT macro. I hope to fix it later.
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

// Note #1: I2C library (Wire.h) takes at least 32bytes of memory for internal buffers
// Note #2: I2C library (Wire.h) activate internal pullups for SDA & SCL pins when Wire.begin() called

/*/ 
/=/     Enable I2C processing and commands:
/=/       - I2C.Scan[];
/=/       - I2C.Write[];
/=/       - I2C.Read[];
/=/       - I2C.BitWrite[];
/=/       - I2C.BitRead[]
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
/=/     Note: BME280 is BMP280+Humidity sensor. If you want to get all, uncomment SUPPORT_BMP280_INCLUDE too.
/*/
//#define SUPPORT_BME280_INCLUDE

/*/ 
/=/     Enable ROHM BH1750 ambient light sensor handling and command:
/=/       - BH1750.Light[]
/=/ 
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
#define FEATURE_EEPROM_ENABLE

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
#define FEATURE_DEBUG_COMMANDS_ENABLE

/*/
/=/     View the debug messages on the Serial Monitor
/*/
//#define FEATURE_DEBUG_TO_SERIAL

/*/
/=/     View the additional debug messages on the Serial Monitor when network errors probaly occurs
/*/
//#define FEATURE_NET_DEBUG_TO_SERIAL

/*/
/=/     Recieve command from Serial Monitor too
/*/
//#define FEATURE_SERIAL_LISTEN_TOO

/*/
/=/     Use interrupt on Timer1 for internal metric gathering
/*/
// Need to try Phase Correct PWM mode - _BV(WGM13);
#define GATHER_METRIC_USING_TIMER_INTERRUPT

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                               ALARM SECTION 
*/

// Where is state LED connected
#define PIN_STATE_LED              	                        0x09
// State LED must blink or just be turned on?
#define ON_ALARM_STATE_BLINK            
// Use more blinks to runtime stage indication
//#define ADVANCED_BLINKING

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            NETWORK MODULE SECTION 

       Note, that changing MAC or IP-address separately may cause "strange" network errors until the moment when the router delete old ARP-records from the cache.

*/

#define NET_DEFAULT_USE_DHCP                                  false

#ifdef USE_NETWORK_192_168_0_0
  #define NET_DEFAULT_MAC_ADDRESS                             {0xDE,0xAD,0xBE,0xEF,0xFE,0xF9}
  #define NET_DEFAULT_IP_ADDRESS                              {192,168,0,228}
  #define NET_DEFAULT_GATEWAY                                 {192,168,0,1}
#else
  #define NET_DEFAULT_MAC_ADDRESS                              {0xDE,0xAD,0xBE,0xEF,0xFE,0xF7}
  #define NET_DEFAULT_IP_ADDRESS                              {172,16,100,228}
  #define NET_DEFAULT_GATEWAY                                 {172,16,100,254}
#endif
#define NET_DEFAULT_NETMASK                                   {255,255,255,0}


// Include headers for an network module
#if defined (W5100_ETHERNET_SHIELD)
   #define NET_MODULE_NAME                                   "W5100"
   #include <Ethernet.h> 
   #include <SPI.h>
#elif defined(W5200_ETHERNET_SHIELD)
   #define NET_MODULE_NAME                                   "W5200"
   #include <Ethernet.h>
   #include <SPI.h>
#elif defined(W5500_ETHERNET_SHIELD)
   #define NET_MODULE_NAME                                   "W5500"
   #include <Ethernet.h>
   #include <SPI.h>
#elif defined(ENC28J60_ETHERNET_SHIELD)
   #define NET_MODULE_NAME                                   "ENC28J60"
   #include <UIPEthernet.h>
/* You need to do one change in UIPEthernet\utility\Enc28J60Network.h before uncomment USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE:         
    *  private:
            ...    
            static uint8_t readReg(uint8_t address);  // << move its to __public__ section
            ...
             
         public: 
             ...
   */
   //#define USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE
#endif


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         SYSTEM CONFIGURATION SECTION 
*/

#define SYS_DEFAULT_PROTECTION      	                        true
// It's just number of "long int" type. Surprise!
#define SYS_DEFAULT_PASSWORD        	                        0x000

// Digital pin which must shorted on the GND for HOLD_TIME_TO_FACTORY_RESET time to save default system setting into EEPROM
#define PIN_FACTORY_RESET           	                        8 

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          AGENT CONFIGURATION SECTION 
*/

#define ZBX_AGENT_DEFAULT_HOSTNAME                            "zabbuino"
// Domain name used only to make FDQN if FEATURE_NET_USE_MCUID is allowed, and FEATURE_EEPROM_ENABLE is disabled
#define ZBX_AGENT_DEFAULT_DOMAIN                              ".local.net"


#define ZBX_AGENT_VERISON             	                      "Zabbuino 1.1.0"


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 HEADERS SECTION
*/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/boot.h>
// used by interrupts-related macroses
#include <wiring_private.h>

#include "src/platforms.h"
#include "src/defaults.h"

/* runtime libs */
#include "src/adc.h"
#include "src/eeprom.h"
#include "src/io_regs.h"
#include "src/system.h"

/* I2C devices */
#include "src/i2c_bus.h"
#include "src/i2c_bmp.h"
#include "src/i2c_lcd.h"
#include "src/i2c_sht.h"

/* 1-Wire devices */
#include "src/ow_bus.h"
#include "src/ow_sensors.h"

/* UART connected devices */
#include "src/uart_bus.h"
#include "src/uart_apcsmart.h"
#include "src/uart_megatec.h"
#include "src/uart_pzem.h"


/* Other devices */
#include "src/dht.h"
#include "src/ir.h"
#include "src/interrupts.h"
#include "src/ultrasonic.h"
#include "src/shiftout.h"
#include "src/busMicrowire.h"

#endif


