#ifndef _ZABBUINO_BASIC_CONFIG_H_
#define _ZABBUINO_BASIC_CONFIG_H_
#include <Arduino.h>
#include <IPAddress.h>
#define USE_NETWORK_192_168_0_0

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
   
   Old releases of Arduino IDE can do not processing #includes inside #if pragmas and stops on compiling or show errors.
   Release 1.6.11 is okay.
                                                               
   Go to NETWORK MODULE SECTION, that placed below to change local address / subnet / gateway


                                                     !!! ENC28J60 users !!!

   1. Please do not try to get 3.3V from Arduino board pin if you do not sure that it provide sufficient power. 
      The likelihood of getting problems tends to 100%. Use additional external power source.
   2. Leave as much free memory as possible. If Arduino IDE show to you "Low memory available, stability problems may occur", it 
      means that the chances of sudden and unpredictable hang your device are large. 
   3. Sometime ENC28J60's RX buffer or configuration registry have corrupt and network module stops network processing. But Arduino board
      still live and make blink by State LED. 
      You can enable FEATURE_NETWORK_MONITORING declaration to let the Zabbuino detect this case and try to fix it.
   4. When (1) & (2) & (3) did not help to add stability, you can buy Wiznet 5xxx shield or rewrite the source code.
   
                >>> NOTE that network drivers (UIPEthernet & WIZNet libs) are integrated to Zabbuino source code <<<
*/

#define W5100_ETHERNET_SHIELD       // Arduino Ethernet Shield and Compatibles ...
//#define ENC28J60_ETHERNET_SHIELD      // Microchip __ENC28J60__ network modules
//#define W5200_ETHERNET_SHIELD       // WIZ820io, W5200 Ethernet Shield , not tested yet
//#define W5500_ETHERNET_SHIELD       // WIZ550io, ioShield series of WIZnet , tested but not satisfied with the performance on intensive traffic

/*/ 
/=/      Let Zabbuino to detect network module errors, and try to fix it.
/=/      
/=/      Note: This functionality realised for ENC28J60 only.
/*/
//#define FEATURE_NETWORK_MONITORING

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 PROGRAMM FEATURES SECTION

                                  Comment #define's below to save RAM and Flash and uncomment to enable some feature 

*/

/**** NEW ****/
/*/ 
/=/      Enable INA219 Zer0-Drift, Bidirectional Current/Power Monitor With I2C Interface support and commands:
/=/        - INA219.BusVoltage[]
/=/        - INA219.Current[]
/=/        - INA219.Power[]
/*/
//#define FEATURE_INA219_ENABLE

/*/ 
/=/      Enable support the user display (LCD which connected via I2C interface) 
/=/      You must build it manually by example virtual screen #1 in plugin.ino subroutine 
/*/
//#define FEATURE_USER_DISPLAY_ENABLE

//#define FEATURE_USER_FUNCTION_PROCESSING

/*/ 
/=/      Enable support the system RTC (DS3231 RTC chip which connected via I2C interface) and commands:
/=/        - set.localtime[]
/=/        - system.localtime[]
/=/      
/=/      Refer to SYSTEM HARDWARE SECTION in src\tune.h
/*/
//#define FEATURE_SYSTEM_RTC_ENABLE


/*/ 
/=/      Enable command: 
/=/        - system.run[]
/*/
//#define FEATURE_REMOTE_COMMANDS_ENABLE

/*/ 
/=/      Enable support I2C connected EEPROM chip (AT24C family) and commands:
/=/        - AT24CXX.write[]
/=/        - AT24CXX.read[]
/=/      
/*/
//#define FEATURE_AT24CXX_ENABLE


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
/=/      Use last byte of MCU ID as IP's 4-th octet, and last 3 byte as MAC`s NIC speciific part (4,5,6 octets) 
/=/      
/=/      Note: Unique MCU ID defined for ATMega328PB and can not exist on other MCU's (but seems that it exists on my ATMega328P). 
/=/            You need try to read it before use for network addresses generating.
/=/            Using only 3 last bytes not guarantee making unique MAC or IP.
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
#define FEATURE_OW_ENABLE

/*/ 
/=/     Enable Dallas DS18x20 sensors handling and command:
/=/       - DS18x20.Temperature[]
/*/

#define FEATURE_DS18X20_ENABLE

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
/=/     Enable BOSCH BMP180 sensors handling and commands:
/=/       - BMP.Pressure[];
/=/       - BMP.Temperature[]
/=/ 
/*/
#define FEATURE_BMP180_ENABLE

/*/ 
/=/     Enable BOSCH BMP280 sensors handling 
/*/ 
// #define FEATURE_BMP280_ENABLE

/*/ 
/=/     Enable BOSCH BME280 sensors handling and enable additional command
/=/       - BME.Humidity[]
/=/
/=/     Note: BME280 is BMP280+Humidity sensor. Temperature and pressure is can be taken with BMP.Temperature[] / BMP.Pressure[] commands
/*/
//#define FEATURE_BME280_ENABLE

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
#define FEATURE_DHT_ENABLE

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
#define FEATURE_EEPROM_ENABLE

/*/
/=/     Force protect (enable even netConfig.useProtection is false) your system from illegal access for change runtime settings and reboots 
/*/
//#define FEATURE_PASSWORD_PROTECTION_FORCE

/*/
/=/     Enable commands which returns system information and can be used in system debug process:
/=/       - System.HW.CPU[];
/=/       - System.HW.Chassis[];
/=/       - Sys.MCU.ID[];
/=/       - Sys.PHY.Module[];
/=/       - Sys.Cmd.Count[];
/=/       - Sys.Cmd.TimeMax[];
/=/       - Sys.RAM.Free[];
/=/       - Sys.RAM.FreeMin[]
/*/
#define FEATURE_SYSINFO_ENABLE

/*/
/=/     View the more or less debug messages on the Serial Monitor. Choose one.
/*/
//#define FEATURE_DEBUG_TO_SERIAL_LOW
//#define FEATURE_DEBUG_TO_SERIAL_MIDDLE
//#define FEATURE_DEBUG_TO_SERIAL_HIGH
//#define FEATURE_DEBUG_TO_SERIAL_DEV

/*/
/=/     View the additional debug messages on the Serial Monitor when network errors probaly occurs
/*/
//#define FEATURE_NETWORK_MONITORING

/*/
/=/     Recieve command from Serial Monitor too. Do not forget to enable one of FEATURE_DEBUG_TO_SERIAL_* macro 
/*/
//#define FEATURE_SERIAL_LISTEN_TOO

/*/
/=/     Send back to user text messages if error is occurs. Otherwise - send numeric code
/*/
//#define USE_TEXT_ERROR_MESSAGES

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

/*
   Universally administered and locally administered addresses are distinguished by setting the second-least-significant bit of the first octet of the address. 
   This bit is also referred to as the U/L bit, short for Universal/Local, which identifies how the address is administered. If the bit is 0, the address is universally 
   administered. If it is 1, the address is locally administered. In the example address 06-00-00-00-00-00 the first octet is 06 (hex), the binary form of which
   is 00000110, where the second-least-significant bit is 1. Therefore, it is a locally administered address.[7] Consequently, this bit is 0 in all OUIs.
   https://en.wikipedia.org/wiki/MAC_address



   Note: changing MAC or IP-address separately may cause "strange" network errors until the moment when the router flush ARP cache.
*/

// Zabbuino's IP address
#ifdef USE_NETWORK_192_168_0_0
  #define NET_DEFAULT_MAC_ADDRESS                              {0xBE,0xAD,0xEB,0xA8,0x00,0xDD}
  #define NET_DEFAULT_IP_ADDRESS                               {192,168,0,221}
  #define NET_DEFAULT_GATEWAY                                  {192,168,0,1}
  #define NET_DEFAULT_NETMASK                                  {255,255,255,0}
#else
  #define NET_DEFAULT_MAC_ADDRESS                              {0xBE,0xAD,0xEB,0x10,0x64,0xDC}
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

// Debug serial port speed in baud
const uint16_t constSerialMonitorSpeed                          = 9600; 

// Access password must be used anytime.
const uint8_t constSysDefaultProtection                        = true; 

// It's just number of "long int" type. Surprise!
const uint32_t constSysDefaultPassword                         = 0x000; 

// Digital pin which must shorted on the GND for constHoldTimeToFactoryReset time to copy default system setting into EEPROM
const uint8_t constFactoryResetButtonPin                       = 0x08; 

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          AGENT CONFIGURATION SECTION 
*/

#define ZBX_AGENT_TCP_PORT 10050

#define ZBX_AGENT_DEFAULT_HOSTNAME                             "zabbuino"

// Domain name used only to make FDQN if FEATURE_NET_USE_MCUID is allowed, and FEATURE_EEPROM_ENABLE is disabled
#define ZBX_AGENT_DEFAULT_DOMAIN                               ".local.net"


const char constZbxAgentVersion[] PROGMEM =                   "Zabbuino 1.2.0";
#endif // #ifndef _ZABBUINO_BASIC_CONFIG_H_

