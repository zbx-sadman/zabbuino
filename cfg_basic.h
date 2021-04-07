#pragma once
#include <Arduino.h>

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

   Old releases of Arduino IDE can do not processing some compiler options and throw error compilation.
   Release 1.6.11 is okay.

   Go to NETWORK MODULE SECTION, that placed below to change local address / subnet / gateway


                                                     !!! ENC28J60 users !!!

   1. Please do not try to use 3.3V from Arduino board pin if you do not sure that it provide sufficient power.
      The likelihood of getting problems tends to 100%. Use additional external power source.
   2. Leave as much free memory as possible. If Arduino IDE show to you "Low memory available, stability problems may occur", it
      means that the chances of sudden and unpredictable hang your device are large.
   3. Sometime ENC28J60's RX buffer or configuration registry have corrupt and network module stops network processing. But Arduino board
      still live and make blink by State LED. Zabbuino will try detect this case and fix the problem.
   4. When (1) & (2) & (3) did not help to add stability, you can buy Wiznet 5xxx shield or rewrite the source code.

                >>> NOTE that network drivers (UIPEthernet & WIZNet libs) are integrated to Zabbuino source code <<<
*/


// AVR-based systems
#if defined(ARDUINO_ARCH_AVR)
#define NETWORK_ETHERNET_W5100           // Arduino Ethernet Shield and Compatible ...
//#define NETWORK_ETHERNET_ENC28J60      // Microchip __ENC28J60__ network module
//#define NETWORK_ETHERNET_W5500         // WIZ550io, ioShield series of WIZnet, tested but not satisfied with the performance on intensive traffic
//#define NETWORK_SERIAL_INTERFACE       // not implemented yet

// ESP8266-based systems
#elif (defined(ARDUINO_ARCH_ESP8266) || defined (ARDUINO_ARCH_ESP32))
#define NETWORK_WIRELESS_ESP_NATIVE      // ESP's internal WiFi module
#else
#error "Unknow architecture"
#endif

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 PROGRAMM FEATURES SECTION

                                  Comment #define's below to save RAM and Flash and uncomment to enable some feature

*/

/****       New              ****/
//
//      Obtain an IP-address using DHCP
//
//      Note: Do not forget to enable UDP prototol support for network module driver.
//
#define FEATURE_NET_DHCP_ENABLE

//
//      Force obtain IP-address using DHCP even netConfig.useDHCP = false
//
#define FEATURE_NET_DHCP_FORCE

//
//      MCU ID (in HEX) used as hostname, and last byte of MCU ID as IP's 4-th octet, and last 3 byte as MAC`s NIC speciific part (4,5,6 octets)
//
//      Note: Unique MCU ID defined for ATMega328PB and can not exist on other MCU's (but seems that it exists on my ATMega328P).
//            You need try to read it before use for network addresses generating.
//            Using only 3 last bytes not guarantee making unique MAC or IP.
//
#define FEATURE_NET_USE_MCUID

/****       Arduino wrappers     ****/

//      Enable commands:
//        - analogWrite[];
//        - digitalWrite[]
//        - analogRead[];
//        - digitalRead[]
//        - delay[]
//
//
#define FEATURE_ARDUINO_BASIC_ENABLE

//
//      Enable commands:
//        - Tone[];
//        - NoTone[]
//
//
//#define FEATURE_TONE_ENABLE

//
//      Enable commands:
//        - RandomSeed[];
//        - Random[]
//
#define FEATURE_RANDOM_ENABLE

//
//      Enable command:
//        - ShiftOut[]
//
#define FEATURE_SHIFTOUT_ENABLE

/****       Interrupts related   ****/

//
//      Enable external interrupts handling and commands:
//        - ExtInt.Count[]
//
#define FEATURE_EXTERNAL_INTERRUPT_ENABLE

//
//     Handle incremental encoder that connected to interrupt-pin and enable command:
//     - IncEnc.Value[]
//
//#define FEATURE_INCREMENTAL_ENCODER_ENABLE

/****      1-Wire bus      ****/
//
//     Enable 1-Wire processing and command:
//       - OW.Scan[]
//
#define FEATURE_OW_ENABLE

//
//     Enable Dallas DS18x20 sensors handling and command:
//       - DS18x20.Temperature[]
//
#define FEATURE_DS18X20_ENABLE

/****       I2C bus        ****/

//
//     Enable I2C processing and commands:
//       - I2C.Scan[];
//       - I2C.Write[];
//       - I2C.Read[];
//       - I2C.BitWrite[];
//       - I2C.BitRead[]
//
//
#define FEATURE_I2C_ENABLE

//
//     Enable BOSCH BMP180 sensors handling and commands:
//       - BMP.Pressure[];
//       - BMP.Temperature[]
//
//
#define FEATURE_BMP180_ENABLE

//
//     Enable BOSCH BMP280 sensors handling and commands:
//       - BMP.Pressure[];
//       - BMP.Temperature[]
//
//
#define FEATURE_BMP280_ENABLE

//
//     Enable BOSCH BME280 sensors handling and enable additional command
//       - BME.Humidity[]
//
//     Note: BME280 is BMP280+Humidity sensor. Temperature and pressure is can be taken with BMP.Temperature[] / BMP.Pressure[] commands
//
#define FEATURE_BME280_ENABLE

//
//     Enable ROHM BH1750 ambient light sensor handling and command:
//       - BH1750.Light[]
//
//     Note: To BH1750.Light command can be replaced with I2C.Read[....] with result multiplication by 0.83333.. (lux = raw / 1.2).
//     This replacement allow to save a little progmem space
//
#define FEATURE_BH1750_ENABLE

//
//      Enable MAX44009 support and commands:
//        - MAX44009.light[]
//
#define FEATURE_MAX44009_ENABLE

//
//      Enable TSL2561 support and commands:
//        - TSL2561.light[]
//
#define FEATURE_TSL2561_ENABLE

//
//      Enable VEML6070 support and commands:
//        - VEML6070.uv[]
//
#define FEATURE_VEML6070_ENABLE

//
//      Enable ADPS9960 support and commands:
//        - ADPS9960.ambient[]
//        - ADPS9960.red[]
//        - ADPS9960.green[]
//        - ADPS9960.blue[]
//
#define FEATURE_ADPS9960_ENABLE

//
//     Enable LCD that connected via PCF8574(A)-based I2C expander and command:
//       - PCF8574.LCDPrint[]
//
//
#define FEATURE_PCF8574_LCD_ENABLE

//
//     Enable Sensirion SHT2x sensors handling and commands:
//       - SHT2x.Humidity[];
//       - SHT2x.Temperature[]
//
#define FEATURE_SHT2X_ENABLE

//
//      Enable INA219 Zer0-Drift, Bidirectional Current/Power Monitor With I2C Interface support and commands:
//        - INA219.BusVoltage[]
//        - INA219.Current[]
//        - INA219.Power[]
//
#define FEATURE_INA219_ENABLE

//
//      Enable support external I2C EEPROM chip (AT24C family) and commands:
//        - AT24CXX.write[]
//        - AT24CXX.read[]
//
#define FEATURE_AT24CXX_ENABLE

//
//      Enable PCA9685 support and commands:
//        - PCA9685.write[]
//
#define FEATURE_PCA9685_ENABLE

//
//     Enable Sensirion SGP30 sensor support and command:
//       - sgp30.co2e[]
//       - sgp30.tvoc[]
//
#define FEATURE_SGP30_ENABLE

//
//     Enable Telaire T67XX sensors support and command:
//       - t67xx.i2c.co2[]
//
#define FEATURE_T67XX_ENABLE

//
//     Enable Melexis MLX90614 sensor support and command:
//       - mlx90614.temperature[]
//
#define FEATURE_MLX90614_ENABLE

//
//     Enable Wuhan Cubic PM2011 dust sensor support and command:
//       - WCPM.I2C.All[]
//
#define FEATURE_WUHAN_CUBIC_PM_I2C_ENABLE

/****       MicroWire bus       ****/
//
//     Enable MAX7219 with 8x8 LED matrix handling and command:
//       - MAX7219.Write[]
//
#define FEATURE_MAX7219_ENABLE

/****       UART bus       ****/
//
//     Enable PZEM-004 (non-Modbus release) energy monitor support and commands:
//       - pzem004.current[]
//       - pzem004.voltage[]
//       - pzem004.power[]
//       - pzem004.energy[]
//
#define FEATURE_PZEM004_ENABLE

//
//     Enable DFPlayer Mini support and command:
//       - DFPlayer.run[]
//
#define FEATURE_DFPLAYER_ENABLE

//
//     Enable APC SmartUPS protocol support and command:
//       - ups.apcsmart[]
//
#define FEATURE_UPS_APCSMART_ENABLE

//
//     Enable Megatec protocol support and command:
//       - ups.megatec[]
//
//     Note: command is not tested on real hardware. Please, send report to me.
//
#define FEATURE_UPS_MEGATEC_ENABLE

//
//     Enable Plantower PMS-xxxx sensors support and command:
//       - PMSxx.all[]
//       - PMSxx.epm[]
//       - PMSxx.fpm[]
//
//     Note: PMS.all command output is JSON (for Zabbix v3.4 and above)
//
#define FEATURE_PLANTOWER_PMSXX_ENABLE

//
//     Enable Nova Fitness SDSxxx sensors support and command:
//       - SDS.all[]
//       - SDS.epm[]
//
//     Note: SDS.all command output is JSON (for Zabbix v3.4 and above)
//
#define FEATURE_NOVA_FITNESS_SDS_ENABLE

//
//     Enable Wuhan Cubic PM2011 dust sensor support and command:
//       - WCPM.UART.All[]
//
#define FEATURE_WUHAN_CUBIC_PM_UART_ENABLE

/****       UART bus / Winsen sensors    ****/
//
//     Enable MH-Zxx family PWM/UART protocol support and command:
//       - mhzxx.pwm.co2[]
//       - mhzxx.uart.co2[]
//
//#define FEATURE_MHZXX_PWM_ENABLE
#define FEATURE_MHZXX_UART_ENABLE

//     Enable Winsen sensor:
// - ZE08-CH2O
// - ZE14-O3 (ZE25-O3, ZE25-O3);
// - ZP14 (Natural gas / Combustible Gas);
// - ZE15-CO;
// - ZE16-CO;
//  support and commands:
//       - ze08.ch2o[]
//       - ze14.o3[]
//       - zp14.ng[]
//       - ze15.co[]
//       - ze16.co[]
//
#define FEATURE_WINSEN_ZE08_CH2O_ENABLE
#define FEATURE_WINSEN_ZE14_O3_ENABLE
#define FEATURE_WINSEN_ZP14_ENABLE
#define FEATURE_WINSEN_ZE15_CO_ENABLE
#define FEATURE_WINSEN_ZE16_CO_ENABLE


/****       DHT/AM family    ****/

//
//     Enable DHT/AM humidity sensors handling and commands:
//       - DHT.Humidity[];
//       - DHT.Temperature[]
//       - DHT.All[]
//
#define FEATURE_DHT_ENABLE

/****       Ultrasonic    ****/

//
//     Enable HC-SR04 sensor and command:
//       - Ultrasonic.Distance[];
//
#define FEATURE_ULTRASONIC_ENABLE

/****       InfraRed transmitters emulation    ****/
//
//     Enable commands:
//       - IR.Send[];
//       - IR.SendRAW[]
//
//     Note: See below to include special signal types supporting
//
//     Compability: AVR
//
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
//
//     Enable WS281x (2811,2812,2813) led chip support and command:
//       - ws281x.sendraw[]
//
#define FEATURE_WS281x_ENABLE


/****      Unsorted features     ****/
//
//     Enable MAX6675 support and command:
//       - MAX6675.temperature[]
//
#define FEATURE_MAX6675_ENABLE


//
//     Enable digital Servo's (like SG-90) support and command:
//       - Servo.turn[]
//
#define FEATURE_SERVO_ENABLE

//
//     Enable various turn on / turn off tricks support and command:
//       - relay[]
//       - pulse[]
//
#define FEATURE_RELAY_ENABLE

/****       System        ****/
//
//      Enable calling user functions on device start and every _constUserFunctionCallInterval_ if no active network session exist
//      You can write to _plugin.ino_ your own code and use all Zabbuino's internal functions to query sensors and handle actuators
//
#define FEATURE_USER_FUNCTION_PROCESSING

//
//      Support Zabbix's Action functionality and enable command:
//        - system.run[]
//
#define FEATURE_REMOTE_COMMANDS_ENABLE

//
//     Enable watchdog
//
//     AVR Note:
//     NOT ALL BOOTLOADERS HANDLE WATCHDOG PROPERLY: http://forum.arduino.cc/index.php?topic=157406.0
//
//     Note: OptiBoot is watchdog compatible and use less flash space that stock bootloader.
//     Note: watchdog timeout may be vary for many controllers, see comments to macro WTD_TIMEOUT in zabbuino.h
//
//     Compability: AVR, ESP8266, ESP32
//
#define FEATURE_WATCHDOG_ENABLE

//
//     Use analogReference() function in analogread[] and acs7xx.*[] commands
//
//     Uncomment ***only*** if you know all about AREF pin using ***risks***
//
// // // // #define FEATURE_AREF_ENABLE

//
//     Store settings to EEPROM and use its on start
//
#define FEATURE_EEPROM_ENABLE

//
//     Force protect (enable even netConfig.useProtection is false) your system from illegal access for change runtime settings and reboots
//
//#define FEATURE_PASSWORD_PROTECTION_FORCE

//
//     Enable commands which returns system information and can be used in system debug process:
//       - System.HW.CPU[];
//       - System.HW.Chassis[];
//       - Net.PHY.Name[];
//       - Net.PHY.Reinits[];
//       - Sys.Cmd.Count[];
//       - Sys.Cmd.TimeMax[];
//       - Sys.RAM.Free[];
//       - Sys.RAM.FreeMin[]
//
#define FEATURE_SYSINFO_ENABLE

//     Enable commands which returns system information and can be used in system debug process:
//       - Sys.All;
#define FEATURE_SYSINFO_ALL_ENABLE

//
//      Enable support the system RTC (DS3231 or PCF8563 RTC chip which connected via I2C interface) and commands:
//        - set.localtime[]
//        - system.localtime[]
//
//      Refer to SYSTEM HARDWARE SECTION in src/cfg_tune.h
//
//      Compability: AVR
//
//#define FEATURE_SYSTEM_RTC_DS3231_ENABLE
//#define FEATURE_SYSTEM_RTC_PCF8563_ENABLE

//
//    View the more or less debug messages on the Serial Monitor. 0 - no messages .. 3 - max
//
#define FEATURE_DEBUG_MESSAGING_LEVEL     2

//    Various runtime info for development needs
//#define FEATURE_DEBUG_TO_SERIAL_DEV

//
//     Recieve command from Serial Monitor too. Do not forget to enable one of FEATURE_DEBUG_TO_SERIAL_* macro
//
//     Note that 64 bytes buffer reserved for Serial (refer to HardwareSerial.h) and long commands like ws2812.sendraw[5,1,over9000chars] can be processed uncorrectly
//
#define FEATURE_SERIAL_LISTEN_TOO

//
//     Send back to user text messages if error is occurs. Otherwise - send numeric code
//
#define USE_TEXT_ERROR_MESSAGES

//
//     Use interrupt on Timer1 for internal metric gathering
//
//#define GATHER_METRIC_USING_TIMER_INTERRUPT

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            NETWORK MODULE SECTION

       Note: MAC or IP-address separately changing may cause "strange" network errors until the moment when the router delete old ARP-records from the cache.

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
const uint8_t constDefaultMacAddress[0x06]             PROGMEM = {0xBE, 0xAD, 0xEB, 0xA8, 0x00, 0xDE};
const uint8_t constDefaultIPAddress[0x04]              PROGMEM = {192, 168, 0, 123};
const uint8_t constDefaultGateway[0x04]                PROGMEM = {192, 168, 0, 1};
const uint8_t constDefaultNetmask[0x04]                PROGMEM = {255, 255, 255, 0};

const char* const constWiFiSsid                                = "YourAP";
const char* const constWiFiPsk                                 = "YourPSK";

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  ALARM SECTION
*/

// Where is state LED connected
#if defined(ARDUINO_ARCH_AVR)
const uint8_t constStateLedPin                                 = 0x09;
const uint8_t constStateLedOn                                  = HIGH;

// It is optional define for multiarch/multidevice debug processing
#elif defined(ARDUINO_ARCH_ESP8266)
const uint8_t constStateLedPin                                 = D4;
// Some ESP-based (Wemos D1 mini, for example) boards have inverted "bulit led"
const uint8_t constStateLedOn                                  = LOW;
#elif defined(ARDUINO_ARCH_ESP32)
const uint8_t constStateLedPin                                 = 0x02;
const uint8_t constStateLedOn                                  = HIGH;
#endif
// State LED must blink or just be turned on?
#define ON_ALARM_STATE_BLINK
// Use more blinks to runtime stage indication
//#define ADVANCED_BLINKING

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  SYSTEM CONFIGURATION SECTION
*/

#define DEBUG_PORT                                             Serial

// Debug serial port speed in baud
const uint32_t constSerialMonitorSpeed                         = 115200;
//const uint32_t constSerialMonitorSpeed                         = 74880;

// Access password must be used anytime.
const uint8_t constSysDefaultProtection                        = true;

// It's just number of "long int" type. Surprise!
const uint32_t constSysDefaultPassword                         = 0x00;

// Digital pin which must shorted on the GND for constHoldTimeToFactoryReset time to copy default system setting into EEPROM
#if defined(ARDUINO_ARCH_AVR)
const uint8_t constUserFunctionButtonPin                       = 0x08;
const uint8_t constUserFunctionButtonActiveState               = LOW;

// It is optional define for multiarch/multidevice debug processing
#elif defined(ARDUINO_ARCH_ESP8266)
const uint8_t constUserFunctionButtonPin                       = D7;
const uint8_t constUserFunctionButtonActiveState               = LOW;
#elif defined(ARDUINO_ARCH_ESP32)
const uint8_t constUserFunctionButtonPin                       = 26;
const uint8_t constUserFunctionButtonActiveState               = LOW;
#endif

// Time to wait network adapter ready state after reinit action
#if defined(ARDUINO_ARCH_AVR)
const uint32_t constNetworkInfoShowDelay                       = 1000;
#elif (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))
const uint32_t constNetworkInfoShowDelay                       = 5000;
#endif

// How much "User Function" button must be active/unactive before userFunctionButtonActivate() / userFunctionButtonDeactivate() will be called
const uint32_t constUserFunctionButtonDebounceTime             = 5000;

//
const uint16_t constSysTZOffset                                = 0x2A30; // 10800 sec, 3h;

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                          AGENT CONFIGURATION SECTION
*/

const uint16_t constZbxAgentTcpPort                            = 10050;

const char constZbxAgentDefaultHostname[]            PROGMEM   = "zabbuino";

// Domain name used only to make FDQN if FEATURE_NET_USE_MCUID is allowed, and FEATURE_EEPROM_ENABLE is disabled
const char constZbxAgentDefaultDomain[]              PROGMEM   = ".local.net";


const char constZbxAgentVersion[]                    PROGMEM   = "Zabbuino 1.5.0";
