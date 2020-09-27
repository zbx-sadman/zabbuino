#pragma once

#if defined (ARDUINO_ARCH_ESP8266) 
    #define _CPU_NAME_ "ESP8266EX"
    #if defined(ARDUINO_ESP8266_GENERIC)
        #define BOARD "Generic ESP8266 Module"
    #elif defined(ARDUINO_ESP8266_ESP01)
        #define BOARD "Generic ESP8285 Module"
    #elif defined(ARDUINO_ESP8266_ESP13)
        #define BOARD "ESPDuino / "
    #elif defined(ARDUINO_ESP8266_ESP12)
        #define BOARD "Adafruit Feather HUZZAH"
    #elif defined(ARDUINO_ESP8266_ESPRESSO_LITE_V1)
        #define BOARD "ESPresso Lite 1.0"
    #elif defined(ARDUINO_ESP8266_ESPRESSO_LITE_V2)
        #define BOARD "ESPresso Lite 2.0"
    #elif defined(ARDUINO_ESP8266_PHOENIX_V1)
        #define BOARD "Phoenix 1.0"
    #elif defined(ARDUINO_ESP8266_PHOENIX_V2)
        #define BOARD "Phoenix 2.0"
    #elif defined(ARDUINO_ESP8266_NODEMCU)
        #define BOARD "NodeMCU"
    #elif defined(ARDUINO_MOD_WIFI_ESP8266)
        #define BOARD "Olimex MOD-WIFI-ESP8266"
    #elif defined(ARDUINO_ESP8266_THING)
        #define BOARD "SparkFun ESP8266 Thing"
    #elif defined(ARDUINO_ESP8266_THING_DEV)
        #define BOARD "SparkFun ESP8266 Thing Dev"
    #elif defined(ARDUINO_ESP8266_ESP210)
        #define BOARD "SweetPea ESP-210"
    #elif defined(ARDUINO_ESP8266_WEMOS_D1MINI)
        #define BOARD "Wemos D1 MINI"
    #elif defined(ARDUINO_ESP8266_WEMOS_D1MINIPRO)
        #define BOARD "Wemos D1 MINI Pro"
    #elif defined(ARDUINO_ESP8266_WEMOS_D1MINILITE)
        #define BOARD "Wemos D1 MINI Lite"
    #elif defined(ARDUINO_ESP8266_WEMOS_D1R1) 
        #define BOARD "Wemos D1 R1"
    #elif defined(ARDUINO_WIFINFO)
        #define BOARD "WifInfo"
    #elif defined(ARDUINO_ESP8266_ARDUINO)
        #define BOARD "Arduino (ESP8266)"
    #elif defined(ARDUINO_GEN4_IOD)
        #define BOARD "4D Systems gen4 IoD Range"
    #elif defined(ARDUINO_ESP8266_OAK)
        #define BOARD "Digistump Oak"
    #elif defined(ARDUINO_WIFIDUINO_ESP8266)
        #define BOARD "WiFiduino"
    #elif defined(ARDUINO_AMPERKA_WIFI_SLOT)
        #define BOARD "Amperka WiFi Slot"
    #elif defined(ARDUINO_ESP8266_WIO_LINK)
        #define BOARD "Seeed Wio Link"
    #elif defined(ARDUINO_ESP8266_ESPECTRO_CORE)
        #define BOARD "ESPectro Core"
    #elif defined(ARDUINO_ESP8266_SCHIRMILABS_EDUINO_WIFI)
        #define BOARD "Schirmilabs Eduino WiFi"
    #elif defined(ARDUINO_ESP8266_SONOFF_SV)
        #define BOARD "ITEAD Sonoff"
    #endif

#elif defined (ARDUINO_ARCH_ESP32) 
    #define	_CPU_NAME_ "ESP32"
    #if defined(ARDUINO_D1_MINI32)
        #define BOARD "Wemos D1 MINI ESP32"
    #elif defined(ARDUINO_MH_ET_LIVE_ESP32MINIKIT)
        #define BOARD "MH ET LIVE ESP32MiniKit"
    #endif

#elif defined(TEENSYDUINO) 
    //  --------------- Teensy -----------------
    #if defined(__AVR_ATmega32U4__)
        #define BOARD "Teensy 2.0"
    #elif defined(__AVR_AT90USB1286__)       
        #define BOARD "Teensy++ 2.0"
    #elif defined(__MK20DX128__)       
        #define BOARD "Teensy 3.0"
    #elif defined(__MK20DX256__)       
        #define BOARD "Teensy 3.1" // and Teensy 3.2
    #elif defined(__MKL26Z64__)       
        #define BOARD "Teensy LC"
    #elif defined(__MK66FX1M0__)
        #define BOARD "Teensy++ 3.0" // coming soon
    #else
//       #error "Unknown board"
    #endif
#else // --------------- Arduino ------------------

    #if   defined(ARDUINO_AVR_ADK)       
        #define BOARD "Mega Adk"
    #elif defined(ARDUINO_AVR_BT)    // Bluetooth
        #define BOARD "Bt"
    #elif defined(ARDUINO_AVR_DUEMILANOVE)       
        #define BOARD "Duemilanove"
    #elif defined(ARDUINO_AVR_ESPLORA)       
        #define BOARD "Esplora"
    #elif defined(ARDUINO_AVR_ETHERNET)       
        #define BOARD "Ethernet"
    #elif defined(ARDUINO_AVR_FIO)       
        #define BOARD "Fio"
    #elif defined(ARDUINO_AVR_GEMMA)
        #define BOARD "Gemma"
    #elif defined(ARDUINO_AVR_LEONARDO)       
        #define BOARD "Leonardo"
    #elif defined(ARDUINO_AVR_LILYPAD)
        #define BOARD "Lilypad"
    #elif defined(ARDUINO_AVR_LILYPAD_USB)
        #define BOARD "Lilypad Usb"
    #elif defined(ARDUINO_AVR_MEGA)       
        #define BOARD "Mega"
    #elif defined(ARDUINO_AVR_MEGA2560)       
        #define BOARD "Mega 2560"
    #elif defined(ARDUINO_AVR_MICRO)       
        #define BOARD "Micro"
    #elif defined(ARDUINO_AVR_MINI)       
        #define BOARD "Mini"
    #elif defined(ARDUINO_AVR_NANO)       
        #define BOARD "Nano"
    #elif defined(ARDUINO_AVR_NG)       
        #define BOARD "NG"
    #elif defined(ARDUINO_AVR_PRO)       
        #define BOARD "Pro"
    #elif defined(ARDUINO_AVR_ROBOT_CONTROL)       
        #define BOARD "Robot Ctrl"
    #elif defined(ARDUINO_AVR_ROBOT_MOTOR)       
        #define BOARD "Robot Motor"
    #elif defined(ARDUINO_AVR_UNO)       
        #define BOARD "Uno"
    #elif defined(ARDUINO_AVR_YUN)       
        #define BOARD "Yun"

    // These boards must be installed separately:
    #elif defined(ARDUINO_SAM_DUE)       
        #define BOARD "Due"
    #elif defined(ARDUINO_SAMD_ZERO)       
        #define BOARD "Zero"
    #elif defined(ARDUINO_ARC32_TOOLS)       
        #define BOARD "101"
    #else
       #error "Unknown board"
    #endif

#endif

//**************************************************************************************************
//*
//*	Atmel AVR CPU name strings
//*
//**************************************************************************************************
//*	Sep 19,	2010	<MLS> Started on avr_cpunames.h
//**************************************************************************************************

//#include	"avr_cpunames.h"

//**************************************************************************************************


#if defined (__AVR_AT94K__)
                                                                        #define	_CPU_NAME_	"AT94k"
#elif defined (__AVR_AT43USB320__)
#elif defined (__AVR_AT43USB355__)
#elif defined (__AVR_AT76C711__)
#elif defined (__AVR_AT86RF401__)
#elif defined (__AVR_AT90PWM1__)
#elif defined (__AVR_AT90PWM2__)
#elif defined (__AVR_AT90PWM2B__)
#elif defined (__AVR_AT90PWM3__)
#elif defined (__AVR_AT90PWM3B__)
#elif defined (__AVR_AT90PWM216__)
#elif defined (__AVR_AT90PWM316__)
#elif defined (__AVR_ATmega32C1__)
#elif defined (__AVR_ATmega32M1__)
#elif defined (__AVR_ATmega32U4__)
									#define	_CPU_NAME_	"ATmega32U4"
#elif defined (__AVR_ATmega32U6__)
									#define	_CPU_NAME_	"ATmega32U6"
#elif defined (__AVR_ATmega128__)
									#define	_CPU_NAME_	"Atmega128"
#elif defined (__AVR_ATmega1280__)
									#define	_CPU_NAME_	"ATmega1280"
#elif defined (__AVR_ATmega1281__)
									#define	_CPU_NAME_	"ATmega1281"
#elif defined (__AVR_ATmega1284P__)
									#define	_CPU_NAME_	"ATmega1284"
#elif defined (__AVR_ATmega128RFA1__)
									#define	_CPU_NAME_	"ATmega128RFA1"
#elif defined (__AVR_ATmega2560__)
									#define	_CPU_NAME_	"ATmega2560"
#elif defined (__AVR_ATmega2561__)
									#define	_CPU_NAME_	"ATmega2561"
#elif defined (__AVR_AT90CAN32__)
									#define	_CPU_NAME_	"AT90CAN32"
#elif defined (__AVR_AT90CAN64__)
									#define	_CPU_NAME_	"AT90CAN64"
#elif defined (__AVR_AT90CAN128__)
									#define	_CPU_NAME_	"AT90CAN128"
#elif defined (__AVR_AT90USB82__)
									#define	_CPU_NAME_	"AT90USB82"
#elif defined (__AVR_AT90USB162__)
									#define	_CPU_NAME_	"AT90USB162"
#elif defined (__AVR_AT90USB646__)
									#define	_CPU_NAME_	"AT90USB646"
#elif defined (__AVR_AT90USB647__)
									#define	_CPU_NAME_	"AT90USB647"
#elif defined (__AVR_AT90USB1286__)
									#define	_CPU_NAME_	"AT90USB1286"
#elif defined (__AVR_AT90USB1287__)
									#define	_CPU_NAME_	"AT90USB1287"
#elif defined (__AVR_ATmega64__)
									#define	_CPU_NAME_	"ATmega64"
#elif defined (__AVR_ATmega640__)
									#define	_CPU_NAME_	"ATmega640"
#elif defined (__AVR_ATmega644__)
									#define	_CPU_NAME_	"ATmega644"
#elif defined (__AVR_ATmega644P__)
									#define	_CPU_NAME_	"ATmega644P"
#elif defined (__AVR_ATmega645__)
									#define	_CPU_NAME_	"ATmega645"
#elif defined (__AVR_ATmega6450__)
									#define	_CPU_NAME_	"ATmega6450"
#elif defined (__AVR_ATmega649__)
									#define	_CPU_NAME_	"ATmega649"
#elif defined (__AVR_ATmega6490__)
									#define	_CPU_NAME_	"ATmega6490"
#elif defined (__AVR_ATmega103__)
									#define	_CPU_NAME_	"ATmega103"
#elif defined (__AVR_ATmega32__)
									#define	_CPU_NAME_	"Atmega32"
#elif defined (__AVR_ATmega323__)
									#define	_CPU_NAME_	"ATmega323"
#elif defined (__AVR_ATmega324P__)
									#define	_CPU_NAME_	"ATmega324P"
#elif defined (__AVR_ATmega325__)
									#define	_CPU_NAME_	"ATmega325"
#elif defined (__AVR_ATmega325P__)
									#define	_CPU_NAME_	"ATmega325P"
#elif defined (__AVR_ATmega3250__)
									#define	_CPU_NAME_	"ATmega3250"
#elif defined (__AVR_ATmega3250P__)
									#define	_CPU_NAME_	"ATmega3250P"
#elif defined (__AVR_ATmega328P__)
									#define	_CPU_NAME_	"ATmega328P"
#elif defined (__AVR_ATmega329__)
									#define	_CPU_NAME_	"ATmega329"
#elif defined (__AVR_ATmega329P__)
									#define	_CPU_NAME_	"ATmega329P"
#elif defined (__AVR_ATmega3290__)
									#define	_CPU_NAME_	"ATmega3290"
#elif defined (__AVR_ATmega3290P__)
									#define	_CPU_NAME_	"ATmega3290P"
#elif defined (__AVR_ATmega32HVB__)
									#define	_CPU_NAME_	"ATmega32HVB"
#elif defined (__AVR_ATmega406__)
									#define	_CPU_NAME_	"ATmega406"
#elif defined (__AVR_ATmega16__)
									#define	_CPU_NAME_	"Atmega16"
#elif defined (__AVR_ATmega161__)
									#define	_CPU_NAME_	"ATmega161"
#elif defined (__AVR_ATmega162__)
									#define	_CPU_NAME_	"ATmega162"
#elif defined (__AVR_ATmega163__)
									#define	_CPU_NAME_	"ATmega163"
#elif defined (__AVR_ATmega164P__)
									#define	_CPU_NAME_	"ATmega164P"
#elif defined (__AVR_ATmega165__)
									#define	_CPU_NAME_	"ATmega165"
#elif defined (__AVR_ATmega165P__)
									#define	_CPU_NAME_	"ATmega165P"
#elif defined (__AVR_ATmega168__)
									#define	_CPU_NAME_	"ATmega168"
#elif defined (__AVR_ATmega168P__)
									#define	_CPU_NAME_	"ATmega168P"
#elif defined (__AVR_ATmega169__)
									#define	_CPU_NAME_	"Atmega169"
#elif defined (__AVR_ATmega169P__)
									#define	_CPU_NAME_	"ATmega169P"
#elif defined (__AVR_ATmega8HVA__)
									#define	_CPU_NAME_	"ATmega8HVA"
#elif defined (__AVR_ATmega16HVA__)
									#define	_CPU_NAME_	"ATmega16HVA"
#elif defined (__AVR_ATmega8__)
									#define	_CPU_NAME_	"ATmega8"
#elif defined (__AVR_ATmega48__)
									#define	_CPU_NAME_	"ATmega48"
#elif defined (__AVR_ATmega48P__)
									#define	_CPU_NAME_	"ATmega48P"
#elif defined (__AVR_ATmega88__)
									#define	_CPU_NAME_	"ATmega88"
#elif defined (__AVR_ATmega88P__)
									#define	_CPU_NAME_	"ATmega88P"
#elif defined (__AVR_ATmega8515__)
									#define	_CPU_NAME_	"ATmega8515"
#elif defined (__AVR_ATmega8535__)
									#define	_CPU_NAME_	"ATmega8535"
#elif defined (__AVR_AT90S8535__)
#elif defined (__AVR_AT90C8534__)
#elif defined (__AVR_AT90S8515__)
#elif defined (__AVR_AT90S4434__)
#elif defined (__AVR_AT90S4433__)
#elif defined (__AVR_AT90S4414__)
#elif defined (__AVR_ATtiny22__)
#elif defined (__AVR_ATtiny26__)
#elif defined (__AVR_AT90S2343__)
#elif defined (__AVR_AT90S2333__)
#elif defined (__AVR_AT90S2323__)
#elif defined (__AVR_AT90S2313__)
#elif defined (__AVR_ATtiny2313__)
									#define	_CPU_NAME_	"ATtiny2313"
#elif defined (__AVR_ATtiny13__)
#elif defined (__AVR_ATtiny13A__)
#elif defined (__AVR_ATtiny25__)
#elif defined (__AVR_ATtiny45__)
#elif defined (__AVR_ATtiny85__)
#elif defined (__AVR_ATtiny24__)
#elif defined (__AVR_ATtiny44__)
#elif defined (__AVR_ATtiny84__)
#elif defined (__AVR_ATtiny261__)
#elif defined (__AVR_ATtiny461__)
#elif defined (__AVR_ATtiny861__)
#elif defined (__AVR_ATtiny43U__)
#elif defined (__AVR_ATtiny48__)
#elif defined (__AVR_ATtiny88__)
#elif defined (__AVR_ATtiny167__)
#elif defined (__AVR_ATmega8U2__)
									#define	_CPU_NAME_	"ATmega8U2"
#else
//	#error cpu not defined
#endif


#if !defined (_CPU_NAME_)
	#define	_CPU_NAME_	"UNKNOWN"
#endif

