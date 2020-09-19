#pragma once

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                              SYSTEM HEADERS SECTION
*/

#include <Arduino.h>
#include <IPAddress.h>
#include <SoftwareSerial.h>

#include <time.h>

#if defined(ARDUINO_ARCH_AVR)
    #include <util/atomic.h>
    #include <avr/interrupt.h>
    #include <avr/pgmspace.h>
#endif

#include <wiring_private.h>


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                               ZABBUINO HEADERS SECTION
*/

// cfg_basic.h & cfg_tune.h must be included before other zabbuino's headers because they contain configuration data
#include "sys_includes.h"

#include "wrap_network.h"
#include "wrap_i2c.h"
#include "sys_commands.h"
#include "sys_platforms.h"

// runtime libs 
#include "adc.h"
#include "eeprom.h"
#include "io_regs.h"
//#include "rtc.h"
#include "system.h"
#include "service.h"

// I2C devices 
#include "i2c_bus.h"
#include "i2c_common.h"

#include "i2c_bh1750.h"
#include "i2c_ina2xx.h"
#include "i2c_lcd.h"
#include "i2c_sht.h"
#include "i2c_bmp.h"
//#include "i2c_ds3231.h"
#include "i2c_pca9685.h"
//#include "i2c_pcf8563.h"
#include "i2c_at24cxx.h"
#include "i2c_max44009.h"
#include "i2c_veml6070.h"
#include "i2c_tsl2561.h"
#include "i2c_adps9960.h"
#include "i2c_mlx90614.h"
#include "i2c_sgp30.h"
#include "i2c_t67xx.h"

// 1-Wire devices 
#include "ow_bus.h"
#include "ow_sensors.h"

/*
// UART connected devices 
#include "uart_bus.h"
#include "uart_apcsmart.h"
#include "uart_megatec.h"
#include "uart_player.h"
#include "uart_pzem.h"
#include "uart_plantower.h"
#include "uart_novafitness.h"
#include "uart_winsen.h"

// SPI-compatible devices 
#include "spi_bus.h"
#include "spi_max6675.h"
*/
// Other devices //
#include "dht.h"
/*
#include "ir.h"
#include "interrupts.h"
#include "mh_zxx.h"
#include "ultrasonic.h"
#include "shiftout.h"
#include "microwire_bus.h"
#include "actuators.h"
#include "modbus.h"
*/

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                   GLOBAL VARIABLES SECTION
*/

// some members of struct used in timer's interrupt
volatile sysmetrics_t sysMetrics;
netconfig_t sysConfig;
#ifdef TWI_USE
SoftwareTWI SoftTWI;
#endif
#ifdef INTERRUPT_USE
extern volatile extInterrupt_t extInterrupt[];
#endif
