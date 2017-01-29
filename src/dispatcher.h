#ifndef _ZABBUINO_DISPATCHER_H_
#define _ZABBUINO_DISPATCHER_H_



/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                              SYSTEM HEADERS SECTION
*/

#include <Arduino.h>
#include <IPAddress.h>

#include <util/atomic.h>
#include <avr/interrupt.h>

#include <avr/pgmspace.h>
#include <wiring_private.h>


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                               ZABBUINO HEADERS SECTION
*/

// basic.h & tune.h must be included before other zabbuino's headers because they contain configuration data
#include "../basic.h"
#include "tune.h"

#include "structs.h"
#include "network.h"
#include "platforms.h"

/* runtime libs */
#include "adc.h"
#include "eeprom.h"
#include "io_regs.h"
#include "system.h"
#include "service.h"


/* I2C devices */
#include "i2c_bus.h"
#include "i2c_bmp.h"
#include "i2c_lcd.h"
#include "i2c_sht.h"
#include "i2c_ina2xx.h"

/* 1-Wire devices */
#include "ow_bus.h"

#include "ow_sensors.h"
/* UART connected devices */
#include "uart_bus.h"
#include "uart_apcsmart.h"
#include "uart_megatec.h"
#include "uart_pzem.h"

/* Other devices */
#include "dht.h"
#include "ir.h"
#include "interrupts.h"
#include "ultrasonic.h"
#include "shiftout.h"
#include "busMicrowire.h"


#endif // _ZABBUINO_DISPATCHER_H_

