#ifndef Zabbuino_h
#define Zabbuino_h

#include <Arduino.h>
#include <IPAddress.h>
#include "avr_cpunames.h"
#include "platforms.h"


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                               ALARM SECTION 
*/

// Where is state LED connected
#define PIN_STATE_LED              	9
// State LED must blink or just be turned on?
#define ON_ALARM_STATE_BLINK            
// Turn off state LED blink (no errors found)

// State LED no blink type
#define BLINK_NOPE                 	0
// State LED blink type with DHCP problem reached (no renew lease or more)
#define BLINK_DHCP_PROBLEM         	150 // ~150ms on, ~850ms off
// State LED blink type with Network activity problem (no packets processed for NET_IDLE_TIMEOUT)
#define BLINK_NET_PROBLEM          	500 // ~500ms on, ~500ms off


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            NETWORK MODULE SECTION 
*/

// How often do ENC28J60 module reinit for more stable network
#define NET_ENC28J60_REINIT_PERIOD  	10000UL  // 10 sec
// Network activity timeout (for which no packets processed or no DHCP lease renews finished with success)
#define NET_IDLE_TIMEOUT            	60000UL  // 60 sec
// How often do renew DHCP lease
#define NET_DHCP_RENEW_PERIOD       	30000UL  // 30 sec
// How often do renew DHCP lease
#define NET_STABILIZATION_DELAY     	100L     // 0.1 sec

#ifdef ethernet_h
#define NET_MODULE_NAME         	"WizNet 5xxx"
#elif defined UIPETHERNET_H
#define NET_MODULE_NAME         	"Microchip ENC28J60"
#endif

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         SYSTEM CONFIGURATION SECTION 
*/

#define SYS_DEFAULT_USE_DHCP        	false
#define SYS_DEFAULT_MAC_ADDRESS     	{0xDE,0xAD,0xBE,0xEF,0xFE,0xF5}
//#define SYS_DEFAULT_IP_ADDRESS      	{192,168,0,228}
#define SYS_DEFAULT_IP_ADDRESS      	{172,16,100,228}
#define SYS_DEFAULT_NETMASK         	{255,255,255,0}
//#define SYS_DEFAULT_GATEWAY         	{192,168,0,1}
#define SYS_DEFAULT_GATEWAY         	{172,16,100,254}
// It's just number of "long int" type. Surprise!
#define SYS_DEFAULT_PASSWORD        	0x000
#define SYS_DEFAULT_PROTECTION      	true

#define ZBX_AGENT_DEFAULT_HOSTNAME  	"zabbuino.local.net"
// How much bytes will allocated to hostname store
#define ZBX_AGENT_HOSTNAME_MAXLEN   	25

// Digital pin which must shorted on the GND for HOLD_TIME_TO_FACTORY_RESET time to save default system setting into EEPROM
#define PIN_FACTORY_RESET           	8 
#define HOLD_TIME_TO_FACTORY_RESET  	5000L // 5 seconds

// How many secs device may be stay in infinitibe loop before reboot
// Also you can use:
// WDTO_1S
// WDTO_2S
// WDTO_4S - not for all controllers (please reference to avr/wdt.h)
// WDTO_8S - not for all controllers (please reference to avr/wdt.h)
#define WTD_TIMEOUT                   	WDTO_8S 

#define SYS_METRICS_MAX            	3
#define SYS_METRIC_IDX_CMDCOUNT        	0
#define SYS_METRIC_IDX_VCCMIN         	1
#define SYS_METRIC_IDX_VCCMAX         	2
#define SYS_METRIC_RENEW_PERIOD        	1000L // 1 sec

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            COMMAND NAMES SECTION 
*/
// Increase this if add new command 
#define CMD_MAX 0x25

// Add command macro with new sequental number
#define CMD_ZBX_AGENT_PING           	0x00
#define CMD_ZBX_AGENT_HOSTNAME       	0x01
#define CMD_ZBX_AGENT_VERSION        	0x02
#define CMD_SYS_CMDCOUNT             	0x03
#define CMD_SYS_CPUNAME              	0x04
#define CMD_SYS_FREERAM              	0x05
#define CMD_SYS_NETMODULE           	0x06
#define CMD_SYS_GET_VCCMIN           	0x07
#define CMD_SYS_GET_VCC              	0x08
#define CMD_SYS_GET_VCCMAX           	0x09
#define CMD_SYS_UPTIME               	0x0A
#define CMD_SYS_PORTWRITE            	0x0B
#define CMD_SYS_REBOOT               	0x0C
#define CMD_SYS_SET_HOSTNAME         	0x0D
#define CMD_SYS_SET_PASSWORD         	0x0E
#define CMD_SYS_SET_PROTECTION       	0x0F
#define CMD_SYS_SET_NETWORK          	0x10
#define CMD_SYS_SHIFTOUT             	0x11
#define CMD_ARDUINO_ANALOGREAD		0x12
#define CMD_ARDUINO_ANALOGWRITE        	0x13
#define CMD_ARDUINO_ANALOGREFERENCE    	0x14
#define CMD_ARDUINO_DELAY              	0x15
#define CMD_ARDUINO_DIGITALREAD        	0x16
#define CMD_ARDUINO_DIGITALWRITE       	0x17
#define CMD_ARDUINO_NOTONE           	0x18
#define CMD_ARDUINO_TONE             	0x19
#define CMD_ARDUINO_RANDOM           	0x1A
#define CMD_ARDUINO_RANDOMSEED       	0x1B
#define CMD_DS18X20_SEARCH           	0x1C
#define CMD_DS18X20_TEMPERATURE      	0x1D
#define CMD_DHT_HUMIDITY             	0x1E
#define CMD_DHT_TEMPERATURE          	0x1F
#define CMD_BMP_PRESSURE             	0x20
#define CMD_BMP_TEMPERATURE          	0x21
#define CMD_BH1750_LIGHT           	0x22
#define CMD_I2C_SCAN            	0x23
#define CMD_INTERRUPT_COUNT            	0x24

// add new command as "const char command_<COMMAND_MACRO> PROGMEM". Only 'const' push string to PROGMEM. Tanx, Arduino.
const char command_CMD_ZBX_AGENT_PING[] 		PROGMEM	= "agent.ping";
const char command_CMD_ZBX_AGENT_HOSTNAME[] 		PROGMEM = "agent.hostname";
const char command_CMD_ZBX_AGENT_VERSION[] 		PROGMEM = "agent.version";
const char command_CMD_SYS_CMDCOUNT[] 			PROGMEM = "sys.cmdcount";
const char command_CMD_SYS_CPUNAME[] 			PROGMEM = "sys.cpuname";
const char command_CMD_SYS_FREERAM[] 			PROGMEM = "sys.freeram";
const char command_CMD_SYS_NETMODULE[] 			PROGMEM = "sys.netmodule";
const char command_CMD_SYS_GET_VCCMIN[] 		PROGMEM = "sys.vccmin";
const char command_CMD_SYS_GET_VCC[] 			PROGMEM = "sys.vcc";
const char command_CMD_SYS_GET_VCCMAX[] 		PROGMEM = "sys.vccmax";
const char command_CMD_SYS_UPTIME[] 			PROGMEM = "sys.uptime";
const char command_CMD_SYS_PORTWRITE[] 			PROGMEM = "portwrite";
const char command_CMD_SYS_REBOOT[] 			PROGMEM = "reboot";              
const char command_CMD_SYS_SET_HOSTNAME[] 		PROGMEM = "sethostname";
const char command_CMD_SYS_SET_PASSWORD[] 		PROGMEM = "setpassword";        
const char command_CMD_SYS_SET_PROTECTION[] 		PROGMEM = "setprotection";
const char command_CMD_SYS_SET_NETWORK[] 		PROGMEM = "setnetwork";
const char command_CMD_SYS_SHIFTOUT[] 			PROGMEM = "shiftout";            
const char command_CMD_ARDUINO_ANALOGREAD[] 		PROGMEM = "analogread";
const char command_CMD_ARDUINO_ANALOGWRITE[] 		PROGMEM = "analogwrite";         
const char command_CMD_ARDUINO_ANALOGREFERENCE[] 	PROGMEM	= "analogreference";
const char command_CMD_ARDUINO_DELAY[] 			PROGMEM = "delay";
const char command_CMD_ARDUINO_DIGITALREAD[] 		PROGMEM = "digitalread"; 
const char command_CMD_ARDUINO_DIGITALWRITE[] 		PROGMEM = "digitalwrite";
const char command_CMD_ARDUINO_NOTONE[] 		PROGMEM = "notone";
const char command_CMD_ARDUINO_TONE[] 			PROGMEM = "tone";
const char command_CMD_ARDUINO_RANDOM[] 		PROGMEM = "random";
const char command_CMD_ARDUINO_RANDOMSEED[] 		PROGMEM = "randomseed";
const char command_CMD_DS18X20_SEARCH[] 		PROGMEM = "ds18x20.search";
const char command_CMD_DS18X20_TEMPERATURE[] 		PROGMEM	= "ds18x20.temperature";
const char command_CMD_DHT_HUMIDITY[] 			PROGMEM = "dht.humidity";
const char command_CMD_DHT_TEMPERATURE[] 		PROGMEM = "dht.temperature";
const char command_CMD_BMP_PRESSURE[] 			PROGMEM = "bmp.pressure";
const char command_CMD_BMP_TEMPERATURE[] 		PROGMEM = "bmp.temperature";
const char command_CMD_BH1750_LIGHT[] 			PROGMEM = "bh1750.light";
const char command_CMD_I2C_SCAN[] 			PROGMEM = "i2c.scan";
const char command_CMD_INTERRUPT_COUNT[] 		PROGMEM = "interrupt.count";

// do not insert new command to any position without syncing indexes. Tanx, Arduino, for this method of string array pushing to PROGMEM
const char* const commands[] PROGMEM = {
  command_CMD_ZBX_AGENT_PING, 		
  command_CMD_ZBX_AGENT_HOSTNAME,
  command_CMD_ZBX_AGENT_VERSION, 		
  command_CMD_SYS_CMDCOUNT, 			
  command_CMD_SYS_CPUNAME, 			
  command_CMD_SYS_FREERAM, 			
  command_CMD_SYS_NETMODULE, 			
  command_CMD_SYS_GET_VCCMIN, 		
  command_CMD_SYS_GET_VCC, 			
  command_CMD_SYS_GET_VCCMAX, 		
  command_CMD_SYS_UPTIME, 			
  command_CMD_SYS_PORTWRITE, 			
  command_CMD_SYS_REBOOT, 			
  command_CMD_SYS_SET_HOSTNAME, 		
  command_CMD_SYS_SET_PASSWORD, 		
  command_CMD_SYS_SET_PROTECTION, 		
  command_CMD_SYS_SET_NETWORK, 		
  command_CMD_SYS_SHIFTOUT, 			
  command_CMD_ARDUINO_ANALOGREAD, 		
  command_CMD_ARDUINO_ANALOGWRITE, 		
  command_CMD_ARDUINO_ANALOGREFERENCE, 	
  command_CMD_ARDUINO_DELAY, 			
  command_CMD_ARDUINO_DIGITALREAD, 		
  command_CMD_ARDUINO_DIGITALWRITE, 		
  command_CMD_ARDUINO_NOTONE, 		
  command_CMD_ARDUINO_TONE, 			
  command_CMD_ARDUINO_RANDOM, 		
  command_CMD_ARDUINO_RANDOMSEED, 		
  command_CMD_DS18X20_SEARCH, 		
  command_CMD_DS18X20_TEMPERATURE, 		
  command_CMD_DHT_HUMIDITY, 			
  command_CMD_DHT_TEMPERATURE, 		
  command_CMD_BMP_PRESSURE, 			
  command_CMD_BMP_TEMPERATURE, 		
  command_CMD_BH1750_LIGHT, 			
  command_CMD_I2C_SCAN, 			
  command_CMD_INTERRUPT_COUNT
};

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           VARIOUS DEFINES SECTION 
*/

#define EXTERNAL_INTERRUPTS_MAX        2

// Number of expected arguments of the command
#define ARGS_MAX                    	6
// Size of buffer's argument part. All separators and delimiters must be taken into account
#define BUFFER_ARGS_PART_SIZE         	50
// Size of buffer's command part
#define BUFFER_CMD_PART_SIZE          	25
// The total size of the buffer
#define BUFFER_SIZE                   	BUFFER_CMD_PART_SIZE + BUFFER_ARGS_PART_SIZE

// Zabbix v2.x header prefix ('ZBXD\x01')
#define ZBX_HEADER_PREFIX             	"zbxd\1"
// sizeof() give wrong result -> 6
#define ZBX_HEADER_PREFIX_LENGTH      	5
// Zabbix v2.x header length
#define ZBX_HEADER_LENGTH             	12

#define ZBX_NOTSUPPORTED_MSG          	"ZBX_NOTSUPPORTED"

#define ZBX_AGENT_VERISON             	"Zabbuino 1.0.0"

#define SENS_READ_TEMP 			0x01
#define SENS_READ_HUMD 			0x02
#define SENS_READ_PRSS 			0x03
#define SENS_READ_LUX                   0x04

#define SENS_READ_RAW 			0xFF

#define RESULT_IS_FAIL                	-0xFFAL
#define RESULT_IS_OK                  	-0xFFBL
#define RESULT_IN_BUFFER              	-0xFFCL
#define RESULT_IS_PRINTED             	-0xFFDL

// Error Codes
#define DEVICE_DISCONNECTED_C         	-127
#define DEVICE_DISCONNECTED_F         	-196.6
#define DEVICE_DISCONNECTED_RAW       	-7040

// see below: const byte port_protect[PORTS_NUM] = {...}
#if defined (ARDUINO_AVR_DUEMILANOVE) || defined (ARDUINO_AVR_MINI) || defined (ARDUINO_AVR_NANO) || defined (ARDUINO_AVR_NG) || defined (ARDUINO_AVR_PRO) || defined (ARDUINO_AVR_ETHERNET)
#define PORTS_NUM 5
#elif defined(ARDUINO_AVR_LEONARDO) || defined (ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_ROBOT_CONTROL) || defined(ARDUINO_AVR_ROBOT_MOTOR) || defined (ARDUINO_AVR_YUN)
#define PORTS_NUM 7
#elif defined (ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
#define PORTS_NUM 13
#else // Unknow boards equal to "Arduino Duemilanove"
#define PORTS_NUM 5
#define ARDUINO_AVR_DUEMILANOVE
#endif


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
#define ANALOG_CHAN_VBG 		0xE // B1110
#define ANALOG_CHAN_GND 		0xF // B1111

#define DBG_PRINT_AS_MAC 		0x1
#define DBG_PRINT_AS_IP  		0x2


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         PROGRAMM STRUCTURES SECTION 
*/
// Note: netconfig_t size must be no more ___uint8_t___ bytes, because readConfig()'s read cycle use uint8_t counter. 
// Change the index's variable type if bigger size need
typedef struct {
  uint8_t useDHCP;         			// 1 byte
  uint8_t macAddress[6];   			// 6 byte 
  IPAddress ipAddress;     			// 6 byte (uint8_t[])
  IPAddress ipNetmask;     			// 6 byte (uint8_t[])
  IPAddress ipGateway;     			// 6 byte (uint8_t[])
  char hostname[ZBX_AGENT_HOSTNAME_MAXLEN];  	// 255 - (1 + 6*4 + 4 + 1) = 225 bytes max
  // #ifdef ... #elif ... #endif does not work with struct operator
  uint32_t password;        			// 4 byte
  uint8_t useProtection;    			// 1 byte
} netconfig_t ;


typedef struct {
  uint32_t count;        			// 4 byte
  // mode == -1 => Interrupt is not attached
  int8_t mode;         			        // 1 byte 
} extInterrupt_t ;

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         I/O PORTS SETTING SECTION 
*/

/*
  Защитные маски портов ввода/вывода.
  Значение '1' в определенной битовой позиции означает то, что при операциях записи в порт данный бит не будет изменен (находится под защитой). Эта маски
  также применяются для проверки безопасности при операциях изменения состояния определенного пина. Функция isSafePin() проверяет не установлена ли защита
  для конкретного пина.

  Вы можете изменять как сами маски, так и их количество. Однако, количество элементов данного массива должно соответствовать параметру PORTS_NUM.

  Например: Необходимо защитить от изменения pin D13, так как он используется библиотекой Ethernet и его изменение извне приведет к некорректной работе
  сетевого адаптера. В заголовочном файле pins_arduino.h определенном для необходимой платформы находим массив digital_pin_to_bit_mask_PGM. Элемент #13 указывает на
  связь данного пина с портом B (PORTB) и битом 5. Значит, для защиты данного пина необходимо установить в нижележащем массиве port_protect 5-й символ справа
  в строке "B..... // PORTB" в значение '1'.
*/

const byte port_protect[PORTS_NUM] = {
#if defined (ARDUINO_AVR_DUEMILANOVE) || defined (ARDUINO_AVR_MINI) || defined (ARDUINO_AVR_NANO) || defined (ARDUINO_AVR_NG) || defined (ARDUINO_AVR_PRO) || defined (ARDUINO_AVR_ETHERNET)
  B00000000, // not a port
  B00000000, // not a port
  // Pins D10, D11, D12, D13 is protected by setting 2, 3, 4, 5 bits, due its used to SPI (ethernet shield)
  // Bits 6, 7 have not correspondented pins in Arduino Mini Pro / Freeduino 2009
  B11111100, /*     PORTB        
D13 -^    ^- D8    <- pins   */
  B00000000, /*     PORTC 
   ^-A7   ^-A0   <- pins    */
  // Pins D0, D1 is protected by settings 0, 1 bits, due its used to RX/TX lines of UART and make it possible to transmit data to Serial Monitor  
  B00000011  /*     PORTD 
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

/*
   Маски для задания направления ввода/вывода выходов микроконтроллера.
   Значение '1' в определенной битовой позиции означает то, что при инициализации соответствующий биту пин будет установлен в состояние OUTPUT.
   В противном случае он будет оставлен с состоянии по умолчанию для платформы, на которой выполняется программный код.

   Вы можете изменять как сами маски, так и их количество. Однако, количество элементов данного массива должно соответствовать параметру PORTS_NUM.

   Например: Необходимо при инициализации установить пин D8 в режим OUTPUT, а пин D9 оставить в состоянии по умолчанию - INPUT.
   В заголовочном файле pins_arduino.h определенном для необходимой платформы находим массив digital_pin_to_bit_mask_PGM. Элемент #8 связан с битом 0 порта B,
   а элемент #9 с битом 1 того же порта. Значит, для правильной инициализации следует установить в нижележащем массиве port_mode 0-й символ справа
   в строке "B..... // PORTB" в значение '1', а 1-й символ справа в значение '0'

   Будьте внимательны и осторожны. Установка пина в состояние INPUT увеличивает при неаккуратном обращении с устройством вероятность вывода из строя
   соответствующего вывода микроконтроллера.
*/


const byte port_mode[PORTS_NUM] = {
 
  // All bits equal '0' cause setting corresponding pin to INPUT mode
  // All bits equal '1' cause setting corresponding pin to OUTPUT mode
 
#if defined (ARDUINO_AVR_DUEMILANOVE) || defined (ARDUINO_AVR_MINI) || defined (ARDUINO_AVR_NANO) || defined (ARDUINO_AVR_NG) || defined (ARDUINO_AVR_PRO) || defined (ARDUINO_AVR_ETHERNET)
  B00000000, // not a port
  B00000000, // not a port
  // Bits 6, 7 have not correspondented pins in Arduino Mini Pro / Freeduino 2009
  B00111110, /*     PORTB        
D13 -^    ^- D8    <- pins   */
  B11111110, /*     PORTC 
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

/*
  Маски для установки состояния выходов микроконтроллера.
  Значение '1' в определенной битовой позиции задает высокое состояние пина при инициализации (см. описание функции Arduino pinMode()). Если маской port_mode
  соответствующий пин определен, как OUTPUT, то его итоговое состояние станет OUTPUT+HIGH. В случае с определением пина, как работающего в режиме INPUT, итоговым
  состоянием будет INPUT_PULLUP.

  Вы можете изменять как сами маски, так и их количество. Однако, количество элементов данного массива должно соответствовать параметру PORTS_NUM.
  Вычисление битов, соотвующих пинам аналогично способам, применяемым в port_protect и port_mode.
*/
const byte port_pullup[PORTS_NUM] = {

  // All bits equal '0' cause do not pull-up corresponding pin
  // All bits equal '1' cause pull-up corresponding pin

#if defined (ARDUINO_AVR_DUEMILANOVE) || defined (ARDUINO_AVR_MINI) || defined (ARDUINO_AVR_NANO) || defined (ARDUINO_AVR_NG) || defined (ARDUINO_AVR_PRO) || defined (ARDUINO_AVR_ETHERNET)
  B00000000, // not a port
  B00000000, // not a port
  // Bits 6, 7 have not correspondented pins in Arduino Mini Pro / Freeduino 2009
  B00000001, /*     PORTB        
D13 -^    ^- D8    <- pins   */
  B00000000, /*     PORTC 
   ^-A7   ^-A0   <- pins    */
  B00000011  /*     PORTD 
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
#endif
