#pragma once

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            ALARM SECTION 
*/

// "No error detected" code
#define ERROR_NONE                 	                        (0x00)
// "No network activity" error code
#define ERROR_NET                 	                        (0x01)
// "DHCP problem" error code
#define ERROR_DHCP                 	                        (0x02)

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           VARIOUS DEFINES SECTION 
*/

// Zabbix v2.x header prefix ('ZBXD\x01')
#define ZBX_HEADER_PREFIX                                       "zbxd\1"
// sizeof() returns wrong result -> 6
#define ZBX_HEADER_PREFIX_LENGTH                                (5)
// Zabbix v2.x header length
#define ZBX_HEADER_LENGTH                                       (13)

#define PACKET_TYPE_NONE                                        (0x00)
#define PACKET_TYPE_PLAIN                                       (0x01)
#define PACKET_TYPE_ZABBIX                                      (0x02)

#define SENS_READ_TEMP                                          (0x01)
#define SENS_READ_HUMD                                          (0x02)
#define SENS_READ_PRSS                                          (0x03)
#define SENS_READ_LUX                                           (0x04)
#define SENS_READ_LIGHT_AMBIENT                                 (0x05)
#define SENS_READ_LIGHT_RED                                     (0x06)
#define SENS_READ_LIGHT_GREEN                                   (0x07)
#define SENS_READ_LIGHT_BLUE                                    (0x08)
#define SENS_READ_UV                                            (0x09)
                                                                    
#define SENS_READ_ZC                                            (0x0A)
#define SENS_READ_AC                                            (0x0B)
#define SENS_READ_DC                                            (0x0C)
                                                                    
#define SENS_READ_VOLTAGE                                       (0x0D)
#define SENS_READ_SHUNT_VOLTAGE                                 (0x0E)
#define SENS_READ_BUS_VOLTAGE                                   (0x0F)
#define SENS_READ_POWER                                         (0x10)
#define SENS_READ_ENERGY                                        (0x11)                                                                   
#define SENS_CHANGE_ADDRESS                                     (0x12)

#define SENS_READ_STANDART_PM10                                 (0x20)
#define SENS_READ_STANDART_PM25                                 (0x21)
#define SENS_READ_STANDART_PM100                                (0x22)
#define SENS_READ_ENVIRONMENT_PM10                              (0x23)
#define SENS_READ_ENVIRONMENT_PM25                              (0x24)
#define SENS_READ_ENVIRONMENT_PM100                             (0x25)
#define SENS_READ_PARTICLES_03_UM                               (0x26)
#define SENS_READ_PARTICLES_05_UM                               (0x27)
#define SENS_READ_PARTICLES_10_UM                               (0x28)
#define SENS_READ_PARTICLES_25_UM                               (0x29)
#define SENS_READ_PARTICLES_50_UM                               (0x2A)
#define SENS_READ_PARTICLES_100_UM                              (0x2B)

#define SENS_READ_ID                                            (0xFD)
#define SENS_READ_ALL                                           (0xFE)
#define SENS_READ_RAW                                           (0xFF)

#define OUTPUT_NUMBER                                           (0x01)
#define OUTPUT_STRING                                           (0x02)
#define OUTPUT_JSON                                             (0x03)

#define RESULT_IS_FAIL                                          false
#define RESULT_IS_OK                                            true
#define RESULT_IS_BUFFERED                                      (0x02)
#define RESULT_IS_PRINTED                                       (0x04)
#define RESULT_IS_SIGNED_VALUE                                  (0x08)
#define RESULT_IS_UNSIGNED_VALUE                                (0x10)
#define RESULT_IS_UNSTORED_IN_EEPROM                            (0x20)
#define RESULT_IS_SYSTEM_REBOOT_ACTION                          (0x30)
// RESULT_IS_NEW_COMMAND's value must not equal any command index to avoid incorrect processing
#define RESULT_IS_NEW_COMMAND                                   (0xF9)

// Error Codes
#define ZBX_NOTSUPPORTED                                        (-0x01)
#define DEVICE_ERROR_CONNECT                                    (-0x02)
#define DEVICE_ERROR_ACK_L                                      (-0x04)
#define DEVICE_ERROR_ACK_H                                      (-0x08)
#define DEVICE_ERROR_CHECKSUM                                   (-0x10)
#define DEVICE_ERROR_TIMEOUT                                    (-0x20)
#define DEVICE_ERROR_WRONG_ID                                   (-0x30)
#define DEVICE_ERROR_NOT_SUPPORTED                              (-0x40)
#define DEVICE_ERROR_WRONG_ANSWER                               (-0x50)
#define DEVICE_ERROR_EEPROM_CORRUPTED                           (-0x60)
/*
ATMega 328 ADC channels 

     • Bits 3:0 – MUX[3:0]: Analog Channel Selection Bits
       The value of these bits selects which analog inputs are connected to the ADC. See Table 24-4 for details. If
       these bits are changed during a conversion, the change will not go in effect until this conversion is complete
       (ADIF in ADCSRA is set).

       Table 24-4. Input Channel Selections
       MUX3..0  Single Ended Input
       1110     1.1V (VBG)
       1111     0V (GND)       - noise level measurement possible
*/
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define ANALOG_CHAN_VBG 		                        (0x1E) 
  #define ANALOG_CHAN_GND 		                        (0x1F)  
#else
  #define ANALOG_CHAN_VBG 		                        (0x0E) // B1110
  #define ANALOG_CHAN_GND 		                        (0x0F) // B1111
#endif
                                                                    
#define MAC_ADDRESS                                             (0x01)
#define IP_ADDRESS  	                                        (0x02)
#define I2C_ADDRESS                                             (0x03)
#define OW_ADDRESS                                              (0x04)

#define I2C_ADDRESS_LENGTH                                      (0x01)
#define OW_ADDRESS_LENGTH                                       (0x08)

// PoConfig will be stored or loaded on ...                         
#define CONFIG_STORE_PTR_ADDRESS                                (0x01)
#define CONFIG_STORE_DEFAULT_START_ADDRESS                      (0x02)
// one byte used to pointer to EEPROM's start address from which config will saved, max stored pointer value is 255
#define LAST_EEPROM_CELL_ADDRESS                                (0xFF) 

// Who is use interrupt pin
#define OWNER_IS_NOBODY                                         (0x00)
#define OWNER_IS_EXTINT                                         (0x01)
#define OWNER_IS_INCENC                                         (0x02)
                                                                    
#define NO_REINIT_ANALYZER                                      false
#define REINIT_ANALYZER                                         true

#define WANTS_VALUE_NONE                                        (0x00)
#define WANTS_VALUE_WHOLE                                       (0x01)
#define WANTS_VALUE_SCALED                                      (0x0F)
                                                                    
                                                                    
