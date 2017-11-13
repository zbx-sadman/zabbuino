#ifndef _ZABBUINO_MACROS_H_
#define _ZABBUINO_MACROS_H_

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
#define ZBX_HEADER_PREFIX_LENGTH                                4
// Zabbix v2.x header length
#define ZBX_HEADER_LENGTH                                       12

/*

Enum take more progspace on compilation that macro :(
Why? All sources tell me thah enum is preprocessor feature. May be it use 16-bit width of ptr's or so?

typedef enum {
   SENS_READ_RAW,
   SENS_READ_TEMP,
   SENS_READ_HUMD,
   SENS_READ_PRSS,
   SENS_READ_LUX,
   SENS_READ_ZC,
   SENS_READ_AC,
   SENS_READ_DC,
   SENS_READ_VOLTAGE,
   SENS_READ_POWER,
   SENS_READ_ENERGY
} sens_metrics_t;
*/

#define SENS_READ_TEMP                                          (0x01)
#define SENS_READ_HUMD                                          (0x02)
#define SENS_READ_PRSS                                          (0x03)
#define SENS_READ_LUX                                           (0x04)
                                                                    
#define SENS_READ_ZC                                            (0x08)
#define SENS_READ_AC                                            (0x09)
#define SENS_READ_DC                                            (0x0A)
                                                                    
#define SENS_READ_VOLTAGE                                       (0x0B)
#define SENS_READ_SHUNT_VOLTAGE                                 (0x0C)
#define SENS_READ_BUS_VOLTAGE                                   (0x0D)
#define SENS_READ_POWER                                         (0x0E)
#define SENS_READ_ENERGY                                        (0x0F)                                                                   
#define SENS_CHANGE_ADDRESS                                     (0x10)

#define SENS_READ_RAW                                           (0xFF)


#define RESULT_IS_FAIL                                          false
#define RESULT_IS_OK                                            true
#define RESULT_IN_BUFFER                                        (0x02)
#define RESULT_IS_PRINTED                                       (0x04)
#define RESULT_IS_SIGNED_VALUE                                  (0x08)
#define RESULT_IS_UNSIGNED_VALUE                                (0x10)
// RESULT_IS_NEW_COMMAND's value must not equal any command index to avoid incorrect processing
#define RESULT_IS_NEW_COMMAND                                   (0xF5)

// Error Codes
//#define DEVICE_DISCONNECTED_C         	-127

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
#define ANALOG_CHAN_VBG 		                        (0x0E) // B1110
#define ANALOG_CHAN_GND 		                        (0x0F) // B1111
                                                                    
#define DBG_PRINT_AS_MAC 		                        (0x01)
#define DBG_PRINT_AS_IP  		                        (0x02)

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
                                                                    
                                                                    
#endif // _ZABBUINO_MACROS_H_

