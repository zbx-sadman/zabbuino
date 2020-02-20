#pragma once

/*

*/

#define T67XX_I2C_ADDRESS                                       (0x15)

#define T67XX_PREHEAT_GAS_CONCENTRATION                         (399)      

#define T67XX_I2C_REQUEST_SIZE                                  (0x05)
#define T67XX_I2C_RESPONSE_SIZE                                 (0x04)

#define T67XX_I2C_RESPONSE_DELAY                                (0x0A) // 5..10 ms, according to the datasheet

#define T67XX_I2C_FIELD_FUNCTION_CODE                           (0x00)
#define T67XX_I2C_FIELD_BYTES_COUNT                             (0x01)
#define T67XX_I2C_FIELD_DATA_MSB                                (0x02)
#define T67XX_I2C_FIELD_DATA_LSB                                (0x03)

#define T67XX_I2C_FIELD_REGISTER_MSB                            (0x03)
#define T67XX_I2C_FIELD_REGISTER_LSB                            (0x04)

#define T67XX_I2C_FIELD_STARTING_ADDRESS_MSB                    (0x01)
#define T67XX_I2C_FIELD_STARTING_ADDRESS_LSB                    (0x02)

#define T67XX_MODBUS_ADDRESS_FIRMWARE_MSB                       (0x13)
#define T67XX_MODBUS_ADDRESS_FIRMWARE_LSB                       (0x89)

#define T67XX_MODBUS_ADDRESS_STATUS_MSB                         (0x13)
#define T67XX_MODBUS_ADDRESS_STATUS_LSB                         (0x8A)

#define T67XX_MODBUS_ADDRESS_GAS_PPM_MSB                        (0x13)
#define T67XX_MODBUS_ADDRESS_GAS_PPM_LSB                        (0x8B)

#define T67XX_MASK_STATUS_OK                                    (0x0000UL)
#define T67XX_MASK_STATUS_ERROR_CONDITION                       (0x0001UL)
#define T67XX_MASK_STATUS_ERROR_FLASH                           (0x0002UL)
#define T67XX_MASK_STATUS_ERROR_CALIBRATION                     (0x0004UL)
#define T67XX_MASK_STATUS_REBOOT                                (0x0400UL)
#define T67XX_MASK_STATUS_WARM_UP_MODE                          (0x0800UL)
#define T67XX_MASK_STATUS_SINGLE_POINT_CALIBRATION              (0x8000UL)



/*****************************************************************************************************************************
*
*  Read specified metric's value of the T67XX sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE       on success when CONCENTRATION metric specified
*    - DEVICE_ERROR_EEPROM_CORRUPTED  on sensor's flash error detect
*    - DEVICE_ERROR_TIMEOUT           on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT           on connection error
*
*****************************************************************************************************************************/
int8_t getT67XXMetric(SoftwareWire*, uint8_t, uint8_t, int32_t*);
