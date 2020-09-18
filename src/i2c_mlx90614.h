#pragma once
/*
 
 Based on https://github.com/sparkfun/SparkFun_MLX90614_Arduino_Library
*/

#define MLX90614_I2C_ADDR                                       (0x5A)

#define MLX90614_OPCODE_READ_RAM                                (0x00)
#define MLX90614_OPCODE_READ_ROM                                (0x20)
#define MLX90614_OPCODE_READ_FLAGS                              (0xF0)
#define MLX90614_OPCODE_ENTER_SLEEP_MODE                        (0xFF)

// RAM
#define MLX90614_RAWIR1                                         (0x04)
#define MLX90614_RAWIR2                                         (0x05)
#define MLX90614_TA                                             (0x06)
#define MLX90614_TOBJ1                                          (0x07)
#define MLX90614_TOBJ2                                          (0x08)
#define MLX90614_TX_ERROR_MASK                                  (0x8000U)

// EEPROM
#define MLX90614_TOMAX                                          (0x20)
#define MLX90614_TOMIN                                          (0x21)
#define MLX90614_PWMCTRL                                        (0x22)
#define MLX90614_TARANGE                                        (0x23)
#define MLX90614_EMISS                                          (0x24)
#define MLX90614_CONFIG                                         (0x25)
#define MLX90614_ADDR                                           (0x0E)
#define MLX90614_ID1                                            (0x3C)
#define MLX90614_ID2                                            (0x3D)
#define MLX90614_ID3                                            (0x3E)
#define MLX90614_ID4                                            (0x3F)

#define MLX90614_TEMPERATURE_ZONE_NONE                          (0xFF)
#define MLX90614_TEMPERATURE_ZONE_AMBIENT                       (0x00)
#define MLX90614_TEMPERATURE_ZONE_01                            (0x01)
#define MLX90614_TEMPERATURE_ZONE_02                            (0x02)


/*****************************************************************************************************************************
*
*  Read specified metric's value of the MLX90614 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_FAIL              on any error flags 
*    - RESULT_IS_UNSIGNED_VALUE    on success when ID metric specified
*    - RESULT_IS_FLOAT_02_DIGIT    on success when TEMP metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getMLX90614Metric(SoftwareTWI* _softTWI, const uint8_t _i2cAddress, const uint8_t _temperatureZone, const uint8_t _metric, int32_t* _value);
