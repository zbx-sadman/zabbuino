#pragma once

#define SHT2X_I2C_ADDRESS                                       (0x40)
#define SHT2X_CMD_MEASURE_TEMPERATURE_HOLD                      (0xE3)
#define SHT2X_CMD_MEASURE_HUMIDITY_HOLD                         (0xE5)
#define SHT2X_CMD_WRITE_USER_REGISTER                           (0xE6)
#define SHT2X_CMD_READ_USER_REGISTER                            (0xE7)


#define SHT2X_TEMPERATURE_14BIT_CONVERSION_TIME_MS              (50U)
#define SHT2X_TEMPERATURE_13BIT_CONVERSION_TIME_MS              (25U)
#define SHT2X_TEMPERATURE_12BIT_CONVERSION_TIME_MS              (13U)
#define SHT2X_TEMPERATURE_11BIT_CONVERSION_TIME_MS              (7U)

#define SHT2X_HUMIDITY_12BIT_CONVERSION_TIME_MS                 (16U)
#define SHT2X_HUMIDITY_11BIT_CONVERSION_TIME_MS                 (8U)
#define SHT2X_HUMIDITY_10BIT_CONVERSION_TIME_MS                 (5U)
#define SHT2X_HUMIDITY_08BIT_CONVERSION_TIME_MS                 (3U)


#define SHT2X_CONVERSION_MODE_CLEAR_MASK                        (B00111111) // Drops "mode" bits
#define SHT2X_CONVERSION_MODE_00                                (B00000000) // RH 12bit, T 14bit
#define SHT2X_CONVERSION_MODE_01                                (B01000000) // RH 8bit , T 12bit
#define SHT2X_CONVERSION_MODE_02                                (B10000000) // RH 10bit, T 13bit
#define SHT2X_CONVERSION_MODE_03                                (B10000000) // RH 11bit, T 11bit

#define SHT2X_CRC_SHIFTED_DIVISOR                               (0x988000UL)

/*****************************************************************************************************************************
*
*  Read specified metric's value of the SHT2X sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_FLOAT_02_DIGIT    on success when TEMP metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getSHT2XMetric(SoftwareWire*, uint8_t, const uint8_t, int32_t*);

