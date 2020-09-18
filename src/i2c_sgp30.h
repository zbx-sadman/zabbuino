#pragma once

/*
*/

#define SGP30_I2C_ADDRESS                                       (0x58)

#define SGP30_CMD_BYTE_COMMON                                   (0x20)

#define SGP30_CMD_BYTE_IAQ_INIT                                 (0x03)
#define SGP30_CMD_BYTE_IAQ_MEASURE                              (0x08)
#define SGP30_CMD_BYTE_MEASURE_TEST                             (0x32)
#define SGP30_CMD_BYTE_MEASURE_RAW                              (0x50)
#define SGP30_CMD_BYTE_SET_ABSOLUTE_HUMIDITY                    (0x61)

#define SGP30_TIME_IAQ_INIT                                     (10)
#define SGP30_TIME_IAQ_MEASURE                                  (12)
#define SGP30_TIME_MEASURE_TEST                                 (220)
#define SGP30_TIME_MEASURE_RAW                                  (25)
#define SGP30_TIME_SET_ABSOLUTE_HUMIDITY                        (10)

#define SGP30_CO2_BYTE_DATA                                     (0x00)
#define SGP30_CO2_BYTE_CRC                                      (0x02)
#define SGP30_TVOC_BYTE_DATA                                    (0x03)
#define SGP30_TVOC_BYTE_CRC                                     (0x05)


/*****************************************************************************************************************************
*
*   Read specified metric's value of the SGP30 sensor, put it to output buffer on success. 
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*   Returns: 
*    - RESULT_IS_BUFFERED on success
*    - DEVICE_ERROR_CONNECT on test connection error
*    - RESULT_IS_FAIL - on other fails
*
*****************************************************************************************************************************/
int8_t getSGP30Metric(SoftwareTWI* _softTWI, uint8_t _i2cAddress, const int32_t _absHumidity, const uint8_t _metric, const uint8_t _reInit, int32_t* _value);
