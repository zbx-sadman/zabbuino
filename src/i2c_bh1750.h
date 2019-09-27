#pragma once

/*

  Datasheet: http://rohmfs.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1750fvi-e.pdf

*/

#define BH1750_I2C_ADDRESS                                      (0x23)

// No active state
#define BH1750_CMD_POWERDOWN                                    (0x00)
// Wating for measurment command
#define BH1750_CMD_POWERON                                      (0x01)
// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_CMD_RESET                                        (0x07)

// Start measurement at 1 lux resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGHRES                               (0x10)
// Start measurement at 0.5 lux resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGHRES_2                             (0x11)
// Start measurement at 4 lux resolution. Measurement time is approx 16ms.
#define BH1750_CONTINUOUS_LOWRES                                (0x13)
// Start measurement at 1 lux resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_HIGHRES                                  (0x20)
// Start measurement at 0.5 lux resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_HIGHRES_2                                (0x21)
// Start measurement at 4 lux resolution. Measurement time is approx 16ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_LOWRES                                   (0x23)

// 10 lx => 12
#define BH1750_HIGHRES_2_TRESHOLD                               (12U)

#define BH1750_LOWRES_CONVERSION_TIME_MS                        (24U)
#define BH1750_HIGHRES_2_CONVERSION_TIME_MS                     (180U)
#define BH1750_HIGHRES_CONVERSION_TIME_MS                       (180U)

/*****************************************************************************************************************************
*
*  Read specified metric's value of the BH1750 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success when RAW metric specified
*    - RESULT_IS_FLOAT_02_DIGIT    on success when LUX metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getBH1750Metric(SoftwareWire*, uint8_t, uint8_t, int32_t*);
