#ifndef _ZABBUINO_I2C_BH1750_H_
#define _ZABBUINO_I2C_BH1750_H_

/*

  Datasheet: http://rohmfs.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1750fvi-e.pdf
*/
#define BH1750_I2C_FIRST_ADDRESS                                0x23
#define BH1750_I2C_SECOND_ADDRESS                               0x5C

// No active state
#define BH1750_CMD_POWERDOWN                                    0x00
// Wating for measurment command
#define BH1750_CMD_POWERON                                      0x01
// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_CMD_RESET                                        0x07

// Start measurement at 1 lux resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGHRES                               0x10
// Start measurement at 0.5 lux resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGHRES_2                             0x11
// Start measurement at 4 lux resolution. Measurement time is approx 16ms.
#define BH1750_CONTINUOUS_LOWRES                                0x13
// Start measurement at 1 lux resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_HIGHRES                                  0x20
// Start measurement at 0.5 lux resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_HIGHRES_2                                0x21
// Start measurement at 4 lux resolution. Measurement time is approx 16ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONETIME_LOWRES                                   0x23


/*****************************************************************************************************************************
*
*   Read specified metric's value of the BH1750 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/

int8_t getBH1750Metric(SoftwareWire*, uint8_t, uint8_t, const uint8_t, char*);
int8_t getBH1750Metric(SoftwareWire*, uint8_t, uint8_t, const uint8_t, uint32_t*);
int8_t getBH1750Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _metric, char *_dst, uint32_t* _value, const uint8_t _wantsNumber = false);

#endif // #ifndef _ZABBUINO_I2C_BH1750_H_