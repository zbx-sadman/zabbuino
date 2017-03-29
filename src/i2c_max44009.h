#ifndef _ZABBUINO_I2C_MAX44009_H_
#define _ZABBUINO_I2C_MAX44009_H_

/*

  Datasheet: http://datasheets.maximintegrated.com/en/ds/MAX44009.pdf

*/

#define MAX44009_REG_CONFIGURATION                              (0x02)
#define MAX44009_REG_LUXREADING                                 (0x03)


#define MAX44009_ONETIME_MODE                                   (0x00)
#define MAX44009_CONTINUOUS_MODE                                (0x80)


// This is a preferred mode for boosting low-light sensitivity.
#define MAX44009_INTEGRATION_TIME_800MS                         (0x00)

#define MAX44009_INTEGRATION_TIME_400MS                         (0x01)
#define MAX44009_INTEGRATION_TIME_200MS                         (0x02)
// This is a preferred mode for high-brightness applications.
#define MAX44009_INTEGRATION_TIME_100MS                         (0x03)

// Manual mode only, 50ms
#define MAX44009_INTEGRATION_TIME_50MS                          (0x04)
// Manual mode only, 25ms
#define MAX44009_INTEGRATION_TIME_25MS                          (0x05)
// Manual mode only, 12.5ms
#define MAX44009_INTEGRATION_TIME_12MS                          (0x06)
// Manual mode only, 6.25ms
#define MAX44009_INTEGRATION_TIME_6MS                           (0x07)

#define MAX44009_INTEGRATION_TIME_AUTO                          (0x08)



/*****************************************************************************************************************************
*
*   Read specified metric's value of the BH1750 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_CONNECT on connection error
*
*****************************************************************************************************************************/

int8_t getMAX44009Metric(SoftwareWire*, uint8_t, uint8_t, const uint8_t, const uint8_t, char*);
int8_t getMAX44009Metric(SoftwareWire*, uint8_t, uint8_t, const uint8_t, const uint8_t, uint32_t*);
int8_t getMAX44009Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _integration_time, const uint8_t _metric, char *_dst, uint32_t* _value, const uint8_t _wantsNumber = false);

#endif // #ifndef _ZABBUINO_I2C_MAX44009_H_