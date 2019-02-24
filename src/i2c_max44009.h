#pragma once
/*

  Datasheet: http://datasheets.maximintegrated.com/en/ds/MAX44009.pdf

*/

#define MAX44009_I2C_ADDRESS                                    (0x4A)

#define MAX44009_REG_CONFIGURATION                              (0x02)
#define MAX44009_REG_LUXREADING                                 (0x03)

#define MAX44009_800MS_CYCLE_MODE                               (0x00)
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
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getMAX44009Metric(SoftwareWire*, uint8_t, uint8_t, const uint8_t, const uint8_t, char*);
int8_t getMAX44009Metric(SoftwareWire*, uint8_t, uint8_t, const uint8_t, const uint8_t, uint32_t*);


/*****************************************************************************************************************************
*
*   Read specified metric's value of the BH1750 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on test connection error
*     - RESULT_IS_FAIL - on other fails
*
*****************************************************************************************************************************/
int8_t getMAX44009Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _integration_time, const uint8_t _metric, char *_dst, uint32_t* _value, const uint8_t _wantsNumber = false);

