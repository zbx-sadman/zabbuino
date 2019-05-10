#pragma once
#include "SoftwareWire/SoftwareWire.h"

/*
*/

#define SGP30_I2C_ADDRESS                                       (0x58)

#define SGP30_CMD_IAQ_INIT                                      (0x0320)
#define SGP30_CMD_IAQ_MEASURE                                   (0x0820)

#define SGP30_TIME_IAQ_INIT                                     (10)
#define SGP30_TIME_IAQ_MEASURE                                  (12)

#define SGP30_CO2E_DATA_BYTE                                    (0x00)
#define SGP30_CO2E_CRC_BYTE                                     (0x02)
#define SGP30_TVOC_DATA_BYTE                                    (0x03)
#define SGP30_TVOC_CRC_BYTE                                     (0x05)


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
int8_t getSGP30Metric(SoftwareWire&, const uint8_t, const uint8_t, const uint8_t, char*);
int8_t getSGP30Metric(SoftwareWire&, const uint8_t, const uint8_t, const uint8_t, uint32_t*);
