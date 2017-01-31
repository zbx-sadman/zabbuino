#ifndef _ZABBUINO_I2C_BUS_H_
#define _ZABBUINO_I2C_BUS_H_

// Wire lib for I2C sensors
#include <Wire.h>
#include "../basic.h"
#include "tune.h"
#include "service.h"
#include "system.h"
#include "transport.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                      COMMON I2C SECTION

 -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

#define WireToU8(_source)  ((uint8_t) _source[0])
#define WireToS8(_source)  ((int8_t) _source[0])

#define WireToU16(_source)  ((uint16_t) ( ((uint16_t) _source[0] << 8)| _source[1]))
#define WireToS16(_source)  ((int16_t) ( ((uint16_t) _source[0] << 8)| _source[1]))

#define WireToU16LE(_source)  ((uint16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))
#define WireToS16LE(_source)  ((int16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))

#define WireToU24(_source)  ((uint32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))
#define WireToS24(_source)  ((int32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))

/*****************************************************************************************************************************
*
*   Scan I2C bus and print to ethernet client addresses of all detected devices 
*
*   Returns: 
*     - RESULT_IS_PRINTED on success
*     - RESULT_IS_FAIL of no devices found 
*
*****************************************************************************************************************************/
int8_t scanI2C(EthernetClient *_ethClient);

/*****************************************************************************************************************************
*
*   Send one byte to writeBytesToI2C() subroutine
*
*   Returns: 
*     - writeBytesToI2C()'s result code
*
*****************************************************************************************************************************/
uint8_t writeByteToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _src);

/*****************************************************************************************************************************
*
*   Write incoming bytes to I2C device register (if specified) or just to device
*
*   Returns: 
*     - Wire.endTransmission result code
*       0 - success
*       1 - data too long to fit in transmit buffer
*       2 - received NACK on transmit of address
*       3 - received NACK on transmit of data
*       4 - other error
*
*****************************************************************************************************************************/
uint8_t writeBytesToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t *_src, uint8_t _len);

/*****************************************************************************************************************************
*
*   Reads bytes from device's register (or not) over I2C.
*
*   Returns: 
*     - Wire.endTransmission result code
*       0 - success
*       1 - data too long to fit in transmit buffer
*       2 - received NACK on transmit of address
*       3 - received NACK on transmit of data
*       4 - other error
*
*
*****************************************************************************************************************************/
uint8_t readBytesFromi2C(const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t *_dst, const uint8_t _len);

/*****************************************************************************************************************************
*
*   Ping I2C slave device
*
*   Returns: 
*     - Wire.endTransmission result code
*       0 - success
*       1 - data too long to fit in transmit buffer
*       2 - received NACK on transmit of address
*       3 - received NACK on transmit of data
*       4 - other error
*
*
*****************************************************************************************************************************/
uint8_t inline isI2CDeviceReady(uint8_t _i2cAddress)
{
  //
  Wire.beginTransmission(_i2cAddress);
  return (0 == Wire.endTransmission(true));
}

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                                     BH1750 SECTION 
                                                                    (Can be removed)

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/


#define BH1750_I2C_FIRST_ADDRESS                                0x23
#define BH1750_I2C_SECOND_ADDRESS                               0x5C
/*

  Datasheet: http://rohmfs.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1750fvi-e.pdf
*/

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
int8_t getBH1750Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _metric, char *_dst);

#endif // #ifndef _ZABBUINO_I2C_BUS_H_