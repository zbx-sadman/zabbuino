#ifndef ZabbuinoI2C_BUS_h
#define ZabbuinoI2C_BUS_h

#include <Arduino.h>
// Wire lib for I2C sensors
#include <Wire.h>
#include "../zabbuino.h"
#include "defaults.h"
#include "service.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                      COMMON I2C SECTION
*/

#define WireToU8(_source)  ((uint8_t) _source[0])
#define WireToS8(_source)  ((int8_t) _source[0])

#define WireToU16(_source)  ((uint16_t) ( ((uint16_t) _source[0] << 8)| _source[1]))
#define WireToS16(_source)  ((int16_t) ( ((uint16_t) _source[0] << 8)| _source[1]))

#define WireToU16LE(_source)  ((uint16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))
#define WireToS16LE(_source)  ((int16_t) ( ((uint16_t) _source[1] << 8)| _source[0]))

#define WireToU24(_source)  ((uint32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))
#define WireToS24(_source)  ((int32_t) ( ((uint32_t) _source[0] << 16) | (_source[1] << 8) | _source[2]))

int8_t scanI2C(EthernetClient *_ethClient);

uint8_t writeByteToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t _data);
uint8_t writeBytesToI2C(const uint8_t _i2cAddress, const int16_t _registerAddress, const uint8_t* _data, uint8_t _len);
uint8_t readBytesFromi2C(const uint8_t _i2cAddress, const int16_t _registerAddress, uint8_t* _dst, const uint8_t _len);

/* ****************************************************************************************************************************
*
*  Ping I2C slave
*
**************************************************************************************************************************** */
uint8_t inline isI2CDeviceReady(uint8_t _i2cAddress)
{
  //
  Wire.beginTransmission(_i2cAddress);
  return (0 == Wire.endTransmission(true));
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                     BH1750 SECTION
*/


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


int8_t getBH1750Metric(const uint8_t _sdaPin, const uint8_t _sclPin, uint8_t _i2cAddress, uint8_t _mode, const uint8_t _metric, char* _dst);

#endif