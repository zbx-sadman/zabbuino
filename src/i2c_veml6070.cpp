// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_veml6070.h"


/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getVEML6070Metric(SoftwareWire* _softTWI, const uint8_t _integrationTime, const uint8_t _metric, uint32_t* _value)
{
  char stubBuffer;
  return getVEML6070Metric(_softTWI, _integrationTime, &stubBuffer, _value, true);

}

int8_t getVEML6070Metric(SoftwareWire* _softTWI, uint8_t _integrationTime, const uint8_t _metric, char* _dst)
{
  uint32_t stubValue;
  return getVEML6070Metric(_softTWI, _integrationTime, _dst, &stubValue, false);
}



/*****************************************************************************************************************************
*
*   Read specified metric's value of the VEML6070 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on test connection error
*     - RESULT_IS_FAIL - on other fails
*
*****************************************************************************************************************************/
int8_t getVEML6070Metric(SoftwareWire* _softTWI, const uint8_t _integrationTime, char* _dst, uint32_t* _value, const uint8_t _wantsNumber)
{
  int8_t rc = RESULT_IS_FAIL;
  uint8_t value[2];
  // VEML6070_1_T is default init value
  static uint8_t integrationTime = VEML6070_1_T;

  if (!isI2CDeviceReady(_softTWI, VEML6070_ADDR_L)) {
    rc = DEVICE_ERROR_CONNECT;
    goto finish;
  }

  // Initialization
  // VEML6070 needs to be initialized while the system’s power is on. The initialization includes two major steps: (1) clear ACK state
  // of UVS and (2) fill the initial value, 06 (HEX), into the 0x70 addresses. After the initialization is completed, VEML6070 can be
  // programmable for operation by write command setting from the host. VEML6070 initialization is recommended to be completed within 150 ms.
  if (millis() <= VEML6070_INIT_TIME_MS) {
    // I think that confition is never run, because init stage of main code is longer that VEML6070_INIT_TIME_MS
    delay(VEML6070_INIT_TIME_MS);
  }

  if (_integrationTime != integrationTime) {
    integrationTime = _integrationTime;
    // Only next conversion will be affected on changing integration time
    // For example: correct result can be obtained after 125ms if sensor switched to VEML6070_1_T mode.
    uint8_t configReg = (integrationTime << 2);
    if (0x00 != writeByteToI2C(_softTWI, VEML6070_ADDR_L, I2C_NO_REG_SPECIFIED, configReg)) {
      goto finish;
    }
  }

  if (0x01 != readBytesFromI2C(_softTWI, VEML6070_ADDR_H, I2C_NO_REG_SPECIFIED, &value[0], 0x01)) {
    goto finish;
  }
  if (0x01 != readBytesFromI2C(_softTWI, VEML6070_ADDR_L, I2C_NO_REG_SPECIFIED, &value[1], 0x01)) {
    goto finish;
  }

  *_value = WireToU16(value);
  if (!_wantsNumber) {
    ltoa(*_value, _dst, 10);
  }

  rc = RESULT_IS_BUFFERED;

  gatherSystemMetrics(); // Measure memory consumption

finish:
  return rc;

}
