// Config & common included files
#include "sys_includes.h"

// OneWire lib for Dallas sensors
#include <OneWire.h>

#include "service.h"
#include "system.h"
#include "network.h"

#include "ow_bus.h"
#include "ow_sensors.h"
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                                   DS18x20 SECTION

   Based on: Dallas Temperature Control Library
   Version 3.7.2  is used

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

/*****************************************************************************************************************************
*
*  Read DS18x20's scratchpad
*
*   Returns: 
*     - true on success
*     - false on fail
*
*****************************************************************************************************************************/
static uint8_t getScratchPadFromDevice(OneWire *_owDevice, const uint8_t *_addr, uint8_t *_scratchPad) {
  uint8_t i, busReady;
  // send the reset command and fail fast
  // reset() returns 1 if a device asserted a presence pulse, 0 otherwise.
  busReady = _owDevice->reset();
  if (0 == busReady) { return false; }

  _owDevice->select(_addr);
  _owDevice->write(DS18X20_CMD_READSCRATCH);
  for (i = 0; i < 9; i++) {
    _scratchPad[i] = _owDevice->read();
  }

  busReady = _owDevice->reset();
  return (1 == busReady);
}


static inline uint8_t isCRCOK(uint8_t *_scratchPad) {
  // '==' here not typo
  return (dallas_crc8(_scratchPad, 8) == _scratchPad[DS18X20_BYTE_SCRATCHPAD_CRC]);
}


/*****************************************************************************************************************************
*
*  Overloaded getDS18X20Metric() functions. Refer to original subroutione to see return codes
*
*****************************************************************************************************************************/
int8_t getDS18X20Metric(const uint8_t _pin, uint8_t _resolution, uint8_t* _id, int32_t* _value)
{
  char stubBuffer;
  return getDS18X20Metric(_pin, _resolution, _id, &stubBuffer, _value, true);

}

int8_t getDS18X20Metric(const uint8_t _pin, uint8_t _resolution, uint8_t* _id, char* _dst)
{
  int32_t stubValue;
  return getDS18X20Metric(_pin, _resolution, _id, _dst, &stubValue, false);
}



/*****************************************************************************************************************************
*
*  Read specified metric's value of the digital sensor of Dallas DS18x20 family, put it to output buffer on success. 
*
*  Note: subroutine is tested with DS18B20 only. 
*        probably you can meet problems with the correct calculation of temperature due to incorrect 'tRaw' adjustment 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on connection error
*     - DEVICE_ERROR_CHECKSUM on detect data corruption
*
*****************************************************************************************************************************/
int8_t getDS18X20Metric(const uint8_t _pin, uint8_t _resolution, uint8_t* _id, char* _dst, int32_t* _value, const uint8_t _wantsNumber)
//int8_t getDS18X20Metric(const uint8_t _pin, uint8_t _resolution, char* _id, char* _dst)
{
  int8_t rc = DEVICE_ERROR_CONNECT;
  uint8_t signBit, parasitePowerUsed, busReady, resolutionIdx;
  uint8_t* scratchPad;
  int16_t conversionTime;
  int32_t tRaw;
  uint8_t resolutionCodes[] = {DS18X20_MODE_9_BIT, DS18X20_MODE_10_BIT, DS18X20_MODE_11_BIT, DS18X20_MODE_12_BIT};
  uint8_t noiseBits[] = {7, 3, 1, 0};

  // Start mass conversion or read data if prev conversion has been finished no more that N sec is good idea, but need to store link busPin<->prevConversionTime
  // static uint32_t prevConversionTime = 0;

  OneWire owDevice(_pin);

  scratchPad = (uint8_t*) _dst;

  if ('\0' == _id[0]) {
    // If ID not given - search any sensor on OneWire bus and use its. Or return error when no devices found.
    owDevice.reset_search();
    // If search() function returns a '1' then it has enumerated the next device and you may retrieve the ROM from the
    // OneWire::address variable. If there are no devices, no further devices, or something horrible happens in the middle of the
    // enumeration then a 0 is returned. (R) OneWire.cpp
    if (0x00 == owDevice.search(_id)) {
      goto finish;  // rc already init with DEVICE_ERROR_CONNECT value
    }
  } else {
    if (dallas_crc8(_id, 7) != _id[7]) {
      goto finish;  // rc already init with DEVICE_ERROR_CONNECT value
    }
  }

  busReady = getScratchPadFromDevice(&owDevice, _id, scratchPad);
  if (!busReady) {
    rc = DEVICE_ERROR_CONNECT;
    goto finish;
  } else if (!isCRCOK(scratchPad)) {
    rc = DEVICE_ERROR_CHECKSUM;
    goto finish;
  }

  switch (_id[0]) {
    //  DS1820 and DS18S20 have no CONFIGURATION registry and work in 9-bit mode only
    case DS18S20_ID:
      _resolution = DS18X20_MIN_RESOLUTION;
      break;
    case DS18B20_ID:
    case DS1822_ID:
    case DS1825_ID:
      // Resolution must be: 9 <= _resolution <= 12
      _resolution = constrain(_resolution, DS18X20_MIN_RESOLUTION, DS18X20_MAX_RESOLUTION);
      break;
    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish;
  }

  resolutionIdx = _resolution - DS18X20_MIN_RESOLUTION;

  // Detect power on sensor - parasite or not
  parasitePowerUsed = false;
  owDevice.reset();
  owDevice.select(_id);
  owDevice.write(DS18X20_CMD_READPOWERSUPPLY);
  if (0x00 == owDevice.read_bit()) {
    parasitePowerUsed = true;
  }

  // Sensor already configured to use '_resolution'? Do not make write operation.
  if (DS18S20_ID != _id[0] && scratchPad[DS18X20_BYTE_CONFIGURATION] != resolutionCodes[resolutionIdx]) {
    scratchPad[DS18X20_BYTE_CONFIGURATION] = resolutionCodes[resolutionIdx];
    owDevice.reset();
    owDevice.select(_id);
    owDevice.write(DS18X20_CMD_WRITESCRATCH);
    // Change only 3 byte. That is enough to sensor's resolution change
    owDevice.write(scratchPad[DS18X20_BYTE_HIGH_ALARM_TEMP]);
    owDevice.write(scratchPad[DS18X20_BYTE_LOW_ALARM_TEMP]);
    owDevice.write(scratchPad[DS18X20_BYTE_CONFIGURATION]);

    // When sensor used 'parasite' power settings must be copied to DS's EEPROM.
    // Otherwise it will be lost on reset()
    // I do not think that need to make COPYSCRATCH everytime to prolong sensor lifetime, because often measurement on single sensor
    // with various resolution may be requested.
    if (parasitePowerUsed) {
      owDevice.write(DS18X20_CMD_COPYSCRATCH, 1);
      // 20ms delay allow 10ms long EEPROM write operation (as specified by datasheet),
      // additional 10ms delay used in Dallas lib for parasite powered sensors
      delay(20+10);
    }
  } // if (DS18S20_ID != _id[0] && scratchPad[DS18X20_BYTE_CONFIGURATION] != resolutionCode)

  // From Dallas's datasheet:
  //        12 bit resolution, 750 ms conversion time
  //        11 bit res, 375 ms
  //        10 bit res, 187.5 ms
  //        09 bit res, 93.75 ms
  // conversionTime = (tCONV + 10%) / (2 ^ (12 [bit resolution]- N [bit resolution])). 12bit => 750ms+10%, 11bit => 375ms+10% ...
  // For some DS sensors you may need increase tCONV to 1250ms or more
  conversionTime = 825 / (1 << (12 - _resolution));

  // Temperature read begin
  owDevice.reset();
  owDevice.select(_id);
  // start conversion, with parasite power on at the end
  owDevice.write(DS18X20_CMD_STARTCONVO, 1);
  // Wait to end conversion
  delay(conversionTime);

  // Read data from DS's ScratchPad or return 'Error' on failure
  // DEVICE_ERROR_CHECKSUM instead DEVICE_ERROR_CONNECT returned because sensor is probaly presented on 1-Wire bus (already tested early)
  busReady = getScratchPadFromDevice(&owDevice, _id, scratchPad);
  if (!busReady) {
    rc = DEVICE_ERROR_CONNECT;
    goto finish;
  } else if (!isCRCOK(scratchPad)) {
    rc = DEVICE_ERROR_CHECKSUM;
    goto finish;
  }
  // Temperature calculation
  tRaw = (((int32_t) scratchPad[DS18X20_BYTE_TEMP_MSB]) << 8) | scratchPad[DS18X20_BYTE_TEMP_LSB];

  // For some DS's models temperature value must be corrected additional
  if (DS18S20_ID == _id[0])
  {
    // DS18S20 & DS1820 have no more 9 bit resolution, higher bits can be dropped
    tRaw = tRaw << 3;
    // DS18S20 have 0x10 in COUNT_PER_C register. Test its for DS18S20 detecting.
    if (scratchPad[DS18X20_BYTE_COUNT_PER_C] == 0x10) {
      // Additional bits will be used in temperature calculation with 12-bit resolution
      // if (TEMP_12_BIT == _resolution) ???
      tRaw = (tRaw & 0xFFF0) + 12 - scratchPad[DS18X20_BYTE_COUNT_REMAIN];
    }
  }
  else
  {
    // Drop unused (noise) bits for DS18B20
    // it may bring wrong results if DS18X20 reconfiguration failed (resolution does not set properly)
    tRaw = tRaw & ~noiseBits[resolutionIdx];
  }
  // Negative value of temperature testing
  // if (signBit) ...
  signBit = false;
  if (tRaw & 0x8000)
  {
    signBit = true;
    // Some magic passes are need to get correct temperature value
    // Refer to DS18B20's datasheet,  Table 1. Temperature/Data Relationship
    tRaw = (tRaw ^ 0xffff) + 1; // 2's comp
  }

  // Do 'unfloat' procedure for using number with my ltoaf() subroutine: multiply temp to 0.0625 (1/16 C)
  tRaw *= 100;
  tRaw = ((tRaw * 6) + (tRaw / 4));
  if (signBit) {
    tRaw = -tRaw;
  }

  *_value = tRaw; 

  if (!_wantsNumber) {
     ltoaf(tRaw, _dst, 4);
  } 

  rc = RESULT_IS_BUFFERED;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}


