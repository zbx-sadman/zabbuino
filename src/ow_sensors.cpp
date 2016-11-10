#include "ow_sensors.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                                   DS18x20 SECTION

 -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/


/*****************************************************************************************************************************
*
*  Read specified metric's value of the digital sensor of Dallas DS18x20 family, put it to output buffer on success. 
*
*  Note: subroutine is tested with DS18B20 only. 
*        probably you can meet problems with the correct calculation of temperature due to incorrect 'tRaw' adjustment 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_CONNECT on connection error
*     - DEVICE_ERROR_CHECKSUM on detect data corruption
*
*****************************************************************************************************************************/
int8_t getDS18X20Metric(const uint8_t _pin, uint8_t _resolution, char* _id, char* _dst)
{
  uint8_t i, signBit, dsAddr[8], scratchPad[9], parasitePowerUsed;
  int16_t conversionTimeout;
  uint32_t tRaw;
  // Start mass conversion or read data if prev conversion has been finished no more that N sec is good idea, but need to store link busPin<->prevConversionTime
  // static uint32_t prevConversionTime = 0;

  // Resolution must be: 9 <= _resolution <= 12 
  _resolution = constrain(_resolution, 9, 12);

  OneWire* owDevice;
  owDevice = new OneWire(_pin);
  

  if ('\0' == _id[0]) {
     // If ID not valid - search any sensor on OneWire bus and use its. Or return error when no devices found.    
     owDevice->reset_search();
     if (!owDevice->search(dsAddr)) { delete owDevice; return DEVICE_ERROR_CONNECT;}
  } else {
     // Convert sensor ID (if its given) from HEX string to byte array (DeviceAddress structure). 
     // Sensor ID is equal DeviceAddress.
     // if converting not sucessfully - return DEVICE_ERROR_CONNECT because no sensor address here
     if (!hstoba((uint8_t*) dsAddr, _id, 8)) {delete owDevice; return DEVICE_ERROR_CONNECT;}
  }

  // Validate sensor. Model id saved in first byte of DeviceAddress (sensor ID).
  if (DS18S20_ID != dsAddr[0] && DS18B20_ID != dsAddr[0] && DS1822_ID != dsAddr[0]) {
    delete owDevice;  return DEVICE_ERROR_CONNECT;}

  // Get values of CONFIGURATION, HIGH_ALARM_TEMP, LOW_ALARM_TEMP registers via ScratchPad reading.
  // Or return error if bad CRC detected
  if (!getScratchPadFromDS18X20(owDevice, dsAddr, scratchPad)) {
    delete owDevice;  return DEVICE_ERROR_CHECKSUM; }

  // Detect power on sensor - parasite or not
  parasitePowerUsed = false;
  owDevice->reset();
  owDevice->select(dsAddr);
  owDevice->write(DS18X20_CMD_READPOWERSUPPLY);
  if (owDevice->read_bit() == 0) parasitePowerUsed = true;

  // Sensor already configured to use '_resolution'? Do not make write operation. 
  if (scratchPad[DS18X20_BYTE_CONFIGURATION] != _resolution) {
     //  DS1820 and DS18S20 have not CONFIGURATION registry, resolution setting have no sense
     if (DS18B20_ID == dsAddr[0]) {
        switch (_resolution) {
          case 12:
            scratchPad[DS18X20_BYTE_CONFIGURATION] = DS18X20_MODE_12_BIT;
            break;
          case 11:
            scratchPad[DS18X20_BYTE_CONFIGURATION] = DS18X20_MODE_11_BIT;
            break;
          case 10:
            scratchPad[DS18X20_BYTE_CONFIGURATION] = DS18X20_MODE_10_BIT;
            break;
          case 9:
          default:
            scratchPad[DS18X20_BYTE_CONFIGURATION] = DS18X20_MODE_9_BIT;
            break;
        }

        owDevice->reset();
        owDevice->select(dsAddr);
        owDevice->write(DS18X20_CMD_WRITESCRATCH);
        // Change only 3 byte. That is enough to sensor's resolution change
        for ( i = DS18X20_BYTE_HIGH_ALARM_TEMP; i <= DS18X20_BYTE_CONFIGURATION; i++) {
          owDevice->write(scratchPad[i]); // configuration
        }
        // When sensor used 'parasite' power settings must be copied to DS's EEPROM.
        // Otherwise its will be lost on 'ow.reset()' operation
        if (parasitePowerUsed) {
           owDevice->write(DS18X20_CMD_COPYSCRATCH, 1);
           delay(11);
       }
     } // if (DS18B20MODEL == dsAddr[0])
  } // if (scratchPad[CONFIGURATION] != _resolution)

  // From Dallas's datasheet:
  //        12 bit resolution, 750 ms conversion time
  //        11 bit res, 375 ms
  //        10 bit res, 187.5 ms
  //        09 bit res, 93.75 ms
  // conversionTimeout = (tCONV + 10%) / (2 ^ (12 [bit resolution]- N [bit resolution])). 12bit => 750ms+10%, 11bit => 375ms+10% ...
  // For some DS sensors you may need increase tCONV to 1250ms or more
  conversionTimeout = 825 / (1 << (12 - _resolution));
  
  // Temperature read begin
  owDevice->reset();
  owDevice->select(dsAddr);
  // start conversion, with parasite power on at the end
  owDevice->write(DS18X20_CMD_STARTCONVO, 1); 
  // Wait to end conversion
  delay(conversionTimeout);

  // Read data from DS's ScratchPad or return 'Error' on failure
 if (!getScratchPadFromDS18X20(owDevice, dsAddr, scratchPad)) {
    delete owDevice;  return DEVICE_ERROR_CHECKSUM;}

  // Temperature calculation
  tRaw = (((int16_t) scratchPad[DS18X20_BYTE_TEMP_MSB]) << 8) | scratchPad[DS18X20_BYTE_TEMP_LSB];

  // For some DS's models temperature value must be corrected additional 
  if (DS18S20_ID == dsAddr[0])
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
    switch (scratchPad[DS18X20_BYTE_CONFIGURATION])
    {
      case DS18X20_MODE_9_BIT:
        tRaw = tRaw & ~7;
        break;
      case DS18X20_MODE_10_BIT:
        tRaw = tRaw & ~3;
        break;
      case DS18X20_MODE_11_BIT:
        tRaw = tRaw & ~1;
        break;
      case DS18X20_MODE_12_BIT:
      default:
        break;
    }
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

  ltoaf(tRaw, _dst, 4);
  delete owDevice; 

  return RESULT_IN_BUFFER;
}

/*****************************************************************************************************************************
*
*  Read DS18x20's scratchpad
*
*   Returns: 
*     - true on success
*     - false on fail
*
*****************************************************************************************************************************/
static  uint8_t getScratchPadFromDS18X20(OneWire* _owDevice, const uint8_t* _addr, uint8_t* _scratchPad) {
  uint8_t i;
  _owDevice->reset();
  _owDevice->select(_addr);
  _owDevice->write(DS18X20_CMD_READSCRATCH);
  for (i = 0; i < 9; i++) {
    _scratchPad[i] = _owDevice->read();
  }
  // CRC checking and return result
  return (dallas_crc8(_scratchPad, 8) == _scratchPad[DS18X20_BYTE_SCRATCHPAD_CRC]);

}

