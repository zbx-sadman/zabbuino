#ifdef FEATURE_USER_FUNCTION_PROCESSING

// System display settings
const uint8_t  constUserDisplaySDAPin                         = A4;     // SDA - A4
const uint8_t  constUserDisplaySCLPin                         = A5;     // SCL - A5
const uint8_t  constUserDisplayI2CAddress                     = 0x20;   // I2C LCD interface board address
const uint8_t  constUserDisplayBackLight                      = 0x00;   // backlight off
const uint16_t constUserDisplayType                           = 1602;   // 16x2 screen, refer to source code of printToPCF8574LCD() subroutine
const uint16_t constUserDisplayRenewInterval                  = 5000UL; // 5sec

const uint16_t constUserEEPROMReadInterval                    = 10000UL; // 10sec
const uint16_t constUserSensorsReadInterval                   = 5000UL;  // 5 sec

const uint8_t  constUserEEPROMSDAPin                          = A4;     // SDA - A4
const uint8_t  constUserEEPROMSCLPin                          = A5;     // SCL - A5
const uint8_t  constUserEEPROMI2CAddress                      = 0x56;   // I2C EEPROM address


    // AT24CXX EEPROM memory structure for atandalone mode example:
    //  Cell #0..7 - BH1750 data, SDA pin#, SCL pin#, I2C address, Mode - sensor's connection data, LightOn - how much lux need to turn room light on, 
    //               and LightOff how much need to turn light off.
    //               Note: if you will placed sensor near light bulb - it can be never turned off. 
    //
    //  Cell #0 ->[SDA pin#][SCL pin#][I2C address][Mode][LightOn HiByte][LightOn LowByte][LightOff HiByte][LightOff LowByte] <- Cell #7
    //
    //  Cell #8..19 - DS18B20 data, OneWire pin#, Resolution, ID/Address - sensor's connection data, 
    //                AlarmOn temperature in Celsius to turn alarm on, and AlarmOff to turn it off
    //                Note: AlarmOn temperature must be bigger that AlarmOn (refer to Hysteresis).
    //
    //  Cell #8 ->[OneWire pin#][Resolution][ID/Address - 8 bytes][AlarmOn temperature][AlarmOff temperature] <- Cell #19
    //
    //  Cell #20 is CRC-8 (calculated by Dallas/iButton algo)
    
#define USER_MEMORY_STRUCTURE_BH1750_SDA            0
#define USER_MEMORY_STRUCTURE_BH1750_SCL            1
#define USER_MEMORY_STRUCTURE_BH1750_I2CADDR        2
#define USER_MEMORY_STRUCTURE_BH1750_MODE           3
#define USER_MEMORY_STRUCTURE_BH1750_LIGHTON_HI     4
#define USER_MEMORY_STRUCTURE_BH1750_LIGHTON_LO     5
#define USER_MEMORY_STRUCTURE_BH1750_LIGHTOFF_HI    6
#define USER_MEMORY_STRUCTURE_BH1750_LIGHTOFF_LO    7

#define USER_MEMORY_STRUCTURE_DS18B20_OW            8
#define USER_MEMORY_STRUCTURE_DS18B20_RES           9
#define USER_MEMORY_STRUCTURE_DS18B20_ADDR          10
#define USER_MEMORY_STRUCTURE_DS18B20_ALARMON       18
#define USER_MEMORY_STRUCTURE_DS18B20_ALARMOFF      19

#define USER_MEMORY_STRUCTURE_CRC8                  20

/*****************************************************************************************************************************
*
*  Subroutine calls on start of Zabbuino
*
*****************************************************************************************************************************/
void initStageUserFunction(char* _buffer) {
  // Note that not all system struct is initialized at this stage and you can't get localIP() or localtime() info
  printToPCF8574LCD(&SoftTWI, constUserDisplayI2CAddress, constUserDisplayBackLight, constUserDisplayType, _buffer);
  _buffer[0] = 0x06;
  _buffer[1] = 0x01;
  strcpy_P(&_buffer[2], constZbxAgentVersion);
  // push data to LCD via I2C
  SoftTWI.reconfigure(constUserDisplaySDAPin, constUserDisplaySCLPin);
  printToPCF8574LCD(&SoftTWI, constUserDisplayI2CAddress, constUserDisplayBackLight, constUserDisplayType, _buffer);
}

/*****************************************************************************************************************************
*
*  Subroutine calls on every loop if no active network session exist
*
*****************************************************************************************************************************/
void loopStageUserFunction(char* _buffer) {
  const uint8_t  constVirtualScreensNum                           = 4;      // Number of report virtual screens
  uint32_t nowTime;
  static uint8_t reportVirtualScreenCnt = 0,
                 bh1750SDAPin        = 18,   // A4
                 bh1750SCLPin        = 19,   // A5
                 bh1750I2CAddress    = 0x23,
                 bh1750Mode          = 0x10,
                 ds18B20OWPin        = 6,    // D6
                 ds18B20OWResolution = 9, // 9 bit
                 ds18B20TempAlarmOn  = 30,   // 30C
                 ds18B20TempAlarmOff = 25,   // 25C
                 ds18B20OWAddr[8];

  static uint16_t bh1750LightOn       = 20,   // in Lux
                  bh1750LightOff      = 600;  // in Lux

  static uint32_t prevUserEEPROMReadTime = 0,
                  prevSensorsReadTime = 0,
                  prevUserDisplayRenewTime = 0;

  // current time
  nowTime = millis();
#ifdef FEATURE_AT24CXX_ENABLE
  // Re-read variable's value on first call and every constEEPROMReadInterval
  if ((0 == prevUserEEPROMReadTime) || (constUserEEPROMReadInterval <= (uint32_t) (nowTime - prevUserEEPROMReadTime))) {
    // Configure TWI to work on special pins
    SoftTWI.reconfigure(constUserEEPROMSDAPin, constUserEEPROMSCLPin);

    // Read 21 bytes started from cell 0 to buffer, that used as uint8_t array.
    // On success - update variable's value
    if (AT24CXXRead(&SoftTWI, constUserEEPROMI2CAddress, 0x00, 21, (uint8_t*) _buffer)) {
      // calculate readed data CRC8 for all bytes, exclude byte #USER_MEMORY_STRUCTURE_CRC8 (from 0 to USER_MEMORY_STRUCTURE_DS18B20_ALARMOFF)
      // if CRC equal data is processed
      //uint8_t calculatedCRC = dallas_crc8((uint8_t*) _buffer), USER_MEMORY_STRUCTURE_CRC8); 
      //if (calculatedCRC == _buffer[USER_MEMORY_STRUCTURE_CRC8]) 
      {
         // *** BH1750 ****
         // Change pin numbers only if it safe (see protect_pin array in src/tune.h )
         if (isSafePin(_buffer[USER_MEMORY_STRUCTURE_BH1750_SDA]) && isSafePin(_buffer[USER_MEMORY_STRUCTURE_BH1750_SCL])) {
            bh1750SDAPin     = _buffer[USER_MEMORY_STRUCTURE_BH1750_SDA];
            bh1750SCLPin     = _buffer[USER_MEMORY_STRUCTURE_BH1750_SCL];
         }
         bh1750I2CAddress = _buffer[USER_MEMORY_STRUCTURE_BH1750_I2CADDR];
         bh1750Mode       = _buffer[USER_MEMORY_STRUCTURE_BH1750_MODE];
         // make 16-byte value from two bytes
         bh1750LightOn    = _buffer[USER_MEMORY_STRUCTURE_BH1750_LIGHTON_HI] << 8 | _buffer[USER_MEMORY_STRUCTURE_BH1750_LIGHTON_LO];
         bh1750LightOff   = _buffer[USER_MEMORY_STRUCTURE_BH1750_LIGHTOFF_HI] << 8 | _buffer[USER_MEMORY_STRUCTURE_BH1750_LIGHTOFF_LO];
         DTSM ( Serial.println("BH1750 settings"); )
         DTSM ( Serial.println(bh1750SDAPin); )
         DTSM ( Serial.println(bh1750SCLPin); )
         DTSM ( Serial.println(bh1750I2CAddress, HEX); )
         DTSM ( Serial.println(bh1750Mode, HEX); )
         DTSM ( Serial.println(bh1750LightOn); )
         DTSM ( Serial.println(bh1750LightOff); )

         // *** DS18B20 ****
         // Change pin number only if it safe (see protect_pin array in src/tune.h )
         if (isSafePin(_buffer[USER_MEMORY_STRUCTURE_DS18B20_OW])) {
            ds18B20OWPin     = _buffer[USER_MEMORY_STRUCTURE_DS18B20_OW];
         }
         ds18B20OWResolution = _buffer[USER_MEMORY_STRUCTURE_DS18B20_RES];
         // just copy 8 bytes of DS18B20 ID starting from buffer[USER_MEMORY_STRUCTURE_DS18B20_ADDR] position to other byte array
         memcpy(ds18B20OWAddr, &_buffer[USER_MEMORY_STRUCTURE_DS18B20_ADDR], 8);
         ds18B20TempAlarmOn  = _buffer[USER_MEMORY_STRUCTURE_DS18B20_ALARMON];
         ds18B20TempAlarmOff = _buffer[USER_MEMORY_STRUCTURE_DS18B20_ALARMOFF];      
         DTSM ( Serial.println("DS18B20 settings"); )
         DTSM ( Serial.println(ds18B20OWPin); )
         DTSM ( Serial.println(ds18B20OWResolution); )
         DTSM ( Serial.println(ds18B20AlarmOn); )
         DTSM ( Serial.println(ds18B20AlarmOff); )
      } // if (calculatedCRC == _buffer[USER_MEMORY_STRUCTURE_CRC8]) 
    } // if (AT24CXXRead(&SoftTWI ...

    prevUserEEPROMReadTime = nowTime;
  } // if ((0 == prevUserEEPROMReadTime)
#endif

  // Read&analyze sensor values on first call and every constUserSensorsReadInterval
  if ((0 == prevSensorsReadTime) || (constUserSensorsReadInterval <= (uint32_t) (nowTime - prevSensorsReadTime))) {
     // *** BH1750 ****
     uint32_t light;
     SoftTWI.reconfigure(bh1750SDAPin, bh1750SCLPin);
     if (RESULT_IN_BUFFER == getBH1750Metric(&SoftTWI, bh1750I2CAddress, bh1750Mode, SENS_READ_LUX, &light)) {
        // Returned scaled value is bigger than real, make it normal. For BH1750 normal_lux = scaled_lux / 100;
        light = light / 100;
        DTSM ( Serial.println("BH1750 value (lux): "); )
        DTSM ( Serial.println(light); )
        // if (bh1750LightOn < light) { 
        //   Turn light on   
        // }
     }

     // *** DS18B20 ****
     int32_t temperature;
     if (RESULT_IN_BUFFER == getDS18X20Metric(ds18B20OWPin, ds18B20OWResolution, ds18B20OWAddr, &temperature)) {
        // Returned scaled value is bigger than real, make it normal. For DS18x20 normal_temp = scaled_temp / 1000;
        temperature = temperature / 10000;
        DTSM ( Serial.println("DS18B20 whole part of temp (C): "); )
        DTSM ( Serial.println(temperature); )
        // if (ds18B20AlarmOff > temperature) { 
        //   Turn alarm off
        // }
        // if (ds18B20AlarmOn < temperature) { 
        //   Turn alarm on   
        // }
     }    
    prevSensorsReadTime = nowTime;
  }

  //Show virtual screens
#ifdef FEATURE_USER_DISPLAY_ENABLE

//        LCD1602
//     +- C0 .... CF-+
//     v             v
// R1 |XXX.XXX.XXX.XXX |  << IP Address
// R2 |4294967295 ms   |  << uptime
//
//    |Zabbuino 1.2.3  |  <<
//    |     ALL OK     |  <<
//
// R1, R2 - row #0x1, row #0x2
// C0..RF - col #0x0 .. col #0xF
//
//        0                                                         N
//  _buffer |--Zabbuino 1.2.3--ALL OK                                   |
//        ^^              ^^
//  0x01 -++- 0x06  '\n' -++-'\t'
//
//  N - constBufferSize (tune.h)
//
//  !!! Resulted string must be shorter that constBufferSize to avoid mailfunction !!!
//

uint8_t dataLength;
  uint32_t timestamp;
  tm dateTime;

  // do nothing if renew wait time is not expiried
  if (constUserDisplayRenewInterval > (uint32_t) (nowTime - prevUserDisplayRenewTime)) {
    return;
  }

  prevUserDisplayRenewTime = nowTime;

  // buffer can be nulled to help detects EOL by '\0' at end of string
  // how fast memset?
  memset(_buffer, 0x00, constBufferSize + 1);
  reportVirtualScreenCnt++;
  if (constVirtualScreensNum <= reportVirtualScreenCnt) {
    reportVirtualScreenCnt = 0;
  }

  // Write to _buffer commands for output direction (0x06) and clear screen (0x01)
  // Note: strcpy() place "\x6\x1" to _buffer as bytes with values 6 and 1 (_buffer[0]=6, _buffer[1]=1), not as C-string '\x6\x1'
  _buffer[0] = 0x06;
  _buffer[1] = 0x01;
  dataLength = 2;

  switch (reportVirtualScreenCnt) {
    case 0x00:
      // copy constZbxAgentVersion variable (see "AGENT CONFIGURATION SECTION" in basic.h) content to src, starting from [dataLength] cell
      strcpy_P(&_buffer[dataLength], constZbxAgentVersion);
      // _buffer is C-string ('\0' terminated) and we can get it length with strlen() for correcting dataLength.
      dataLength = strlen(_buffer);
      // replace to '\0' by '\n' at the end position of string to have newline on physical screen
      _buffer[dataLength++] = '\n';
      // copy "Uptime: " string to src, following '\n'
      strcpy_P(&_buffer[dataLength], PSTR("Up: "));
      dataLength += 4;
      // write millis() value to the _buffer starting from [dataLength] cell (take address of (_buffer[0] + dataLength) and give ltoa() as buffer);
      //ultoa(millis(), &_buffer[dataLength], 10);
      //dataLength = strlen(_buffer);
      //strcpy_P(&_buffer[dataLength], PSTR(" ms"));
      // timestamp must be in seconds, not ms
      timestamp = (millis() / 1000);
      gmtime_r(&timestamp, &dateTime);
      dateTime.tm_yday--;
      // need to add tm::tm_yday directly to _buffer to avoid 'Up: 001D ...' at the start
      strftime(&_buffer[dataLength], 30, "%jD %T", &dateTime);
      //timestampToDataTimeStr(timestamp, &_buffer[dataLength]);

      break;

    case 0x01:
      strcpy_P(&_buffer[dataLength], PSTR("VCC:"));
      // Move write position to 4 char ('VCC:')
      dataLength += 4;
      // write unit32_t value to the _buffer starting from [dataLength] cell;
      // getADCVoltage - internal function, that return actual MCU voltage
      ultoa(getADCVoltage(ANALOG_CHAN_VBG), &_buffer[dataLength], 10);
      // voltage in mv, usually take 4 char
      dataLength += 4;
      strcpy_P(&_buffer[dataLength], PSTR("mV Mem:"));
      dataLength += 7;
      // write unit32_t value to the _buffer starting from [dataLength] cell;
      // sysMetrics.sysRamFree - internal metric (refer to structs.h)
      ultoa(sysMetrics.sysRamFree, &_buffer[dataLength], 10);
      // Now len of string unknown - sysRamFree may take 3 char or 2...
      dataLength = strlen(_buffer);
      _buffer[dataLength++] = 'b';
      // Go to new line
      _buffer[dataLength++] = '\n';
      strcpy_P(&_buffer[dataLength], PSTR("Idle: "));
      dataLength += 6;
      //ultoa((millis()-sysMetrics.sysCmdLastExecTime), &_buffer[dataLength], 10);
      // len of string unknown again
      //dataLength = strlen(_buffer);
      //strcpy_P(&_buffer[dataLength], PSTR(" ms"));
      //dataLength += 3;
      timestamp = ((millis() - sysMetrics.sysCmdLastExecTime) / 1000);
      gmtime_r(&timestamp, &dateTime);
      dateTime.tm_yday--;
      strftime(&_buffer[dataLength], 30, "%jD %T", &dateTime);

      break;

    case 0x02:

      time_t y2kts;
      // DS3231 RTC operate by Y2K-based timestamp
      if (RESULT_IS_OK == getY2KTime(&SoftTWI, &y2kts)) {
        localtime_r(&y2kts, &dateTime);
        strcpy_P(&_buffer[dataLength], PSTR("\t\tTIME\n"));
        dataLength += 7;
        strftime(&_buffer[dataLength], 30, "%d/%m/%Y %T", &dateTime);
      }

      break;

    case 0x03:
      strcpy_P(&_buffer[dataLength], PSTR("Light (lux): "));
      // 'Light (lux): ' - 13 chars,
      dataLength += 13;
      // I2C sensor is connected on D2 (SDA) & D3 (SCL). Reconfigure global Software I2C Interface and put light value to output buffer
      SoftTWI.reconfigure(bh1750SDAPin, bh1750SCLPin);
      //uservalue = WANTS_VALUE_NONE;
      // RESULT_IN_BUFFER mean that conversion is finished sucessfully
      // uservalue must be used by reference (with "take address" operation - &) because getBH1750Metric put some data to it
      if (RESULT_IN_BUFFER != getBH1750Metric(&SoftTWI, bh1750I2CAddress, bh1750Mode, SENS_READ_LUX, &_buffer[dataLength])) {
         Serial.println("BH read error");
        // Need to do something if conversion isn't success. May be put "error" word into _buffer?
      };


      break;

    case 0x04:
      break;
    // build your own screen and modify constVirtualScreensNum's value on top of this subroutine

    default:
      // write empty string
      break;
  }
  //dataLength = strlen(_buffer);
  //_buffer[dataLength] = '\0';
  // push data to LCD via I2C
  SoftTWI.reconfigure(constUserDisplaySDAPin, constUserDisplaySCLPin);
  printToPCF8574LCD(&SoftTWI, constUserDisplayI2CAddress, constUserDisplayBackLight, constUserDisplayType, _buffer);
#endif //FEATURE_USER_DISPLAY_ENABLE

}

#endif // FEATURE_USER_FUNCTION_PROCESSING
