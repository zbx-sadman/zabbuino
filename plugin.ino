// System display settings
const uint8_t  constUserDisplaySDAPin                         = A4;     // SDA - A4
const uint8_t  constUserDisplaySCLPin                         = A5;     // SCL - A5
const uint8_t  constUserDisplayI2CAddress                     = 0x20;   // I2C LCD interface board address
const uint8_t  constUserDisplayBackLight                      = 0x00;   // backlight off
const uint16_t constUserDisplayType                           = 1602;   // 16x2 screen, refer to source code of printToPCF8574LCD() subroutine
const uint16_t constUserDisplayRenewInterval                  = 5000UL; // 5sec

const uint16_t constUserEEPROMReadInterval                    = 10000UL; // 10sec

const uint8_t  constUserEEPROMSDAPin                          = A4;     // SDA - A4
const uint8_t  constUserEEPROMSCLPin                          = A5;     // SCL - A5
const uint8_t  constUserEEPROMI2CAddress                      = 0x56;   // I2C EEPROM address


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

void loopStageUserFunction(char* _buffer) {
  const uint8_t  constVirtualScreensNum                           = 4;      // Number of report virtual screens
  uint32_t nowTime;
  uint32_t uservalue;
  static uint8_t reportVirtualScreenCnt = 0,
                 bh1750SDAPin     = 18, // A4 
                 bh1750SCLPin     = 19, // A5
                 bh1750I2CAddress = 0x23,
                 bh1750Mode       = 0x10; 
                 
  static uint32_t prevUserEEPROMReadTime = 0,
                  prevUserDisplayRenewTime = 0;
  
  
  // current time
  nowTime = millis(); 
  // Re-read variable's value on first call and every constEEPROMReadInterval
  if ((0 == prevUserEEPROMReadTime) || (constUserEEPROMReadInterval <= (uint32_t) (nowTime - prevUserEEPROMReadTime))) {
     // Configure TWI to work on special pins
     SoftTWI.reconfigure(constUserEEPROMSDAPin, constUserEEPROMSCLPin);
     // Read 4 bytes started from cell 0 to buffer, that used as uint8_t array.
     // On success - update variable's value
     if (AT24CXXRead(&SoftTWI, constUserEEPROMI2CAddress, 0x00, 4, (uint8_t*) _buffer)) {
        if (isSafePin(_buffer[0]) && isSafePin(_buffer[1])) {
           bh1750SDAPin     = _buffer[0];
           bh1750SCLPin     = _buffer[1];
        }
        bh1750I2CAddress = _buffer[2];
        bh1750Mode       = _buffer[3];

        Serial.println("BH1750 settings");
        Serial.println(bh1750SDAPin, HEX);
        Serial.println(bh1750SCLPin, HEX);
        Serial.println(bh1750I2CAddress, HEX);
        Serial.println(bh1750Mode, HEX);
/*
Sketch uses 30,618 bytes (94%) of program storage space. Maximum is 32,256 bytes.
Global variables use 1,231 bytes (60%) of dynamic memory, leaving 817 bytes for local variables. Maximum is 2,048 bytes.

ketch uses 30,722 bytes (95%) of program storage space. Maximum is 32,256 bytes.
Global variables use 1,231 bytes (60%) of dynamic memory, leaving 817 bytes for local variables. Maximum is 2,048 bytes.

*/
        //uservalue = WANTS_VALUE_WHOLE;
        //uservalue = WANTS_VALUE_SCALED;
        SoftTWI.reconfigure(bh1750SDAPin, bh1750SCLPin);
        if (RESULT_IN_BUFFER == getBH1750Metric(&SoftTWI, bh1750I2CAddress, bh1750Mode, SENS_READ_LUX, &uservalue)) {
           Serial.println("BH1750 metric");
           Serial.print("\tscaled value: "); 
           Serial.println(uservalue); 
           ldiv_t tmpResult;
           tmpResult = ldiv(uservalue, 100);
           Serial.print("\tWhole part: "); 
           Serial.println(tmpResult.quot); 
           Serial.print("\tFrac part: "); 
           Serial.println(tmpResult.rem); 
        };
     }
     prevUserEEPROMReadTime = nowTime;
  }

  //Show virtual screens

#ifdef FEATURE_USER_DISPLAY_ENABLE
  uint8_t dataLength;
  uint32_t timestamp;
  tm dateTime;

  // do nothing if renew wait time is not expiried
  if (constUserDisplayRenewInterval > (uint32_t) (nowTime - prevUserDisplayRenewTime)) { return; }

  prevUserDisplayRenewTime = nowTime;
  
  // buffer can be nulled to help detects EOL by '\0' at end of string
  // how fast memset?
  memset(_buffer, 0x00, constBufferSize + 1);
  reportVirtualScreenCnt++;
  if (constVirtualScreensNum <= reportVirtualScreenCnt) { reportVirtualScreenCnt = 0; }

  // Write to _buffer commands for output direction (0x06) and clear screen (0x01)
  // Note: strcpy() place "\x6\x1" to _buffer as bytes with values 6 and 1 (_buffer[0]=6, _buffer[1]=1), not as C-string '\x6\x1'
  _buffer[0] = 0x06;
  _buffer[1] = 0x01;
  dataLength = 2;

  switch (reportVirtualScreenCnt){       
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
      timestamp = (millis()/1000);
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
      timestamp = ((millis()-sysMetrics.sysCmdLastExecTime)/1000);
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
  
