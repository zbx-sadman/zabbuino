#include <time.h>

/*****************************************************************************************************************************
Write report to system display (LCD1602, etc)

*****************************************************************************************************************************/
void reportToScreen(char* _src, uint8_t _virtualScreenNum) {
  uint8_t dataLength;
  uint32_t timestamp;
  tm dateTime;
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
  //  _src |--Zabbuino 1.2.3--ALL OK                                   |
  //        ^^              ^^
  //  0x01 -++- 0x06  '\n' -++-'\t' 
  // 
  //  N - constBufferSize (tune.h)
  //
  //  !!! Resulted string must be shorter that constBufferSize to avoid mailfunction !!!
  //
  
  //DTSH( SerialPrint_P(PSTR("Show screen #")); Serial.println(_virtualScreenNum); )
  // _virtualScreenNum eq 0xFF mean that _src already contan ready to print data
  if (0xFF != _virtualScreenNum) {
     //memset((void*) _src, 0x20, 50);

     // Write to _src commands for output direction (0x06) and clear screen (0x01)
     //strcpy_P(_src, PSTR("\x6\x1"));
     // Note: "\x6\x1" placed to _src as bytes with values 6 and 1 (_src[0]=6, _src[1]=1), not as C-string '\x6\x1'
     //dataLength = strlen(_src);
     _src[0] = 0x06;
     _src[1] = 0x01;
     dataLength = 2;

     // change constVirtualScreensNum in tune.h if you need more or less virtual screens
     switch (_virtualScreenNum){
       case 0x01:
         // copy constZbxAgentVersion variable (see "AGENT CONFIGURATION SECTION" in basic.h) content to src, starting from [dataLength] cell
         strcpy_P(&_src[dataLength], constZbxAgentVersion);
         // _src is C-string ('\0' terminated) and we can get it length with strlen() for correcting dataLength.
         dataLength = strlen(_src);
         // replace to '\0' by '\n' at the end position of string to have newline on physical screen
         _src[dataLength++] = '\n';
         // copy "Uptime: " string to src, following '\n'
         strcpy_P(&_src[dataLength], PSTR("Up: "));
         dataLength += 4;
         // write millis() value to the _src starting from [dataLength] cell (take address of (_src[0] + dataLength) and give ltoa() as buffer);
         //ultoa(millis(), &_src[dataLength], 10);
         //dataLength = strlen(_src);
         //strcpy_P(&_src[dataLength], PSTR(" ms"));
         // timestamp must be in seconds, not ms
         timestamp = (millis()/1000);
         gmtime_r(&timestamp, &dateTime);
         dateTime.tm_yday--;
         // need to add tm::tm_yday directly to _src to avoid 'Up: 001D ...' at the start
         strftime(&_src[dataLength], 30, "%jD %T", &dateTime);
         //timestampToDataTimeStr(timestamp, &_src[dataLength]);

         dataLength = strlen(_src);
         break;

       case 0x02:
         strcpy_P(&_src[dataLength], PSTR("VCC:"));
         // Move write position to 4 char ('VCC:')
         dataLength += 4;
         // write unit32_t value to the _src starting from [dataLength] cell;
         // getADCVoltage - internal function, that return actual MCU voltage
         ultoa(getADCVoltage(ANALOG_CHAN_VBG), &_src[dataLength], 10);
         // voltage in mv, usually take 4 char
         dataLength += 4; 
         strcpy_P(&_src[dataLength], PSTR("mV Mem:"));
         dataLength += 7; 
         // write unit32_t value to the _src starting from [dataLength] cell;
         // sysMetrics.sysRamFree - internal metric (refer to structs.h)
         ultoa(sysMetrics1.sysRamFree, &_src[dataLength], 10);
         // Now len of string unknown - sysRamFree may take 3 char or 2...
         dataLength = strlen(_src);
         _src[dataLength++] = 'b';
         // Go to new line
         _src[dataLength++] = '\n';
         strcpy_P(&_src[dataLength], PSTR("Idle: "));
         dataLength += 6;
         //ultoa((millis()-sysMetrics1.sysCmdLastExecTime), &_src[dataLength], 10);
         // len of string unknown again 
         //dataLength = strlen(_src);
         //strcpy_P(&_src[dataLength], PSTR(" ms"));        
         //dataLength += 3;
         timestamp = ((millis()-sysMetrics1.sysCmdLastExecTime)/1000);
         gmtime_r(&timestamp, &dateTime);
         dateTime.tm_yday--;
         strftime(&_src[dataLength], 30, "%jD %T", &dateTime);
         //timestampToDataTimeStr(timestamp, &_src[dataLength]);
         dataLength = strlen(_src);

         break;

       case 0x03:
         if (RESULT_IS_OK == getDateTime(constSystemRtcSDAPin, constSystemRtcSCLPin, constSystemRtcI2CAddress, &dateTime)) {
            uint32_t y2kts; 
            int16_t tzOffset; 
            int8_t rc;
            rc = getTZ(constSystemRtcSDAPin, constSystemRtcSCLPin, constSystemRtcEEPROMI2CAddress, &tzOffset);
            if (RESULT_IN_LONGVAR != rc) {
              tzOffset = 0;
            }
            // 76 bytes for TZ correction
            y2kts = mk_gmtime(&dateTime);;
            y2kts += tzOffset;
            gmtime_r(&y2kts, &dateTime);
            
            //dateTime.tm_mon++;   // tm_mon  - months since January [0 to 11], but human wants [1 to 12]
            //dateTime.tm_wday--;  // tm_wday - days since Sunday [0 to 6], but human wants [1 to 7]
            //dateTime.tm_year += 1900; // tm_year - years since 1900
/*
  Serial.print("Sec: "); Serial.println(dateTime.tm_sec);
  Serial.print("Min: "); Serial.println(dateTime.tm_min);
  Serial.print("Hour: "); Serial.println(dateTime.tm_hour);
  Serial.print("Wday: "); Serial.println(dateTime.tm_wday);
  Serial.print("Mday: "); Serial.println(dateTime.tm_mday);
  Serial.print("Month: "); Serial.println(dateTime.tm_mon);
  Serial.print("Year: "); Serial.println(dateTime.tm_year);
  Serial.println();
  */     
          strcpy_P(&_src[dataLength], PSTR("\t\tTIME\n"));
          dataLength += 7;
          strftime(&_src[dataLength], 30, "%d/%m/%Y %T", &dateTime);
          dataLength = strlen(_src);
         }
         
       case 0x04:
         break;
         // build your own screen and increase constVirtualScreensNum's value in tune.h  
         
       default:
         // write empty string
         break;
   }
         _src[dataLength] = '\0';      
   
   }   
   //Serial.println(_src);
   // push data to LCD via I2C (refer to "ALARM & REPORT SECTION" in tune.h)
   printToPCF8574LCD(constSystemDisplaySDAPin, constSystemDisplaySCLPin, constSystemDisplayI2CAddress, constSystemDisplayBackLight, constSystemDisplayType, _src);
}

