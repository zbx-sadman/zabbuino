void initStageUserFunction(char* _src) {
  // Show report screen # 0x0F (just one of existing screens) on start
  // Note that not all system struct is initialized at this stage and you can't get localIP() or localtime() info
  showReportScreen(_src, 0x0F);
}

void loopStageUserFunction() {

}

/*****************************************************************************************************************************
Write report to system display (LCD1602, etc)

*****************************************************************************************************************************/
void showReportScreen(char* _src, int8_t _virtualScreenNum) {
#ifdef FEATURE_SYSTEM_DISPLAY_ENABLE
  const uint8_t  constVirtualScreensNum                           = 4;      // Number of report virtual screens
  static uint8_t reportVirtualScreenNum = 0; 
  uint8_t dataLength, currentVirtualScreenNum;
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
  if (REPORT_SCREEN_SHOW_NEXT == _virtualScreenNum) { 
     reportVirtualScreenNum++;
     if (constVirtualScreensNum <= reportVirtualScreenNum) {
        reportVirtualScreenNum = 0;
     }
     currentVirtualScreenNum = reportVirtualScreenNum;
  } else {
     currentVirtualScreenNum = _virtualScreenNum;
  } 
  
     // Write to _src commands for output direction (0x06) and clear screen (0x01)
     // Note: strcpy() place "\x6\x1" to _src as bytes with values 6 and 1 (_src[0]=6, _src[1]=1), not as C-string '\x6\x1'
     _src[0] = 0x06;
     _src[1] = 0x01;
     dataLength = 2;

     // change constVirtualScreensNum in tune.h if you need more or less virtual screens
     switch (currentVirtualScreenNum){
       case 0x0F:
         strcpy_P(&_src[dataLength], constZbxAgentVersion);
         dataLength = strlen(_src);
         break;   
         
       case 0x00:
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

       case 0x01:
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
         ultoa(sysMetrics.sysRamFree, &_src[dataLength], 10);
         // Now len of string unknown - sysRamFree may take 3 char or 2...
         dataLength = strlen(_src);
         _src[dataLength++] = 'b';
         // Go to new line
         _src[dataLength++] = '\n';
         strcpy_P(&_src[dataLength], PSTR("Idle: "));
         dataLength += 6;
         //ultoa((millis()-sysMetrics.sysCmdLastExecTime), &_src[dataLength], 10);
         // len of string unknown again 
         //dataLength = strlen(_src);
         //strcpy_P(&_src[dataLength], PSTR(" ms"));        
         //dataLength += 3;
         timestamp = ((millis()-sysMetrics.sysCmdLastExecTime)/1000);
         gmtime_r(&timestamp, &dateTime);
         dateTime.tm_yday--;
         strftime(&_src[dataLength], 30, "%jD %T", &dateTime);
         dataLength = strlen(_src);

         break;

       case 0x02:
         time_t y2kts; 
         // DS3231 RTC operate by Y2K-based timestamp
         if (RESULT_IS_OK == getY2KTime(&SoftTWI, &y2kts)) {
            localtime_r(&y2kts, &dateTime);
            
            //dateTime.tm_mon++;   // tm_mon  - months since January [0 to 11], but human wants [1 to 12]
            //dateTime.tm_wday--;  // tm_wday - days since Sunday [0 to 6], but human wants [1 to 7]
            //dateTime.tm_year += 1900; // tm_year - years since 1900
            strcpy_P(&_src[dataLength], PSTR("\t\tTIME\n"));
            dataLength += 7;
            strftime(&_src[dataLength], 30, "%d/%m/%Y %T", &dateTime);
            dataLength = strlen(_src);
         }
         break;
         
       case 0x03:
         strcpy_P(&_src[dataLength], PSTR("Light (lux): "));
         // 'Light (lux): ' - 13 chars, 
         dataLength += 13;
         // I2C sensor is connected on D2 (SDA) & D3 (SCL). Reconfigure global Software I2C Interface and put light value to output buffer
         SoftTWI.reconfigure(2, 3);
         getBH1750Metric(&SoftTWI, 0x23, 0x10, SENS_READ_LUX, &_src[dataLength]);
         dataLength = strlen(_src);

       case 0x04:
         break;
         // build your own screen and modify constVirtualScreensNum's value on top of this subroutine
         
       default:
         // write empty string
         break;
   }
   _src[dataLength] = '\0';      
   
   //Serial.println(_src);
   // push data to LCD via I2C (refer to "SYSTEM HARDWARE SECTION" in src/tune.h)   
   SoftTWI.reconfigure(constSystemDisplaySDAPin, constSystemDisplaySCLPin);
   printToPCF8574LCD(&SoftTWI, constSystemDisplayI2CAddress, constSystemDisplayBackLight, constSystemDisplayType, _src);
#endif //FEATURE_SYSTEM_DISPLAY_ENABLE
}

