/*        
          LCD1602
      +- C0 .... CF-+
      v             v
  R1 |XXX.XXX.XXX.XXX |  << IP Address
  R2 |4294967295 ms   |  << uptime
 
     |Zabbuino 1.2.3  |  << 
     |     ALL OK     |  << 
 
  R1, R2 - row #0x1, row #0x2 
  C0..RF - col #0x0 .. col #0xF 

         0                                                         N
   _src |--Zabbuino 1.2.3--ALL OK                                   |
         ^^              ^^
    0x01 -++- 0x06  '\n' -++-'\t' 
   
   N - constBufferSize (tune.h)

  !!! Resulted string must be shorter that constBufferSize to avoid mailfunction !!!

 / Note: "\x6\x1" placed to _src as bytes with values 6 and 1 (_src[0]=6, _src[1]=1), not as C-string '\x6\x1'

*/
 

/*****************************************************************************************************************************
  Called on init stage

*****************************************************************************************************************************/
void initStageReportScreen(char* _src) {
 uint8_t dataLength;
 // Write to _src commands for output direction (0x06) and clear screen (0x01), then append Agent version
 _src[0] = 0x06;
 _src[1] = 0x01;
 //dataLength +=2;
 //strcpy_P(&_src[dataLength], constZbxAgentVersion);
 strcpy_P(&_src[2], constZbxAgentVersion);
 // Print ready string
 showReportScreen(_src, 0xFF);
 //dataLength = strlen(_src);
 //_src[dataLength] = '\0';      
 }

/*****************************************************************************************************************************
  Called on every loop with constSystemDisplayRenewInterval (ms)
  _virtualScreenNum take value [0 ... constVirtualScreensNum]
  
  (constSystemDisplayRenewInterval & constVirtualScreensNum defined in src/tune.h)
*****************************************************************************************************************************/
void showReportScreen(char* _src, uint8_t _virtualScreenNum) {
  uint8_t dataLength;
  uint32_t timestamp;
  tm dateTime;
  
  //DTSH( SerialPrint_P(PSTR("Show screen #")); Serial.println(_virtualScreenNum); )
  // _virtualScreenNum eq 0xFF mean that _src already contan ready to print data
  if (0xFF != _virtualScreenNum) {
     // Write to _src commands for output direction (0x06) and clear screen (0x01)
     _src[0] = 0x06;
     _src[1] = 0x01;
     dataLength = 2;
/*

Sketch uses 29,956 bytes (92%) of program storage space. Maximum is 32,256 bytes.
Global variables use 1,327 bytes (64%) of dynamic memory, leaving 721 bytes for local variables. Maximum is 2,048 bytes.

*/
     // change constVirtualScreensNum in tune.h if you need more or less virtual screens
     switch (_virtualScreenNum) {
       case 0x00:
         // **** This screen eat ~4,2kB of progspace ****
         // copy "Uptime: " string to src, following '\n'
         strcpy_P(&_src[dataLength], PSTR("Up  : "));
         dataLength += 6;
         timestamp = (millis()/1000);
         // break down timestamp to tm struct
         gmtime_r(&timestamp, &dateTime);
         // elapsed days less that current year day by 1d
         dateTime.tm_yday--;
         // strftime take so much progspace
         strftime(&_src[dataLength], 30, "%jD %T", &dateTime);
         dataLength = strlen(_src);
         // replace to '\0' by '\n' at the end position of string to have newline on physical screen
         _src[dataLength++] = '\n';
         // Copy "Idle: " to show it on second line
         strcpy_P(&_src[dataLength], PSTR("Idle: "));
         dataLength += 6;
         timestamp = ((millis()-sysMetrics1.sysCmdLastExecTime)/1000);
         // again break down timestamp to tm struct and correct year day
         gmtime_r(&timestamp, &dateTime);
         dateTime.tm_yday--;
         strftime(&_src[dataLength], 30, "%jD %T", &dateTime);
         dataLength = strlen(_src);
         break;

       case 0x01:
         // **** This screen eat ~4.6kB of progspace ****
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
         // "print" current date & time on second line
         time_t y2kts; 
         if (getY2KTime(constSystemRtcSDAPin, constSystemRtcSCLPin, constSystemRtcI2CAddress, &y2kts)) {
            localtime_r(&y2kts, &dateTime);
            strcpy_P(&_src[dataLength], PSTR("\t\tTIME\n"));
            dataLength += 7;
            // strftime take so much progspace
            strftime(&_src[dataLength], 30, "%d/%m/%Y %T", &dateTime);
            dataLength = strlen(_src);
         }
         break;

       case 0x02:
         // build your own screen and increase constVirtualScreensNum's value in src/tune.h  
         break;
         
       default:
         // write empty string
         break;
     } // switch (_virtualScreenNum)
     _src[dataLength] = '\0';      
  } // if (0xFF != _virtualScreenNum)  
  //Serial.println(_src);
  // push data to LCD via I2C (refer to "SYSTEM HARDWARE SECTION" in src/tune.h)
  printToPCF8574LCD(constSystemDisplaySDAPin, constSystemDisplaySCLPin, constSystemDisplayI2CAddress, constSystemDisplayBackLight, constSystemDisplayType, _src);
}

