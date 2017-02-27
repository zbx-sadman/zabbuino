/*****************************************************************************************************************************
Write report to physical screen (LCD1602, etc)

*****************************************************************************************************************************/
void reportToScreen(char* _src, uint8_t _virtualScreenNum) {
  uint8_t dataLength;
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
  
  DTSH( SerialPrint_P(PSTR("Show screen #")); Serial.println(_virtualScreenNum); )
  // _virtualScreenNum eq 0xFF mean that _src already contan ready to print data
  if (0xFF != _virtualScreenNum) {
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
         dataLength = strlen(_src);
         // write millis() value to the _src starting from 'dataLength' cell (take address of (_src[0] + dataLength) and give ltoa() as buffer);
         ltoa(millis(), &_src[dataLength], 10);
         dataLength = strlen(_src);
         strcpy_P(&_src[dataLength], PSTR(" ms"));
         break;

       case 0x02:
         // build your own screen and increase constVirtualScreensNum's value in tune.h  
         break;
    
     default:
         // write empty string
         _src[0] = '\0';      
         break;
   }
   }   
   // push data to LCD via I2C (refer to "ALARM & REPORT SECTION" in tune.h)
   printToPCF8574LCD(constReportScreenSDAPin, constReportScreenSCLPin, constReportScreenI2CAddress, constReportScreenBackLight, constReportScreenType, _src);
}

