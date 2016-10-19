#include "busUART.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                      COMMON UART SECTION
*/

uint8_t serialRXFlush(SoftwareSerial* _swSerial, const uint8_t _slowMode = false) {
  while (true) {
    // Seems that APC UPS's slow a bit and need to wait some time before check recieve buffer. 
    // Otherwise - after (0 == swSerial->available()) a few bytes can be found on input
    if (_slowMode) { delay(10); }
    if (0 == _swSerial->available()) { break; }
    _swSerial->read();
  }
  return true;
}

uint8_t serialRecive(SoftwareSerial* _swSerial, uint8_t* _src, const uint8_t _size, const uint32_t _readTimeout, const uint8_t _stopOn, const uint8_t _slowMode = false) {
  unsigned long startTime = millis();
  uint8_t len = 0;
  while ((len <  _size) && (millis() - startTime < _readTimeout)) {
    if (_swSerial) {
       // Slow talk with APC UPC'es
       if (_slowMode) { delay(10); }
       if (_swSerial->available() > 0) {
          uint8_t c = (uint8_t) _swSerial->read();
          if (!c && !len) {
             continue; // skip 0 at startup
          }
          _src[len] = c;
          // Stop and jump out from subroutine if some byte is reached
          if (_stopOn == _src[len]) { return len+1; }
          len++;
       }
    }
  }
  return len;
}

uint8_t serialSend(SoftwareSerial* _swSerial, const uint8_t* _src, const uint8_t _size, const uint8_t _slowMode = false) {
  uint8_t i; 
  if (_swSerial) {
     // Send data
     for (i = 0; i <  _size; i++) {
       // do not rush when work with APC UPS's
       if (_slowMode) { delay(10); }
//       Serial.print("Byte# "); Serial.print(i); Serial.print(" => "); Serial.print(_src[i], HEX);  Serial.print(" '"); Serial.print((char) _src[i]); Serial.println("' ");
       if (! _swSerial->write(_src[i])) { return false; }
    }
  }
  return true;
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           Megatec protocol compatible UPS SECTION

   http://networkupstools.org/protocols/megatec.html
*/

int8_t getMegatecUPSMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _command, uint8_t _fieldNumber, uint8_t* _dst) {
  uint8_t command, len, srcPos, dstPos, fileldNumber;
  SoftwareSerial swSerial(_rxPin, _txPin);

  if (hstoba(_command, (char*) _command, 1)) { _command[1] = '\0'; } ;
  command = _command[0];
  // Serial.print("command: "); Serial.println(command, HEX);
  len = 1; // default length

  switch (command) {
     case 'F':   // UPS Rating Information
     case 'I':   // UPS Information Command
       break;
     case 'Q':   // Q1 in real. Status Inquiry
       len = 2;
       break;
     default:
//       return RESULT_IS_FAIL;
       return 0;
  }
  
  Serial.println("Command allowed");


/*
     case 0x01:  // ^A,  Model string
          command = 'I';
          fieldNumber = 0x01;
          break;
     case 'B':   // Battery voltage, V
          command = "Q1";
          fieldNumber = 0x05;
          break;
     case 'C':   // Internal temperature, C
          command = "Q1";
          fieldNumber = 0x06;
          break;
     case 'F':   // Line frequency, Hz
          command = "Q1";
          fieldNumber = 0x04;
          break;
     case 'L':   // Input line voltage, V
          command = "Q1";
          fieldNumber = 0x00;
          break;     
     case 'O':   // Output voltage, V
          command = "Q1";
          fieldNumber = 0x02;
          break;
     case 'P':   // Power load, %
          command = "Q1";
          fieldNumber = 0x03;
          break;
     case 'l':   // Low transfer voltage, V
          command = "Q1";
          fieldNumber = 0x01;
          break;
*/

  swSerial.begin(MEGATEC_UPS_UART_SPEED);
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
  // Stop the Timer1 to prevent UART errors
  stopTimerOne(); 
#endif

  //  Step #1. Send user's command & recieve answer
  //
  // Flush all device's transmitted data to avoid get excess data in recieve buffer
  serialRXFlush(&swSerial, false);

  if (! serialSend(&swSerial, _command, len , false)) {
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_TIMEOUT; 
  };
  //Serial.println("recieve");
  // Recieve answer from UPS. Answer placed to buffer directly for additional processing 
  len = serialRecive(&swSerial, _dst, MEGATEC_MAX_ANSWER_LENGTH, MEGATEC_DEFAULT_READ_TIMEOUT, '\r');
  //Serial.print("len: "); Serial.println(len);
  //Serial.print("_dst: "); Serial.println((char) _dst);
  // return timeout sign if packet not finished by <CR>
  if ('\r' != _dst[len-1]) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_TIMEOUT; 
  };
  if ('(' != _dst[0]) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_WRONG_ANSWER; 
  };
  _dst[len-1] = '\0';
  if (0 == fileldNumber) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return RESULT_IN_BUFFER; 
  }
  
  //  Step #2. Search the field that specified by number & move data to the output buffer begin
  //           If the field number is greater than the available, then use the data from the last field.
  //
  srcPos = 1; // start from 1-th byte to skip '(' prefix
  fileldNumber = dstPos = 0;
  // Walk over the recieved string while no EOL reached
  while ('\0' != _dst[srcPos]) {
    // Just copy chars from the current "field" to begin of the output buffer
    _dst[dstPos] = _dst[srcPos];
    // The separator was found
    if (0x20 == _dst[srcPos]) {
       fileldNumber++;
       // It is specified field number?
       if (_fieldNumber == fileldNumber) { 
          // jump out from copy-on-search cycle if it's true
          break;
       } else {
          // Otherwise - start next field writing from begin of the output buffer
          dstPos = 0;  
       }
    } else {
      dstPos++;
    } // if (0x20 == _dst[srcPos]) .. else ..
    srcPos++; 
  };
  // make C-string
  _dst[dstPos] = '\0';
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
  // Start the Timer1
  startTimerOne();
#endif
  //Serial.println("Destroy current SoftwareSerial instance");
  swSerial.~SoftwareSerial();

  return RESULT_IN_BUFFER;
}



/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           APC SMART UPS SECTION

   Despite the lack of official information from APC, this table has been constructed. It’s standard RS-232 serial communications at 2400 bps/8N1. 
   Don’t rush the UPS while transmitting or it may stop talking to you. This isn’t a problem with the normal single character queries, but it really 
   does matter for multi-char things like "@000". Sprinkle a few calls to usleep() in your code and everything will work a lot better.
   http://networkupstools.org/protocols/apcsmart.html
*/



/****************************  need to optimise startTimerOne(); calls   *****************************************/
int8_t getAPCSmartUPSMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _command, uint8_t _commandLen,  uint8_t* _dst) {
  uint8_t command, 
          len, 
          sendTimes,
          sendCommandTwice = false;
          
  SoftwareSerial swSerial(_rxPin, _txPin);

  // _src used as input uint8_t array and output char array due it can save RAM. 
  // Data does not corrupt, because hstoba() write take two char (2 byte) and write one uin8_t (1 byte).  
  if (hstoba(_command, (char*) _command, 1)) { _command[1] = '\0'; } ;
  
  
  // May be just make sum of buffer's bytes to use into switch: '^'+'A' => ...
  command = _command[0];
  // Serial.print("command: "); Serial.println(command, HEX);
  switch (command) {
     // Shutdown commands not working yet
     case 0x0E:  // ^N,  Turn on UPS
     case 'K':   // 0x4B Shutdown with grace period
     case 'Z':   // 0x5A Shutdown immediately
          sendCommandTwice = true;
          
     case 0x01:  // ^A,  Model string
     case 'B':   // 0x42 Battery voltage, V
     case 'C':   // 0x43 Internal temperature, C
     case 'F':   // 0x46 Line frequency, Hz
     case 'G':   // 0x47 Cause of transfer   
     case 'L':   // 0x4C Input line voltage, V
     case 'M':   // 0x4D Maximum line voltage, V
     case 'N':   // 0x4E Minimum line voltage, V
     case 'O':   // 0x4F Output voltage, V
     case 'P':   // 0x50 Power load, %
     case 'Q':   // 0x51 Status flags
     case 'V':   // 0x56 Firmware revision
     case 'X':   // 0x58 Self-test results
//     case 'a':  // Protocol info (long string)
     case 'b':   // 0x62 Firmware revision
     case 'c':   // 0x63 UPS local id
     case 'e':   // 0x65 Return threshold, %
     case 'f':   // 0x66 Battery level, %
     case 'g':   // 0x67 Nominal battery voltage, V
     case 'h':   // 0x68 Measure-UPS: Ambient humidity. %
     case 'i':   // 0x69 Measure-UPS: Dry contacts
     case 'j':   // 0x6A Estimated runtime, min
     case 'l':   // 0x6C Low transfer voltage, V
     case 'm':   // 0x6D Manufacturing date
     case 'n':   // 0x6E Serial number
     case 't':   // 0x74 Measure-UPS: Ambient temperature, C
     case 'u':   // 0x75 Upper transfer voltage, V
     case 'v':   // 0x76 Measure-UPS: Firmware
     case 'x':   // 0x78 Last battery change 
     case 'y':   // 0x79 Copyright notice
     case '7':   // 0x37 Dip switch positions
     case '8':   // 0x38 Register #3
     case '9':   // 0x39 Line quality
    //   Serial.println("Command allowed");
       break;
     default:
       return RESULT_IS_FAIL;
  }
  
  swSerial.begin(APC_UPS_UART_SPEED);
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
  // Stop the Timer1 to prevent UART errors
  stopTimerOne(); 
#endif
  //  Step #1. Send Y-command. Its must return 'SM<0x0D>' (3 byte) and switch UPS to Smart mode
  //
  // Flush all device's transmitted data to avoid get excess data in recieve buffer
  // APC UPS can be flushed in slow mode
  serialRXFlush(&swSerial, true);
  command = 'Y';
  if (! serialSend(&swSerial, &command, 1, true)) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_TIMEOUT;      
  };
  len = serialRecive(&swSerial, _dst, 0x03, APC_DEFAULT_READ_TIMEOUT, '\0');
  // Connection timeout occurs (recieved less than 3 byte)
  if (len < 0x03) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_TIMEOUT; 
  }
  // Check for "SM\r"
  if ( 'S' != _dst[0] || 'M' != _dst[1] || '\r' != _dst[2]) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_WRONG_ANSWER; 
  };    
  
  
  //  Step #2. Send user's command & recieve answer
  //
  // If not all data is recieved from talking device on step #1 - its RX buffer must be cleared 
  serialRXFlush(&swSerial, true);
  sendTimes = sendCommandTwice ? 2 : 1;

  while (sendTimes) {
     // All commands fits to 1 byte
     //Serial.println("send");
     if (! serialSend(&swSerial, _command, 1, true)) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
        startTimerOne();
#endif
        return DEVICE_ERROR_TIMEOUT; 
     };
     //Serial.println("recieve");
      // Recieve answer from Smart UPS. Answer placed to buffer directly and does not require additional processing 
     len = serialRecive(&swSerial, _dst, APC_MAX_ANSWER_LENGTH, APC_DEFAULT_READ_TIMEOUT, '\r');
     //Serial.print("len: "); Serial.println(len);
     if (!sendCommandTwice && '\r' != _dst[len-1]) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
        startTimerOne();
#endif
        return DEVICE_ERROR_TIMEOUT; 
     };
     //Serial.print("reply: "); Serial.println((char*) _dst);

     sendTimes--;
     if (0 < sendTimes) { 
        // Zabbuino always return 1, because UPS always return nothing
        _dst[0] = '1';  _dst[1] = '\0';
        // ^N, K-, Z- commands must send twice with 1.5s...3s delay between chars. 
        delay(1700); 
     }
  }

  // Make C-string by replacing '\r' to '\0'
   _dst[len-1] = '\0';
  // 'j'-сommand return number of mins with trailing ':'. We need destroy this obstacle to let Zabbix convert its to numeric value properly.
  if ('j' == _command[0]) {_dst[len-2] = '\0';}   
     
  //  Step #3. Send R-command. It's must return 'BYE' and switch UPS to Dumb mode
  //
  // 
  command = 'R';
  if (! serialSend(&swSerial, &command, 1, true)) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_TIMEOUT; 
  };
  serialRXFlush(&swSerial, true);
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
  // Start the Timer1
  startTimerOne();
#endif
  //Serial.println("Destroy current SoftwareSerial instance");
  swSerial.~SoftwareSerial();
 
  return RESULT_IN_BUFFER;
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           PZEM-004 SECTION
*/

/*
Based on: https://github.com/olehs/PZEM004T
version 1.0 is used

*/


uint8_t crcPZEM004(uint8_t* _data, uint8_t _size) {
    uint16_t crc = 0;
    for(uint8_t i=0; i < _size; i++) { crc += (uint8_t) *_data; _data++;}
    //while (_size) { crc += *_data; _data++; _size--; }
    return (uint8_t)(crc & 0xFF);
}

int8_t getPZEM004Metric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const char* _ip, uint8_t* _dst) {
  uint8_t command, len;
  int32_t result;
  SoftwareSerial swSerial(_rxPin, _txPin);

  swSerial.begin(PZEM_UART_SPEED);

   switch (_metric) {
     case SENS_READ_AC:
       command = PZEM_CURRENT; 
       //Serial.println("Taking Voltage");
       break;
     case SENS_READ_VOLTAGE:
       command = PZEM_VOLTAGE; 
       //Serial.println("Taking Voltage");
       break;
     case SENS_READ_POWER:
       command = PZEM_POWER; 
       //Serial.println("Taking Power");
       break;
     case SENS_READ_ENERGY:
       command = PZEM_ENERGY; 
       //Serial.println("Taking Energy");
       break;
   }

   /*  Send to PZEM004 */
//    Serial.println("Send commmand...");
    
    // Make packet for PZEM
    // 1-th byte in the packet - metric (command)
    _dst[0] = command; 
    // 2..5 bytes - ip address. Convert its from _ip or use default (192.168.1.1) if _ip is invalid
    result = hstoba(&_dst[1], _ip, 4);
    if (!result) {
       _dst[1] = 0xC0;  // 192
       _dst[2] = 0xA8;  // 168
       _dst[3] = 0x01;  // 1
       _dst[4] = 0x01;  // 1
    } 

    // 6-th byte - used to provide the value of the alarm threshold (in kW), 00 else
    _dst[5] = 0x00; 
    // 7-th byte - CRC
    _dst[6] = crcPZEM004(_dst, PZEM_PACKET_SIZE - 1); 

#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
    // Stop the Timer1 to prevent UART errors
    stopTimerOne(); 
#endif
    //for(int i=0; i < sizeof(buffer); i++) { Serial.print("Byte# "); Serial.print(i); Serial.print(" => "); Serial.println(buffer[i], HEX);  }
    if (! serialSend(&swSerial, _dst, PZEM_PACKET_SIZE, false)) { return DEVICE_ERROR_TIMEOUT; };

    /*  Recieve from PZEM004 */
    //Serial.println("Recieve answer...");
    len = serialRecive(&swSerial, _dst, PZEM_PACKET_SIZE, PZEM_DEFAULT_READ_TIMEOUT, '\0');
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
    // Start the Timer1
    startTimerOne();
#endif    
    // Serial.println("Destroy current SoftwareSerial instance");
    swSerial.~SoftwareSerial();// 

    // Connection timeout occurs
    // if (len != PZEM_PACKET_SIZE) { return DEVICE_ERROR_TIMEOUT; }
    if (len < PZEM_PACKET_SIZE) { return DEVICE_ERROR_TIMEOUT; }
    // Wrong answer. buffer[0] must contain command - 0x10 (command B1 -> reply A1)
    // command = command - 0x10;
    //Serial.print("Header expected: "); Serial.println(command, HEX);
    //Serial.print("Header real: "); Serial.println(_dst[0], HEX);
    if (_dst[0] != (command - 0x10)) { return DEVICE_ERROR_WRONG_ANSWER; }
    // Bad CRC
    if (_dst[6] != crcPZEM004( _dst, len - 1)) { return DEVICE_ERROR_CHECKSUM; }
    
//    Serial.println("Calculating...");
   // data is placed in buffer from 2-th byte, because 1-th byte is Header
   switch (_metric) {
     case SENS_READ_AC:
       result = ((_dst[1] << 8) + _dst[2]) * 100 + _dst[3];
       // _dst (cBuffer at real) cast to char due numeric-to-ascii subs require char array
       ltoaf(result, (char*) _dst, 2);
       break;
     case SENS_READ_VOLTAGE:
       result = ((_dst[1] << 8) + _dst[2]) * 10 + _dst[3]; 
       ltoaf(result, (char*) _dst, 1);
       break;
     case SENS_READ_POWER:
       result = (_dst[1] << 8) + _dst[2];
       ltoa(result, (char*) _dst, 10);
       break;
     case SENS_READ_ENERGY:
       result = ((uint32_t) _dst[1] << 16) + ((uint16_t) _dst[2] << 8) + _dst[3];
       ltoa(result, (char*) _dst, 10);
       break;
   }

  return RESULT_IN_BUFFER;
}

