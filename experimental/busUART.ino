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

//uint8_t serialRecive(SoftwareSerial* _swSerial, uint8_t* _buffer, const uint8_t _size, const uint32_t _readTimeout, const uint8_t _stopOnCR) {
uint8_t serialRecive(SoftwareSerial* _swSerial, uint8_t* _buffer, const uint8_t _size, const uint32_t _readTimeout, const uint8_t _stopOn, const uint8_t _slowMode = false) {
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
          _buffer[len] = c;
          // Stop and jump out from subroutine if some byte is reached
          if (_stopOn == _buffer[len]) { return len+1; }
          len++;
       }
    }
  }
  return len;
}

uint8_t serialSend(SoftwareSerial* _swSerial, const uint8_t* _buffer, const uint8_t _size, const uint8_t _slowMode = false) {
  uint8_t i; 
  if (_swSerial) {
     // Send data
     for (i = 0; i <  _size; i++) {
       // do not rush when work with APC UPS's
       if (_slowMode) { delay(10); }
//       Serial.print("Byte# "); Serial.print(i); Serial.print(" => "); Serial.print(_buffer[i], HEX);  Serial.print(" '"); Serial.print((char) _buffer[i]); Serial.println("' ");
       if (! _swSerial->write(_buffer[i])) { return false; }
    }
  }
  return true;
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           Megatec protocol compatible UPS SECTION

   http://networkupstools.org/protocols/megatec.html
*/
#define MEGATEC_UPS_UART_SPEED                2400 // Megatec-compatible UPS works on 2400 baud speed
#define MEGATEC_MAX_ANSWER_LENGTH             50   // Read no more 50 chars from UPS
#define MEGATEC_DEFAULT_READ_TIMEOUT          1000L

int32_t getMegatecUPSMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _command, uint8_t _fieldNumber, uint8_t* _outBuffer) {
  uint8_t command, len, srcPos, dstPos, fileldNumber;
  SoftwareSerial swSerial(_rxPin, _txPin);

  if (hstoba((char*) _command, (char*) _command, 1)) { _command[1] = '\0'; } ;
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
  len = serialRecive(&swSerial, _outBuffer, MEGATEC_MAX_ANSWER_LENGTH, MEGATEC_DEFAULT_READ_TIMEOUT, '\r');
  //Serial.print("len: "); Serial.println(len);
  //Serial.print("_outBuffer: "); Serial.println((char) _outBuffer);
  // return timeout sign if packet not finished by <CR>
  if ('\r' != _outBuffer[len-1]) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_TIMEOUT; 
  };
  if ('(' != _outBuffer[0]) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_WRONG_ANSWER; 
  };
  _outBuffer[len-1] = '\0';
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
  while ('\0' != _outBuffer[srcPos]) {
    // Just copy chars from the current "field" to begin of the output buffer
    _outBuffer[dstPos] = _outBuffer[srcPos];
    // The separator was found
    if (0x20 == _outBuffer[srcPos]) {
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
    } // if (0x20 == _outBuffer[srcPos]) .. else ..
    srcPos++; 
  };
  // make C-string
  _outBuffer[dstPos] = '\0';
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

#define APC_UPS_UART_SPEED                2400 // APC UPS works on 2400 baud speed
#define APC_MAX_ANSWER_LENGTH             30   // Read no more 30 chars from UPS
#define APC_DEFAULT_READ_TIMEOUT          1000L


/****************************  need to optimise startTimerOne(); calls   *****************************************/
int32_t getAPCSmartUPSMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _command, uint8_t _commandLen,  uint8_t* _outBuffer) {
  uint8_t command, 
          len, 
          sendTimes = 1;
  SoftwareSerial swSerial(_rxPin, _txPin);

  // _buffer used as input uint8_t array and output char array due it can save RAM. 
  // Data does not corrupt, because hstoba() write take two char (2 byte) and write one uin8_t (1 byte).  
  if (hstoba((char*) _command, (char*) _command, 1)) { _command[1] = '\0'; } ;
  
  
  // May be just make sum of buffer's bytes to use into switch: '^'+'A' => ...
  command = _command[0];
  // Serial.print("command: "); Serial.println(command, HEX);
  switch (command) {
     // Shutdown commands not working yet
     case 'K':   // Shutdown with grace period
     case 'Z':   // Shutdown immediately
          sendTimes = 2;
          
     case 0x01:  // ^A,  Model string
     case 'B':   // Battery voltage, V
     case 'C':   // Internal temperature, C
     case 'F':   // Line frequency, Hz
     case 'G':   // Cause of transfer   
     case 'L':   // Input line voltage, V
     case 'M':   // Maximum line voltage, V
     case 'N':   // Minimum line voltage, V
     case 'O':   // Output voltage, V
     case 'P':   // Power load, %
     case 'Q':   // Status flags
     case 'V':   // Firmware revision
     case 'X':   // Self-test results
//     case 'a':  // Protocol info (long string)
     case 'b':   // Firmware revision
     case 'c':   // UPS local id
     case 'e':   // Return threshold, %
     case 'g':   // Nominal battery voltage, V
     case 'f':   // Battery level, %
     case 'h':   // Measure-UPS: Ambient humidity. %
     case 'i':   // Measure-UPS: Dry contacts
     case 'j':   // Estimated runtime, min
     case 'l':   // Low transfer voltage, V
     case 'm':   // Manufacturing date
     case 'n':   // Serial number
     case 'u':   // Upper transfer voltage, V
     case 'v':   // Measure-UPS: Firmware
     case 'x':   // Last battery change 
     case 'y':   // Copyright notice
     case '7':   // Dip switch positions
     case '8':   // Register #3
     case '9':   // Line quality
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
  len = serialRecive(&swSerial, _outBuffer, 0x03, APC_DEFAULT_READ_TIMEOUT, '\0');
  // Connection timeout occurs (recieved less than 3 byte)
  if (len < 0x03) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_TIMEOUT; 
  }
  // Check for 'SM<>'
  if ( 'S' != _outBuffer[0] || 'M' != _outBuffer[1] || '\r' != _outBuffer[2]) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
     startTimerOne();
#endif
     return DEVICE_ERROR_WRONG_ANSWER; 
  };    
  
  
  //  Step #2. Send user's command & recieve answer
  //
  while (sendTimes) {
     // If not all data is recieved from talking device on step #1 - its RX buffer must be cleared 
     serialRXFlush(&swSerial, true);
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
     len = serialRecive(&swSerial, _outBuffer, APC_MAX_ANSWER_LENGTH, APC_DEFAULT_READ_TIMEOUT, '\r');
     //Serial.print("len: "); Serial.println(len);
     if ('\r' != _outBuffer[len-1]) { 
#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
        startTimerOne();
#endif
        return DEVICE_ERROR_TIMEOUT; 
     };
     _outBuffer[len-1] = '\0';
     //Serial.print("reply: "); Serial.println((char*) _outBuffer);
     
     sendTimes--;
     if (0 < sendTimes) { 
        // K-Command, Z-command - Send twice with > 1.5s delay between chars. 
        delay(1700); 
     }
  }
  
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

#define PZEM_UART_SPEED                    9600 // baud

#define PZEM_VOLTAGE                       0xB0
#define PZEM_CURRENT                       0xB1
#define PZEM_POWER                         0xB2
#define PZEM_ENERGY                        0xB3
#define PZEM_PACKET_SIZE                   0x07
#define PZEM_DEFAULT_READ_TIMEOUT          1000L

uint8_t crcPZEM004(uint8_t* _data, uint8_t _size) {
    uint16_t crc = 0;
    for(uint8_t i=0; i < _size; i++) { crc += (uint8_t) *_data; _data++;}
    //while (_size) { crc += *_data; _data++; _size--; }
    return (uint8_t)(crc & 0xFF);
}

int32_t getPZEM004Metric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const char* _ip, uint8_t* _buffer) {
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
    _buffer[0] = command; 
    // 2..5 bytes - ip address. Convert its from _ip or use default (192.168.1.1) if _ip is invalid
    result = hstoba((uint8_t*) &_buffer[1], _ip, 4);
    if (!result) {
       _buffer[1] = 0xC0;  // 192
       _buffer[2] = 0xA8;  // 168
       _buffer[3] = 0x01;  // 1
       _buffer[4] = 0x01;  // 1
    } 

    // 6-th byte - used to provide the value of the alarm threshold (in kW), 00 else
    _buffer[5] = 0x00; 
    // 7-th byte - CRC
    _buffer[6] = crcPZEM004(_buffer, PZEM_PACKET_SIZE - 1); 

#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
    // Stop the Timer1 to prevent UART errors
    stopTimerOne(); 
#endif
    //for(int i=0; i < sizeof(buffer); i++) { Serial.print("Byte# "); Serial.print(i); Serial.print(" => "); Serial.println(buffer[i], HEX);  }
    if (! serialSend(&swSerial, _buffer, PZEM_PACKET_SIZE, false)) { return DEVICE_ERROR_TIMEOUT; };

    /*  Recieve from PZEM004 */
    //Serial.println("Recieve answer...");
    len = serialRecive(&swSerial, _buffer, PZEM_PACKET_SIZE, PZEM_DEFAULT_READ_TIMEOUT, '\0');
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
    //Serial.print("Header real: "); Serial.println(_buffer[0], HEX);
    if (_buffer[0] != (command - 0x10)) { return DEVICE_ERROR_WRONG_ANSWER; }
    // Bad CRC
    if (_buffer[6] != crcPZEM004( _buffer, len - 1)) { return DEVICE_ERROR_CHECKSUM; }
    
//    Serial.println("Calculating...");
   // data is placed in buffer from 2-th byte, because 1-th byte is Header
   switch (_metric) {
     case SENS_READ_AC:
       result = ((_buffer[1] << 8) + _buffer[2]) * 100 + _buffer[3];
       // _buffer (cBuffer at real) cast to char due numeric-to-ascii subs require char array
       ltoaf(result, (char*) _buffer, 2);
       break;
     case SENS_READ_VOLTAGE:
       result = ((_buffer[1] << 8) + _buffer[2]) * 10 + _buffer[3]; 
       ltoaf(result, (char*) _buffer, 1);
       break;
     case SENS_READ_POWER:
       result = (_buffer[1] << 8) + _buffer[2];
       ltoa(result, (char*) _buffer, 10);
       break;
     case SENS_READ_ENERGY:
       result = ((uint32_t) _buffer[1] << 16) + ((uint16_t) _buffer[2] << 8) + _buffer[3];
       ltoa(result, (char*) _buffer, 10);
       break;
   }

  return RESULT_IN_BUFFER;
}

