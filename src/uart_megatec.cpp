#include "uart_megatec.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                     MEGATEC-COMPATIBLE UPS SECTION
 
   http://networkupstools.org/protocols/megatec.html

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

/*****************************************************************************************************************************
*
*   Read values of the specified metric from the Megatec-compatible UPS, put it to output buffer on success. 
*
*   Not tested yet.
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*
*****************************************************************************************************************************/
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
  //  Step #1. Send user's command & recieve answer
  //
  // Flush all device's transmitted data to avoid get excess data in recieve buffer
  serialRXFlush(&swSerial, false);

  if (! serialSend(&swSerial, _command, len , false)) {
     return DEVICE_ERROR_TIMEOUT; 
  };
  // Recieve answer from UPS. Answer placed to buffer directly for additional processing 
  len = serialRecive(&swSerial, _dst, MEGATEC_MAX_ANSWER_LENGTH, MEGATEC_DEFAULT_READ_TIMEOUT, '\r', false);
  // return timeout sign if packet not finished by <CR>
  if ('\r' != _dst[len-1]) { 
     return DEVICE_ERROR_TIMEOUT; 
  };
  if ('(' != _dst[0]) { 
     return DEVICE_ERROR_WRONG_ANSWER; 
  };
  _dst[len-1] = '\0';
  if (0 == fileldNumber) { 
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
  swSerial.~SoftwareSerial();

  return RESULT_IN_BUFFER;
}



