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

int8_t getMegatecUPSMetric(const uint8_t _rxPin, const uint8_t _txPin, char* _command, uint8_t _fieldNumber, uint8_t* _dst) {
  int8_t rc = DEVICE_ERROR_TIMEOUT;
  uint8_t srcPos, dstPos, fileldNumber, expectedStartByte;
  int16_t len;
  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(MEGATEC_UPS_UART_SPEED);

  len = hstoba((uint8_t*) _command, _command);
  // Probaly _command is ASCII=string
  if (0 > len) { 
     // How much butes need to processing
     len = strlen(_command);
  }

  // Convert command to UPPERCASE in accordance with the requirements (i'm do not sure that is strict) of the Megatec protocol  
  strupr(_command);
  // add <CR> to the end of command
  _command[len] = '\r'; 
  len++;
  
  DTSD ( SerialPrint_P(PSTR("Command for UPS: ")); Serial.println(_command); )

  // Expected nothing on default
  expectedStartByte = 0x00; // 

  // Note that not all Megatec-"compatible" UPS runs all command and its returns the same as expected from proctocol
  // Some Ippon's ignore I command, for example
  switch (_command[0]) {
     case 'F':   // F - UPS Rating Information
     case 'I':   // I - UPS Information request
       expectedStartByte = '#'; 
       break;
     case 'Q':   
       // Q1 - Status Inquiry request?..
       if ('1' == _command[1]) {
          expectedStartByte = '('; 
       }
       // ... or just Q - Toggle the UPS beeper command 
       break;
     case 'S':   // S - Shut UPS output off in <n> minutes.
     case 'C':   // C - Cancel Shutdown Command & CT - Cancel Test Command
       break;
     default:
       rc = RESULT_IS_FAIL;
       goto finish; 
  }
  
  swSerial.begin(MEGATEC_UPS_UART_SPEED);
  //  Step #1. Send user's command & recieve answer
  //
  // Flush all device's transmitted data to avoid get excess data in recieve buffer
  //DTSD ( Serial.println("Step #1 - communicate to UPS.\n\tFlush Buffer"); )
  serialRXFlush(&swSerial, false);

  DTSD ( SerialPrintln_P(PSTR("Send command")); )
  serialSend(&swSerial, (uint8_t*) _command, len , !UART_SLOW_MODE);
  // Do not expect the answer? Just go out
  if (! expectedStartByte) { rc = RESULT_IS_OK; goto finish; }
  // Recieve answer from UPS. Answer placed to buffer directly for additional processing 
  DTSD ( SerialPrint_P(PSTR("Recieve answer")); )
  // We will expect to income no more MEGATEC_MAX_ANSWER_LENGTH bytes for MEGATEC_DEFAULT_READ_TIMEOUT milliseconds
  // Recieving can be stopped when '\r' char taken.
  // Need to use UART_SLOW_MODE with Megatec UPS?
  len = serialRecive(&swSerial, _dst, MEGATEC_MAX_ANSWER_LENGTH, MEGATEC_DEFAULT_READ_TIMEOUT, UART_STOP_ON_CHAR, '\r', !UART_SLOW_MODE);
  // DTSD ( Serial.print("Recieved "); Serial.print(len); Serial.println(" chars"); )
  // Answer must start with '(' or '#'
  if (expectedStartByte != _dst[0]) { rc = DEVICE_ERROR_WRONG_ANSWER; goto finish; }
  len--;
  // probaly recieve timeout occurs if packet do not finished by <CR>
  if ('\r' != _dst[len])  { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value
  _dst[len] = '\0';

  //  if (0 == fileldNumber) { rc = RESULT_IN_BUFFER; }
  
  //  Step #2. Search the field that specified by number & move data to the output buffer begin
  //           If the field number is greater than the available, then use the data from the last field.
  //
  // DTSD ( Serial.println("Step #2 - taking data from UPS answer"); )
  srcPos = 1; // start from 1-th byte to skip '(' prefix
  fileldNumber = dstPos = 0;
  // Walk over the recieved string while no EOL reached
  while ('\0' != _dst[srcPos]) {
    // Just copy chars from the current "field" to begin of the output buffer
    _dst[dstPos] = _dst[srcPos];
    // The separator was found
    if (0x20 != _dst[srcPos]) {
       dstPos++;
    } else {
       fileldNumber++;
       // Jump out from copy-on-search cycle if specified field number is found
       if (_fieldNumber == fileldNumber) { break; }
       // Otherwise - start next field writing from begin of the output buffer
       dstPos = 0;  
    } // if (0x20 != _dst[srcPos]) .. else ..
    srcPos++; 
  };
  // make C-string
  _dst[dstPos] = '\0';

  rc = RESULT_IN_BUFFER;

  finish:
  swSerial.~SoftwareSerial(); 
  return rc;
}

