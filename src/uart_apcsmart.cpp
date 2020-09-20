// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "uart_bus.h"
#include "uart_apcsmart.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                         APC SMART-UPS SECTION
 
   Despite the lack of official information from APC, this table has been constructed. It’s standard RS-232 serial communications at 2400 bps/8N1. 
   Don’t rush the UPS while transmitting or it may stop talking to you. This isn’t a problem with the normal single character queries, but it really 
   does matter for multi-char things like "@000". Sprinkle a few calls to usleep() in your code and everything will work a lot better.
   http://networkupstools.org/protocols/apcsmart.html

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the APC Smart UPS, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*
*****************************************************************************************************************************/
//int8_t getAPCSmartUPSMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t *_command, uint8_t _commandLen,  uint8_t *_dst) {
int8_t getAPCSmartUPSMetric(const uint8_t _rxPin, const uint8_t _txPin, char* _command, char* _dst) {
  int8_t rc = RESULT_IS_FAIL;
  uint8_t command, 
          len, 
          sendTimes,
          sendCommandTwice = false;
          
  SoftwareSerial swSerial(_rxPin, _txPin);

  // _command used as input uint8_t array and output char array due it can save RAM. 
  // Data does not corrupt, because hstoba() write take two char (2 byte) and write one uint8_t (1 byte).  
  if (0x00 < hstoba((uint8_t*)_command,  _command)) { _command[0x01] = '\0'; } ;
  
  
  // May be just make sum of buffer's bytes to use into switch: '^'+'A' => ...
  command = _command[0x00];
  // DEBUG_PORT.print("command: "); DEBUG_PORT.println(command, HEX);
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
    //   DEBUG_PORT.println("Command allowed");
       break;
     default:
       goto finish;
  }
  
  rc = DEVICE_ERROR_TIMEOUT;
  swSerial.begin(APC_UPS_UART_SPEED);
  //  Step #1. Send Y-command. Its must return 'SM<0x0D>' (3 byte) and switch UPS to Smart mode
  //
  // Flush all device's transmitted data to avoid get excess data in recieve buffer
  // APC UPS can be flushed in slow mode
  flushStreamRXBuffer(&swSerial, APC_DEFAULT_READ_TIMEOUT, UART_SLOW_MODE);

  command = 'Y';
  serialSend(&swSerial, &command, 0x01, true);
  __DMLD( DEBUG_PORT.println(F("Recieving from UPS")); )

  len = serialRecive(&swSerial, (uint8_t*) _dst, 0x03, APC_DEFAULT_READ_TIMEOUT, UART_STOP_ON_CHAR, '\r', UART_SLOW_MODE);
  __DMLD( DEBUG_PORT.print(F("len: ")); DEBUG_PORT.println(len, DEC); )

  // Connection timeout occurs (recieved less than 3 byte)
  if (0x03 > len) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value
  // Check for "SM\r"
  if ( 'S' != _dst[0x00] || 'M' != _dst[0x01] || '\r' != _dst[0x02]) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value
  
  
  //  Step #2. Send user's command & recieve answer
  //
  // If not all data is recieved from talking device on step #1 - its RX buffer must be cleared 
  //serialRXFlush(&swSerial, true);
  flushStreamRXBuffer(&swSerial, APC_DEFAULT_READ_TIMEOUT, UART_SLOW_MODE);
  sendTimes = sendCommandTwice ? 0x02 : 0x01;

  while (sendTimes) {
     // All commands fits to 1 byte
     //DEBUG_PORT.println("send");
     serialSend(&swSerial, (uint8_t*) _command, 0x01, true);
     //DEBUG_PORT.println("recieve");
      // Recieve answer from Smart UPS. Answer placed to buffer directly and does not require additional processing 
     len = serialRecive(&swSerial, (uint8_t*) _dst, APC_MAX_ANSWER_LENGTH, APC_DEFAULT_READ_TIMEOUT, UART_STOP_ON_CHAR, '\r', UART_SLOW_MODE);
     //DEBUG_PORT.print("len: "); DEBUG_PORT.println(len);
     if (!sendCommandTwice && '\r' != _dst[len-1]) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value
     //DEBUG_PORT.print("reply: "); DEBUG_PORT.println((char*) _dst);

     sendTimes--;
     if (0x00 < sendTimes) { 
        // Zabbuino always return 1, because UPS always return nothing
        _dst[0x00] = '1';  _dst[0x01] = '\0';
        // ^N, K-, Z- commands must send twice with 1.5s...3s delay between chars. 
        delay(1700); 
     }
  }

  // Make C-string by replacing '\r' to '\0'
  _dst[len - 0x01] = '\0';
  // 'j'-сommand return number of mins with trailing ':'. We need destroy this obstacle to let Zabbix convert its to numeric value properly.
  if ('j' == _command[0x00]) {_dst[len - 0x02] = '\0';}   
     
  //  Step #3. Send R-command. It's must return 'BYE' and switch UPS to Dumb mode
  //
  // 
  command = 'R';
  serialSend(&swSerial, &command, 0x01, true);
  //serialRXFlush(&swSerial, true);
  flushStreamRXBuffer(&swSerial, APC_DEFAULT_READ_TIMEOUT, UART_SLOW_MODE);
 
  rc = RESULT_IS_BUFFERED;

finish:
  gatherSystemMetrics(); // Measure memory consumption
  swSerial.~SoftwareSerial(); 
  return rc;
}

