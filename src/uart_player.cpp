// Config & common included files
#include "sys_includes.h"

#include <SoftwareSerial.h>

#include "service.h"
#include "system.h"

#include "uart_bus.h"
#include "uart_player.h"


/*****************************************************************************************************************************
*
*   Calculate CRC of DFPlayer Mini data packet
*
*   Returns: 
*     - CRC
*
*****************************************************************************************************************************/
//calc checksum (1~6 byte)
static uint16_t crcDFPlayerMini(uint8_t* _data) {
    uint16_t crc = 0x00;
    for (uint8_t i = DFPLAYER_MINI_BYTE_VERSION; i < DFPLAYER_MINI_BYTE_CRC_00; i++) { 
        crc += _data[i]; 
    }
    return (0xFFFF-crc+1);
}

/*****************************************************************************************************************************
*
*   Send command to DFPlayer Mini and check result of operation
*
*   Returns: 
*     DEVICE_ERROR_TIMEOUT on device stop talking case
*     DEVICE_ERROR_CHECKSUM when bad CRC is recieved
*     DEVICE_ERROR_WRONG_ANSWER if DFPlayer Mini answer is not "success" (0x41)
*     RESULT_IS_FAIL on other fails
*
*****************************************************************************************************************************/

static int8_t sendToDFPlayerMini(Stream* _stream, uint8_t _command, uint16_t _option, uint8_t* _dst) {
  int8_t rc = RESULT_IS_FAIL;
  uint16_t calculatedCRC;

  _dst[DFPLAYER_MINI_BYTE_HEADER]   = DFPLAYER_MINI_HEADER;   // 0
  _dst[DFPLAYER_MINI_BYTE_VERSION]  = DFPLAYER_MINI_VERSION;  // 1
  _dst[DFPLAYER_MINI_BYTE_LENGHT]   = DFPLAYER_MINI_PAYLOAD_LENGHT; // 2
  _dst[DFPLAYER_MINI_BYTE_FEEDBACK] = DFPLAYER_MINI_FEEDBACK; // 4
  _dst[DFPLAYER_MINI_BYTE_COMMAND]  = _command;
  _dst[DFPLAYER_MINI_BYTE_PARAM_00] = (uint8_t) (_option >> 8);
  _dst[DFPLAYER_MINI_BYTE_PARAM_01] = (uint8_t) _option;
  // Normally it’s okay whether users choose to use checksum or not, our module can receive a serial data with or without checksum
  // FN-M16P Embedded MP3 Audio Module Datasheet V1.0 pg.5
  calculatedCRC = crcDFPlayerMini(_dst);
  _dst[DFPLAYER_MINI_BYTE_CRC_00]   = (uint8_t) (calculatedCRC >> 8);
  _dst[DFPLAYER_MINI_BYTE_CRC_01]   = (uint8_t) calculatedCRC;

  _dst[DFPLAYER_MINI_BYTE_END]   = DFPLAYER_MINI_END;

  // Flush all device's transmitted data to avoid get excess data in recieve buffer
  flushStreamRXBuffer(_stream, DFPLAYER_MINI_DEFAULT_READ_TIMEOUT, !UART_SLOW_MODE);

//   for (uint8_t i = 0; i < DFPLAYER_MINI_PACKET_SIZE; i++) { Serial.print("buff["); Serial.print(i); Serial.print("] = 0x"); Serial.println(_dst[i], HEX);  }
 
  serialSend(_stream, _dst, DFPLAYER_MINI_PACKET_SIZE, !UART_SLOW_MODE);

#if (0x01 == DFPLAYER_MINI_FEEDBACK) 
     uint8_t len;
     uint16_t recievedCRC;

     //  Recieve answer from DFPlayer Mini
     //  It actually do not use '\r', '\n', '\0' to terminate string
     len = serialRecive(_stream, _dst, DFPLAYER_MINI_PACKET_SIZE, DFPLAYER_MINI_DEFAULT_READ_TIMEOUT, !UART_STOP_ON_CHAR, '\r', !UART_SLOW_MODE);
   
     // Connection timeout occurs
     if (len < DFPLAYER_MINI_PACKET_SIZE) { rc = DEVICE_ERROR_TIMEOUT; goto finish; }
     calculatedCRC = crcDFPlayerMini(_dst);
     
     recievedCRC = ((uint16_t) _dst[DFPLAYER_MINI_BYTE_CRC_00] << 8) | _dst[DFPLAYER_MINI_BYTE_CRC_01];
   
     // Bad CRC
     if (recievedCRC != calculatedCRC) {
        rc = DEVICE_ERROR_CHECKSUM; goto finish; 
     }
   
     // Wrong answer. buffer[3] must contain 0x41
     if (DFPLAYER_MINI_REPLY_SUCCESS != _dst[DFPLAYER_MINI_BYTE_COMMAND]) {
        rc = DEVICE_ERROR_WRONG_ANSWER; goto finish; 
     }
#endif

  rc = RESULT_IS_OK;

finish:
  return rc;
}


/*****************************************************************************************************************************
*
*   Run specified commad with specified volume on DFPlayer Mini
*
*   Returns: 
*     DEVICE_ERROR_TIMEOUT on device stop talking case
*     DEVICE_ERROR_CHECKSUM when bad CRC is recieved
*     DEVICE_ERROR_WRONG_ANSWER if DFPlayer Mini answer is not "success" (0x41)
*     DEVICE_ERROR_NOT_SUPPORTED when wrong command is given
*     RESULT_IS_FAIL on other fails
*
*****************************************************************************************************************************/
int8_t runDFPlayerMini(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _command, uint16_t _option, int16_t _volume, uint8_t* _dst) {

  int8_t rc = DEVICE_ERROR_NOT_SUPPORTED;
  uint32_t waitTime;
  static uint32_t lastWorkTime = 0x00;
  SoftwareSerial swSerial(_rxPin, _txPin);

  if (DFPLAYER_CMD_SET_DAC < _command) {
     goto finish; 
  }

  swSerial.begin(DFPLAYER_MINI_UART_SPEED);

  waitTime = millis() - lastWorkTime;

  // After the module is powered on, normally it needs about no more than 500ms to 1500ms(depending on the
  // actual track quantities in the storage device) initialization time.
  // MCU can not send commands to control the module until the initialization of the module is done and a data is
  // returned, otherwise the commands sent by MCU will be ignored and also this will effect initializing of the module
  // 
  waitTime = (waitTime < DFPLAYER_MINI_INACTIVE_INTERVAL) ? (DFPLAYER_MINI_INACTIVE_INTERVAL - waitTime) : 0;

  delay(waitTime); 

  if (0 <= _volume) {
     rc = sendToDFPlayerMini(&swSerial, DFPLAYER_CMD_SET_VOLUME, _volume, _dst);
     if (RESULT_IS_OK == rc) { goto finish; } 
     delay(DFPLAYER_MINI_INACTIVE_INTERVAL);
  }

  /*  Send to DFPlayer Mini */
  rc = sendToDFPlayerMini(&swSerial, _command, _option, _dst);
  lastWorkTime = millis();

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  swSerial.~SoftwareSerial(); 
  return rc;
}

