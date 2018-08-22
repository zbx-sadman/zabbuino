// Config & common included files
#include "sys_includes.h"

#include <SoftwareSerial.h>

#include "service.h"
#include "system.h"

#include "uart_bus.h"
#include "uart_pzem.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

Based on: https://github.com/olehs/PZEM004T
version 1.0 is used

*/


/*****************************************************************************************************************************
*
*   Calculate CRC of Peacefair PZEM-004 data packet
*
*   Returns: 
*     - CRC
*
*****************************************************************************************************************************/
static uint8_t crcPZEM004(uint8_t* _data, uint8_t _size) {
    uint16_t crc = 0;
//    for(uint8_t i=0; i < _size; i++) { crc += (uint8_t) *_data; _data++;}
    for(uint8_t i = 0; i < _size; i++) { crc += _data[i]; }
    //while (_size) { crc += *_data; _data++; _size--; }
    return (uint8_t)(crc & 0xFF);
}

/*****************************************************************************************************************************
*
*   Read values of the specified metric from the Peacefair PZEM-004 Energy Meter, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*
*****************************************************************************************************************************/
int8_t getPZEM004Metric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const char* _ip, uint8_t* _dst) {

  int8_t rc = RESULT_IS_FAIL;
  uint8_t command, len;
  int32_t result;
  uint32_t waitTime;
  static uint32_t lastReadTime = 0x00;


  SoftwareSerial swSerial(_rxPin, _txPin);

  swSerial.begin(PZEM_UART_SPEED);

  switch (_metric) {
    case SENS_READ_AC:
      command = PZEM_CMD_CURRENT; 
      break;
    case SENS_READ_VOLTAGE:
      command = PZEM_CMD_VOLTAGE; 
      break;
    case SENS_READ_POWER:
      command = PZEM_CMD_POWER; 
      break;
    case SENS_READ_ENERGY:
      command = PZEM_CMD_ENERGY; 
      break;
    case SENS_CHANGE_ADDRESS:
      command = PZEM_CMD_SETADDR; 
      break;
    default:
      goto finish; 
  }

  waitTime = millis() - lastReadTime;

  // PZEM004 answer twice in second only
  waitTime = (waitTime < 1000) ? (1000 - waitTime) : 0;
  delay(waitTime); 

  /*  Send to PZEM004 */
  
  // Make packet for PZEM
  // 1-th byte in the packet - metric (command)
  _dst[0] = command; 
  // 2..5 bytes - ip address. Convert its from _ip or use default (192.168.1.1) if _ip is invalid
  if (4 != hstoba(&_dst[1], _ip)) {
     _dst[1] = 0xC0;  // 192
     _dst[2] = 0xA8;  // 168
     _dst[3] = 0x01;  // 1
     _dst[4] = 0x01;  // 1
  } 

  // 6-th byte - used to provide the value of the alarm threshold (in kW), 00 else
  _dst[5] = 0x00; 
  // 7-th byte - CRC
  _dst[6] = crcPZEM004(_dst, PZEM_PACKET_SIZE - 1); 
  // Serial.println("Send: ");  for(int i=0; i < PZEM_PACKET_SIZE; i++) { Serial.print("Byte# "); Serial.print(i); Serial.print(" => "); Serial.println(_dst[i], HEX);  }
  // Flush all device's transmitted data to avoid get excess data in recieve buffer
  serialRXFlush(&swSerial, !UART_SLOW_MODE);
  serialSend(&swSerial, _dst, PZEM_PACKET_SIZE, !UART_SLOW_MODE);

  //  Recieve from PZEM004
  //  It actually do not use '\r', '\n', '\0' to terminate string
  len = serialRecive(&swSerial, _dst, PZEM_PACKET_SIZE, PZEM_DEFAULT_READ_TIMEOUT, !UART_STOP_ON_CHAR, '\r', !UART_SLOW_MODE);

  //Serial.println("Recieve: "); for(int i=0; i < len; i++) { Serial.print("Byte# "); Serial.print(i); Serial.print(" => "); Serial.println(_dst[i], HEX);  }
  // Connection timeout occurs
  // if (len != PZEM_PACKET_SIZE) { return DEVICE_ERROR_TIMEOUT; }
  if (len < PZEM_PACKET_SIZE) { rc = DEVICE_ERROR_TIMEOUT; goto finish; }
  // Wrong answer. buffer[0] must contain command - 0x10 (command B1 -> reply A1)
  // command = command - 0x10;
  if (_dst[0] != (command - 0x10)) { rc = DEVICE_ERROR_WRONG_ANSWER; goto finish; }
  // Bad CRC
  if (_dst[6] != crcPZEM004( _dst, len - 1)) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }
  
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
    case SENS_CHANGE_ADDRESS:
      // All returned bytes must be 0x00 on success, CRC must be equal (command - 0x10)
      result = _dst[1] | _dst[2] | _dst[3] | _dst[4] | _dst[5];
      rc = result ? RESULT_IS_FAIL : RESULT_IS_OK;
      goto finish;         
      break;
  }
  rc = RESULT_IS_BUFFERED;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  swSerial.~SoftwareSerial(); 
  return rc;
}

