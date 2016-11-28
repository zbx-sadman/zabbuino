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
uint8_t crcPZEM004(uint8_t *_data, uint8_t _size) {
    uint16_t crc = 0;
    for(uint8_t i=0; i < _size; i++) { crc += (uint8_t) *_data; _data++;}
    //while (_size) { crc += *_data; _data++; _size--; }
    return (uint8_t)(crc & 0xFF);
}

/*****************************************************************************************************************************
*
*   Read values of the specified metric from the Peacefair PZEM-004 Energy Meter, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*
*****************************************************************************************************************************/
int8_t getPZEM004Metric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const char *_ip, uint8_t *_dst) {
  int8_t rc = RESULT_IS_FAIL;
  uint8_t command, len;
  int32_t result;
  SoftwareSerial swSerial(_rxPin, _txPin);

  swSerial.begin(PZEM_UART_SPEED);

   switch (_metric) {
     case SENS_READ_AC:
       command = PZEM_CURRENT; 
       break;
     case SENS_READ_VOLTAGE:
       command = PZEM_VOLTAGE; 
       break;
     case SENS_READ_POWER:
       command = PZEM_POWER; 
       break;
     case SENS_READ_ENERGY:
       command = PZEM_ENERGY; 
       break;
   }

   /*  Send to PZEM004 */
   
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
    // for(int i=0; i < sizeof(_dst); i++) { Serial.print("Byte# "); Serial.print(i); Serial.print(" => "); Serial.println(_dst[i], HEX);  }
    serialSend(&swSerial, _dst, PZEM_PACKET_SIZE, false);

    //  Recieve from PZEM004
    //  It actually do not use '\r', '\n', '\0' to terminate string
    len = serialRecive(&swSerial, _dst, PZEM_PACKET_SIZE, PZEM_DEFAULT_READ_TIMEOUT, !UART_STOP_ON_CHAR, '\r', !UART_SLOW_MODE);

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
    }
  rc = RESULT_IN_BUFFER;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  swSerial.~SoftwareSerial(); 
  return rc;
}

