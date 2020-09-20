// Config & common included files
#include "sys_includes.h"

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
*  Calculate CRC of Peacefair PZEM-004 data packet
*
*  Returns: 
*    - CRC
*
*****************************************************************************************************************************/
static uint8_t crcPZEM004(uint8_t* _data, uint8_t _size) {
    uint16_t crc = 0x00;
//    while (_size) {_size--; crc += *_data; _data++; }
    for (uint8_t i = 0x00; i < _size; i++) { crc += _data[i]; }
    return (uint8_t)(crc & 0xFF);
}

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Peacefair PZEM-004 Energy Meter, put it to output buffer on success. 
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success when POWER/ENERGY metric specified
*    - RESULT_IS_FLOAT_02_DIGIT    on success when CURRENT(AC) metric specified
*    - RESULT_IS_FLOAT_01_DIGIT    on success when VOLTAGE metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_WRONG_ANSWER   on "error" answer
*    - DEVICE_ERROR_CHECKSUM       on bad checksum recieved
*
*****************************************************************************************************************************/
int8_t getPZEM004Metric(const uint8_t _rxPin, const uint8_t _txPin, const char* _ip, uint8_t _metric, int32_t* _value) {

  int8_t rc = DEVICE_ERROR_NOT_SUPPORTED;
  uint8_t command, len;
  uint32_t waitTime;
  static uint32_t lastReadTime = 0x00;

  uint8_t data[PZEM_PACKET_SIZE];

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
  waitTime = (waitTime < MSEC_PER_SECOND) ? (MSEC_PER_SECOND - waitTime) : 0x00;
  delay(waitTime); 

  /*  Send to PZEM004 */
  
  // Make packet for PZEM
  // 1-th byte in the packet - metric (command)
  data[0x00] = command; 
  // 2..5 bytes - ip address. Convert its from _ip or use default (192.168.1.1) if _ip is invalid
  if (0x04 != hstoba(&data[0x01], _ip)) {
     data[0x01] = 0xC0;  // 192
     data[0x02] = 0xA8;  // 168
     data[0x03] = 0x01;  // 1
     data[0x04] = 0x01;  // 1
  } 

  // 6-th byte - used to provide the value of the alarm threshold (in kW), mus be 0x00 if not used
  data[0x05] = 0x00; 
  // 7-th byte - CRC
  data[0x06] = crcPZEM004(data, PZEM_PACKET_SIZE - 1); 
  // Flush all device's transmitted data to avoid get excess data in recieve buffer
  flushStreamRXBuffer(&swSerial, PZEM_DEFAULT_READ_TIMEOUT, !UART_SLOW_MODE);

  serialSend(&swSerial, data, PZEM_PACKET_SIZE, !UART_SLOW_MODE);

  //  Recieve from PZEM004
  //  It actually do not use '\r', '\n', '\0' to terminate string
  len = serialRecive(&swSerial, data, PZEM_PACKET_SIZE, PZEM_DEFAULT_READ_TIMEOUT, !UART_STOP_ON_CHAR, '\r', !UART_SLOW_MODE);

  // Connection timeout occurs
  if (len < PZEM_PACKET_SIZE) { rc = DEVICE_ERROR_TIMEOUT; goto finish; }
  // Wrong answer. buffer[0] must contain command - 0x10 (command B1 -> reply A1)
  // command = command - 0x10;
  if (data[0x00] != (command - 0x10)) { rc = DEVICE_ERROR_WRONG_ANSWER; goto finish; }
  // Bad CRC
  if (data[0x06] != crcPZEM004( data, len - 1)) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }
  
  // data is placed in buffer from 2-th byte, because 1-th byte is Header
  switch (_metric) {
    case SENS_READ_AC: {
      // Current range: 0 ~ 100A (0 ... 99.99)
      // Packet with max values from range: { 0x00 0x63 0x63 } (-- 99.99)
      // Reply current data is D2D3 = 11 20, 11 represent the integer-bit of the current, 20 represent the decimal of the current, the decimal is two digit
      // Test for overflow: data[2] autocasted to int (int16_t), 0xFF*100+0xFF = 0x649B, 0x649B < 0x7FFF (int16_t max), NO OVERFLOW
      *_value = data[0x02] * 100 + data[0x03];
      rc = RESULT_IS_FLOAT_02_DIGIT;
      break;
    }

    case SENS_READ_VOLTAGE: {
      // Current range: 80 ~ 260VAC (110.0 ... 220.0)
      // Packet with max values from range: { 0x01 0x04 0x09 } (260.09)
      // Reply voltage data is D1D2D3 = 00 E6 02, 00 E6 represent the integer-bit of the voltage, 02 represent the decimal of the voltage, the decimal is one digit
      // 
      // Test for overflow: data[1] casted to int32_t, ((0xFF << 8) + 0xFF) * 10 + 0xFF = 0xA00F5, 0xA00F5 < 0x7FFFFFFF (int32_t max), NO OVERFLOW
      *_value = (((int32_t) data[0x01] << 0x08) + data[0x02]) * 10 + data[0x03]; 
      rc = RESULT_IS_FLOAT_01_DIGIT;
      break;
    }

    case SENS_READ_POWER: {
      // Power range: 0 ~ 22KW (0 ... 2200)
      // Packet with max values from range: { 0x08 0x98 0x00 } (2200)
      // Reply power data is D1D2 = 08 98, converts 08 98 to decimal is 2200
      //
      // Test for overflow: data[1] casted to uint16_t, (0xFF << 8) + 0xFF = 0xFFFF, 0xFFFF == 0xFFFF (uint16_t max), NO OVERFLOW
      *_value = ((uint16_t) data[0x01] << 0x08) + data[0x02];
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;
    }

    case SENS_READ_ENERGY: {
      // Energy range: 0 ~ 99999W (0 ... 99999)
      // Packet with max values from range: { 0x01 0x86 0x9F } (99999)
      // Reply energy data is D1D2D3 = 01 86 9F, converts 01 86 9F to decimal is 99999
      //
      // Test for overflow: data[1] casted to int32_t, data[1] casted to uint16_t, (0xFF << 16) + (0xFF << 8) + 0xFF
      // 0xFF0000 + 0xFF00 + 0xFF = 0xFFFFFF, 0xFFFFFF < 0x7FFFFFFF (int32_t max), NO OVERFLOW
      *_value = ((int32_t) data[0x01] << 0x10) + ((uint16_t) data[0x02] << 0x08) + data[0x03];
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;
    }

    case SENS_CHANGE_ADDRESS: {
      // All returned bytes must be 0x00 on success, CRC must be equal (command - 0x10)
      *_value = data[0x01] | data[0x02] | data[0x03] | data[0x04] | data[0x05];
      rc = *_value ? RESULT_IS_FAIL : RESULT_IS_OK;
      goto finish;         
      break;
    }

    default:
      break;
  }

finish:
  gatherSystemMetrics(); // Measure memory consumption
  swSerial.~SoftwareSerial(); 
  return rc;
}

