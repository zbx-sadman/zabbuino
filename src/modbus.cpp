// Config & common included files
#include "sys_includes.h"

#include <SoftwareSerial.h>

#include "service.h"
#include "system.h"

#include "uart_bus.h"
#include "modbus.h"


static uint16_t modbusRtuCalcCRC(uint8_t* _dst, const uint8_t _len) {
  uint16_t temp, temp2, flag;
  temp = 0xFFFF;
  for (uint8_t i = 0x00; _len > i; i++) {
    temp = temp ^ _dst[i];
    for (uint8_t j = 1; j <= 8; j++) {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag) { temp ^= 0xA001; }
    }
  }
  // Reverse byte order.
  temp2 = temp >> 0x08;
  temp = (temp << 0x08) | temp2;
  temp &= 0xFFFF;
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp;
}

static void modbusRtuProcessingFC(const uint8_t _functionCode, uint8_t* _buffer, const uint8_t _registersNum) {
  switch (_functionCode) {
    case MODBUS_FUNCTION_CODE_03:
    case MODBUS_FUNCTION_CODE_04: {
        // Modbus used Big-Endian, but we need Little-endian on AVR8.
        // So, just swap bytes to make correct 16-bit values in "Register values" part of buffer
        for (uint8_t i = 0x00, currentByte = MODBUS_PACKET_ANSWER_HEADER_LENGTH; _registersNum > i; i++, currentByte += 0x02) {
          uint8_t swap = _buffer[currentByte];
          _buffer[currentByte] = _buffer[currentByte + 0x01];
          _buffer[currentByte + 0x01] = swap;
        }
        break;
      }
  }
}

static void modbusRtuMakeQueryHeader(const uint8_t _functionCode, const uint8_t _slaveId, const uint16_t _firstRegister, uint8_t* _buffer, const uint8_t _registersNum) {
  _buffer[MODBUS_PACKET_SLAVE_ID] = _slaveId;
  // Make PDU
  _buffer[MODBUS_PACKET_FUNCTION_CODE] = _functionCode;

  _buffer[MODBUS_PACKET_QUERY_REGISTER_ADDRESS] = (_firstRegister >> 0x08) & 0xFF;
  _buffer[MODBUS_PACKET_QUERY_REGISTER_ADDRESS + 0x01] = _firstRegister & 0xFF;

  _buffer[MODBUS_PACKET_QUERY_REGISTERS_COUNT] = (_registersNum >> 0x08) & 0xFF;
  _buffer[MODBUS_PACKET_QUERY_REGISTERS_COUNT + 0x01] = _registersNum & 0xFF;
  //  /PDU

  uint16_t crc = modbusRtuCalcCRC(_buffer, MODBUS_PACKET_QUERY_CRC);

  _buffer[MODBUS_PACKET_QUERY_CRC] = (crc >> 0x08) & 0xFF;
  _buffer[MODBUS_PACKET_QUERY_CRC + 1] = crc & 0xFF;
}

static modbusQueryState_t modbusRtuValidateAnswer(const uint8_t _slaveId, uint8_t* _src, const uint8_t _registersNum, const uint8_t _len) {
  uint8_t recievedBytes;
  modbusQueryState_t rc = mqsSuccess;

  uint16_t crc = modbusRtuCalcCRC(_src, _len);

  //DEBUG_PORT.print("CRC: "); DEBUG_PORT.println(crc , HEX);
  // CRC != 0 on error
  if (crc) { rc = mqsBadCRC; goto finish; }

  // Other slave was answered?
  if (_slaveId != _src[MODBUS_PACKET_SLAVE_ID]) { rc = mqsWrongAnswer; goto finish; }

  // Wrong answer size?
  if (_registersNum != (_src[MODBUS_PACKET_ANSWER_DATA_BYTES_COUNT] / 0x02)) { rc = mqsWrongAnswer; goto finish; }

  // Error was returned?
  if (MODBUS_FUNCTION_CODE_ERROR_MASK & _src[MODBUS_PACKET_FUNCTION_CODE]) {
    switch (_src[MODBUS_PACKET_ANSWER_EXCEPTION_CODE]) {
      // Illegal Function
      case 0x01: { rc = mqsExceptionIllegalFunction; break; }
      // Illegal Data Address
      case 0x02: { rc = mqsExceptionIllegalDataAddress; break; }
      // Illegal Data Value
      case 0x03: { rc = mqsExceptionIllegalDataValue; break; }
      // Slave Device Failure
      case 0x04: { rc = mqsExceptionSlaveDeviceFailure; break; }
      // Acknowledge
      case 0x05: { rc = mqsExceptionAcknowledge; break; }
      // Slave Device Busy
      case 0x06: { rc = mqsExceptionSlaveDeviceBusy; break; }
      // Negative Acknowledge
      case 0x07: { rc = mqsExceptionNegativeAcknowledge; break; }
      // Memory Parity Error
      case 0x08: { rc = mqsExceptionMemoryParityError; break; }
      // Unknown
      default: { rc = mqsExceptionUnknown; break; }
    } // switch (_dst[MODBUS_PACKET_ANSWER_EXCEPTION_CODE])
    goto finish;
  } // if (MODBUS_FUNCTION_CODE_ERROR_MASK & _dst[MODBUS_PACKET_FUNCTION_CODE])

  recievedBytes = _len - MODBUS_PACKET_ANSWER_FRAME_LENGTH;
  if (recievedBytes != _src[MODBUS_PACKET_ANSWER_DATA_BYTES_COUNT]) { rc = mqsWrongAnswer; }

finish:
  return rc;
}


int8_t modbusRtuUart(const uint8_t _rxPin, const uint8_t _txPin, const int8_t _enaPin, const uint16_t _uartSpeed, const uint8_t _functionCode, const uint8_t _slaveId, const uint16_t _firstRegister, uint8_t* _buffer, const uint8_t _registersNum) {
  int8_t rc = RESULT_IS_FAIL;

  modbusQueryState_t modbusQueryState;

  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(_uartSpeed);

  if (_enaPin >= 0x00) { pinMode(_enaPin, OUTPUT); pinMode(_enaPin, HIGH); }

  modbusRtuMakeQueryHeader(_functionCode, _slaveId, _firstRegister, _buffer, _registersNum);

  __DMLD( DEBUG_PORT.println(F("Req")); 
    for (uint8_t i = 0x00; MODBUS_PACKET_QUERY_LENGTH > i; i++) {
        DEBUG_PORT.print(i); DEBUG_PORT.print(" => 0x"); DEBUG_PORT.println(_buffer[i], HEX);
    }
  )
  uint8_t answerExpectedBytes = _registersNum * 0x02 + MODBUS_PACKET_ANSWER_FRAME_LENGTH; // N * 2-byte registers number + 2 byte CRC
  //DEBUG_PORT.print("Expected: "); DEBUG_PORT.println(answerExpectedBytes , HEX);

  __DMLD( DEBUG_PORT.println(F("Ans")); )
  uint32_t idleStartTime, recievingStartTime, t35Duration;
  uint8_t incomingBytes = 0x00;
  modbusReadState_t readState = mrsProcessing;

  //t35Duration = (1000UL / (_uartSpeed / MODBUS_BITS_PER_SYMBOL)) * 3.5 => (1000UL  * 3.5 * MODBUS_BITS_PER_SYMBOL) / _uartSpeed
  t35Duration = (3500UL * MODBUS_BITS_PER_SYMBOL) / _uartSpeed;

  swSerial.write(_buffer, MODBUS_PACKET_QUERY_LENGTH);

  recievingStartTime = millis();
  idleStartTime = micros();

  if (_enaPin >= 0x00) { pinMode(_enaPin, LOW); }

  while (readState == mrsProcessing) {

    if (millis() - recievingStartTime >= MODBUS_TRANSACTION_TIMEOUT) { readState = mrsErrorTimeout; }

    uint32_t nowMicros = micros();

    // nowMicros is give less cli/sei calls (inside micros()) using, but we need place test for timeout after read block
    // Otherwise any debug messages output eat ticks and make time gap, than caused premature stopping with mrsReadingDone status
    if (0 < swSerial.available()) {
      if (incomingBytes > answerExpectedBytes) {
        readState = mrsErrorOverflow;
        continue;
      }
      _buffer[incomingBytes] = swSerial.read();
      __DMLD( DEBUG_PORT.print(incomingBytes); DEBUG_PORT.print(" => 0x"); DEBUG_PORT.println(_buffer[incomingBytes], HEX); )
      incomingBytes++;
      idleStartTime = nowMicros;
    }
    if ((0 < incomingBytes) && (nowMicros - idleStartTime >= t35Duration)) { readState = mrsReadingDone; }
  }

  //DEBUG_PORT.print(F("Rcv in: ")); DEBUG_PORT.println(millis() - recievingStartTime);

  modbusQueryState = mqsUnexpected;

  switch (readState) {
    case mrsReadingDone: { modbusQueryState = modbusRtuValidateAnswer(_slaveId, _buffer, _registersNum, incomingBytes); break; }
    case mrsErrorTimeout: { modbusQueryState = mqsDeviceTimeout; rc = DEVICE_ERROR_TIMEOUT; break; }
    case mrsErrorOverflow: { modbusQueryState = mqsWrongAnswer; rc = DEVICE_ERROR_WRONG_ANSWER; break; }
    case mrsProcessing:
    default: { break; }
  }

  if (mqsSuccess == modbusQueryState) { modbusRtuProcessingFC(_functionCode, _buffer, _registersNum); rc = RESULT_IS_OK; }


//finish:

  swSerial.~SoftwareSerial();
  return rc;
}

