#include "sys_includes.h"
#include "winsen.h"

uint8_t winsenCalcCrc(uint8_t* _src, const uint8_t _len) {
  uint8_t crc = 0x00;

  for (uint8_t i = 0x00; i < _len; i++) {
    crc += _src[i];
    //Serial.print(" 0x"); Serial.print(ptrRawBuffer[i], HEX);
  }
  crc = (~crc) + 0x01;
  return crc;
}


uint8_t winsenUartRecieve(Stream& _stream, const uint32_t _timeout, uint8_t* _dst, const uint8_t _maxLength) {
  uint8_t headerDetected = false,
          recievedBytes = 0x00;

  uint32_t readStartTime = millis();

  while ((_maxLength > recievedBytes) && (_timeout > millis() - readStartTime)) {
    if (!_stream.available()) {
      continue;
    }
    _dst[recievedBytes] = _stream.read();
//    Serial.print("winsen: 0x"); Serial.println(_dst[recievedBytes], HEX);

    if (!headerDetected) {
      headerDetected = (WINSEN_UART_START_BYTE == _dst[0x00]);
    } else {
      recievedBytes++;
    }
  }
  return recievedBytes;
}
