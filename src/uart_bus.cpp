#include "uart_bus.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                      COMMON UART SECTION

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

/*****************************************************************************************************************************
*
*   Flush the SoftSerial's buffer
*
*   Returns: 
*     - always true
*
*****************************************************************************************************************************/
uint8_t serialRXFlush(SoftwareSerial *_swSerial, const uint8_t _slowMode)
{
  while (true) {
    // Seems that APC UPS's slow a bit and need to wait some time before check recieve buffer. 
    // Otherwise - after (0 == swSerial->available()) a few bytes can be found on input
    if (_slowMode) { delay(10); }
    if (0 == _swSerial->available()) { break; }
    _swSerial->read();
  }
  return true;
}

/*****************************************************************************************************************************
*
*   Read data from the SoftSerial's buffer
*
*   Returns: 
*     - The number of the readed bytes
*
*****************************************************************************************************************************/
uint8_t serialRecive(SoftwareSerial *_swSerial, uint8_t *_src, const uint8_t _size, const uint32_t _readTimeout, const uint8_t _stopOn, const uint8_t _slowMode)
{
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
          _src[len] = c;
          // Stop and jump out from subroutine if some byte is reached
          if (_stopOn == _src[len]) { return len+1; }
          len++;
       }
    }
  }
  return len;
}

/*****************************************************************************************************************************
*
*   Write data to the SoftSerial's buffer
*
*   Returns: 
*     - always true
*
*****************************************************************************************************************************/
void serialSend(SoftwareSerial *_swSerial, const uint8_t *_src, const uint8_t _size, const uint8_t _slowMode)
{
  uint8_t i; 
  if (_swSerial) {
     // Send data
     for (i = 0; i < _size; i++) {
       // do not rush when work with APC UPS's
       if (_slowMode) { delay(10); }
//       Serial.print("Byte# "); Serial.print(i); Serial.print(" => "); Serial.print(_src[i], HEX);  Serial.print(" '"); Serial.print((char) _src[i]); Serial.println("' ");
       // HardwareSerial::write() always return 1
       _swSerial->write(_src[i]);
    }
  }
}


