// Config & common included files
#include "sys_includes.h"

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
/*uint8_t serialRXFlush(Stream *_stream, const uint8_t _slowMode)
{
  while (0 < _stream->available()) {
    // Seems that APC UPS's slow a bit and need to wait some time before check recieve buffer. 
    // Otherwise - after (0 == swSerial->available()) a few bytes can be found on input
    if (_slowMode) { delay(10); }
    _stream->read();
  }
  return true;
}
*/
/*****************************************************************************************************************************
*
*   Read data from the SoftSerial's buffer
*
*   Returns: 
*     - The number of the readed bytes
*
*****************************************************************************************************************************/
uint8_t serialRecive(Stream* _stream, uint8_t* _dst, const uint8_t _size, const uint32_t _readTimeout, const uint8_t _stopOnChar, const uint8_t _stopChar, const uint8_t _slowMode)
{
  uint32_t startTime = millis();
  uint8_t len = 0x00;
  //  while ((len <  _size) && (millis() - startTime < _readTimeout)) {
  while (millis() - startTime < _readTimeout) {
    if (len >=  _size) { break; }
    if (_stream) {
       // Slow talk with APC UPC'es
       if (_slowMode) { delay(10); }
       if (0x00 < _stream->available()) {
          uint8_t c = (uint8_t) _stream->read();
          if (!c && !len) {
             continue; // skip 0 at startup
          }
          __DMLD( DEBUG_PORT.print("rcv: "); DEBUG_PORT.println(c, HEX); )
          _dst[len] = c;
          // Stop and jump out from subroutine if some byte is reached
          if (_stopOnChar && (_stopChar == _dst[len])) { return len + 0x01; }
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
void serialSend(Stream* _stream, uint8_t* _src, uint8_t _size, const uint8_t _slowMode)
{
  __DMLD( uint8_t i = 0x00; )

  if (_stream) {
     while (_size) {
       // do not rush when work with APC UPS's
       if (_slowMode) { delay(10); }

       __DMLD( DEBUG_PORT.print("send byte# "); 
               DEBUG_PORT.print(i); 
               DEBUG_PORT.print(" => "); 
               DEBUG_PORT.print(*_src, HEX);  
               DEBUG_PORT.print(" '"); 
               DEBUG_PORT.print((char) *_src); DEBUG_PORT.println("' "); 
               ++i;
       )
       // HardwareSerial::write() always return 1
       _stream->write(*_src);
       _src++;
       _size--;
    }
  }
}


