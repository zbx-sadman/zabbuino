#include "RS485Client.h"

// do not move #include "RS485.h" to RS485Client.h to avoid "incomplete type" error
#include "RS485.h"
RS485ClientClass::RS485ClientClass(RS485Class& _RS485) {
    *RS485 = _RS485;    
}

RS485ClientClass::RS485ClientClass(RS485Class* _RS485) {
    RS485 = _RS485;    
}


RS485ClientClass::~RS485ClientClass() {
}



/*****************************************************************************************************************************
*
*  Imitate EthernetClient.available() subroutine, return the number of bytes available for reading from internal buffer
*  (that is, the amount of data that has been written to the UART by the remote host if recieved packet is valid).
*
*  Returns: 
*     - The number of bytes available, 0 if buffer is empty, or -1 if no data ready
* 
*  Params:
*     _dst - network address of host to which must be maded connection
* 
*****************************************************************************************************************************/
int16_t RS485ClientClass::available() {
  // return -1 if data is not ready to processing (packet recieving not finished)
  if (!RS485->dataAvailable) { return -1; }
  return (RS485->buffWritePos - RS485->buffReadPos);
}

/*****************************************************************************************************************************
*
*  Imitate EthernetClient.read() subroutine, return the next byte from the internal buffer (after the last call to read()).
*
*  Returns: 
*     - The next byte (or character), or -1 if none is available.
* 
*  Params:
*     _dst - network address of host to which must be maded connection
* 
*****************************************************************************************************************************/
int16_t RS485ClientClass::read() {
  // return -1 if data is not ready to processing (packet is not recieved)
  if (!RS485->dataAvailable) { return -1; }
  // All data is readed - need to wait to new incoming data packet.
  if (RS485->buffReadPos == RS485->buffWritePos) {  RS485->init(); }
  return RS485->buffer[RS485->buffReadPos++];
}

/*****************************************************************************************************************************
*
*  Imitate EthernetClient.println() subroutine, Send data, followed by a newline, as RS485 class packet over UART
*
*  Returns: 
*     - The number of bytes "printed", or -1 if sending is not sucessfully (destination is unknown or no ACK packet recieved)
* 
*  Params:
*     _src - an string to "print"
* 
*****************************************************************************************************************************/
int32_t RS485ClientClass::println(int32_t& _value, int _mode) {
  char dst[10+1];
  ltoa(_value, dst, 10);
  return (println(dst));
};


int32_t RS485ClientClass::println(const char* _src) {
  //if (RS485_INADDR_NONE == ((uint32_t) remoteAddress)) { return -1; }
  DTSM( printf("[*] Sending TEXT packet: %s\n", _src); )
  // Form packet by createPacket() and write it to UART
  NetworkAddress fromAddress = RS485->remoteAddress;
  RS485->send(RS485->buffer, RS485->createPacket(RS485->remoteAddress, RS485_PKTTYPE_DATA, (uint8_t*) _src));
  RS485->init();
  // Wait for ACK packet and return -1 if it not recieved in a RS485ClientClass::commTimeout.
  // Otherwize return back length of _src string
  int32_t rc = RS485->waitForPacket(fromAddress, RS485_PKTTYPE_ACK);
  RS485->init();
  return ((-1 == rc )? -1 : strlen(_src));
};


void RS485ClientClass::stop() {
  // "Jump" to the initial stage.
  RS485->init();
//  Serial.print("RS485->remoteAddress="); Serial.println(RS485->remoteAddress,HEX);
  
}

uint8_t RS485ClientClass::connected() {
  // Client is connected when remoteAddress is defined
  return (RS485_INADDR_NONE != RS485->remoteAddress);
}

// for "if (!RS485ClientClass) {}" operator
RS485ClientClass::operator bool() {
  // Client is valid if connected
  return connected();
}

bool const RS485ClientClass::operator==(const RS485ClientClass& rhs)  {
  // Clients is equal when both is "connected" to the same remote address (will send to the same host)
  // connected() && rhs.connected()
  return (RS485->remoteAddress == rhs.RS485->remoteAddress && RS485_INADDR_NONE == RS485->remoteAddress && RS485_INADDR_NONE == rhs.RS485->remoteAddress);
  //return (remoteAddress == rhs.remoteAddress && RS485_INADDR_NONE == remoteAddress && RS485_INADDR_NONE == rhs->remoteAddress);
}



/*****************************************************************************************************************************
*
*  "Make" connection to remote host
*
*  Returns: 
*     - Always 1
* 
*  Params:
*     _dst - network address of host to which must be maded connection
* 
*****************************************************************************************************************************/
uint8_t RS485ClientClass::connect(const NetworkAddress _dst) {
    return RS485->connect( _dst);
}

