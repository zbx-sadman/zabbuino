#include "RS485Server.h"
#include "RS485.h"

RS485ServerClass::RS485ServerClass(RS485Class& _RS485) {
    *RS485 = _RS485;    
}

RS485ServerClass::RS485ServerClass(RS485Class* _RS485) {
    RS485 = _RS485;    
}

RS485ServerClass::~RS485ServerClass() {
}

/*****************************************************************************************************************************
*
*  Start "listen" UART
*
*  Returns: 
*     - none
* 
*  Params:
*     _address - "IP-type" address to which must be addressed packets, in long int (0xC0A80101, etc)
* 
*****************************************************************************************************************************/
void RS485ServerClass::begin(uint32_t _address) {
  // Set value of RS485ClientClass::localAddress that placed in RS485Class object;
  RS485->localAddress = _address;
  // Call RS485ClientClass::init() sub
  //RS485->client.init();
  
}

/*****************************************************************************************************************************
*
*  Start "listen" UART
*
*  Returns: 
*     - none
* 
*  Params:
*     _octet0 .. _octet3 - "IP-type" address to which must be addressed packets, in four bts (0xC0, 0xA8, 0x01, 0x01)
* 
*****************************************************************************************************************************/
void RS485ServerClass::begin(uint8_t _octet0, uint8_t _octet1, uint8_t _octet2, uint8_t _octet3) {
  RS485->localAddress[0] = _octet0;
  RS485->localAddress[1] = _octet1;
  RS485->localAddress[2] = _octet2;
  RS485->localAddress[3] = _octet3;
  //RS485->client.init();
}


RS485ClientClass RS485ServerClass::available() {
  // Return RS485Class::cient object only if it have ready to read data in buffer (RS485ClientClass.available() > 0)
  //Serial.print("client available="); 
  //if (RS485->client.connected()) {
  //   Serial.println("V");
     return RS485->client;
  //} else {
  //   Serial.println("X");
  //   return NULL;
 // }
}


