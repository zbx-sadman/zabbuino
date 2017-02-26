#ifndef RS485_H
#define RS485_H
#include <Arduino.h>
#include <IPAddress.h>
#include <Print.h>
#include <SoftwareSerial.h>
#include <util/crc16.h>

#include "../NetworkAddress.h"
#include "../service.h"
#include "RS485Structs.h"
#include "RS485Client.h"
#include "RS485Server.h"
#include "RS485-conf.h"

class RS485Class {
  private:

    uint8_t buffer[RS485_BUFFER_SIZE];
    uint8_t buffReadPos, buffWritePos;
    uint8_t dataAvailable;
    uint8_t pktType;
    uint8_t pktRecieved;
    int16_t pktExpectedType;
    uint32_t pktExpectedAddress;
    uint32_t waitAfterSendmS;
    uint16_t commTimeout;
    uint8_t busModePin;

    NetworkAddress remoteAddress;
    NetworkAddress localAddress;
    packet_header_t* packetHeader;

    void init();
    void tick();
    void send(uint8_t* _src, uint16_t _len);
    void switchBusToRX();
    void switchBusToTX();
    
    uint8_t assemblePacket(const uint8_t);
    uint8_t connect(const NetworkAddress);
    uint16_t crc16(const uint8_t*, uint16_t);
    uint16_t createPacket(const NetworkAddress, const uint8_t);
    uint16_t createPacket(const NetworkAddress, const uint8_t, const uint8_t*);
    int32_t waitForPacket(const NetworkAddress, const uint8_t);

  public:
    SoftwareSerial* UART;
    RS485ClientClass client{this};
    RS485ServerClass server{this};

    RS485Class();
    ~RS485Class();
    uint8_t begin(uint16_t, uint8_t, uint8_t, uint8_t);
    uint8_t begin(uint16_t, uint8_t, uint8_t, uint8_t, uint8_t);
    
/*    
    void begin(uint32_t);
    void begin(uint8_t, uint8_t, uint8_t, uint8_t);   
    void stop(){ remoteAddress = RS485_INADDR_NONE; }
    int16_t available();
    int16_t read();

*/
    int16_t maintain();
    void setTimeout(int32_t _timeout) { commTimeout = _timeout; } ;
    int32_t ping(const NetworkAddress _dst);    

//    int32_t println(const char* _src);

    IPAddress localIP();
    IPAddress subnetMask();
    IPAddress gatewayIP();
    IPAddress dnsServerIP();
    friend RS485ClientClass;
    friend RS485ServerClass;

    
};

extern RS485Class RS485;

#endif // RS485_H
