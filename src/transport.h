/*

   transport.h : header file which make virtual transport interface from various physical interface drivers

*/
#ifndef _ZABBUINO_TRANSPORT_H_
#define _ZABBUINO_TRANSPORT_H_

#include "../basic.h"
#include "transport_hlp.h"
//#include "tune.h"

// Include headers for an network module
#if defined(TRANSPORT_ETH_WIZNET)
#include "wiznet\Ethernet.h" 
#include "wiznet\EthernetServer.h" 
#include "wiznet\EthernetClient.h" 
#include <SPI.h> 
#define ETHERNET_CLASS EthernetClass
class TransportClass : public ETHERNET_CLASS
{
  private:
    uint8_t macAddress[6];

  public:
    // uint16_t phyReinits;

    EthernetServer server{ZBX_AGENT_TCP_PORT};  // NOTE: brace need when param is used
    EthernetClient client;
    TransportClass() {}
    ~TransportClass() {}
    uint8_t netModuleCheck();
    int begin(uint8_t *mac_address);
    void begin(uint8_t *mac_address, IPAddress local_ip);
    void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server);
    void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway);
    void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);
};
#elif defined(TRANSPORT_ETH_ENC28J60) //TRANSPORT_ETH_WIZNET 
#include "enc28j60\UIPEthernet.h"
#include "enc28j60\enc28j60.h"
#define ETHERNET_CLASS UIPEthernetClass 
class TransportClass : public ETHERNET_CLASS 
{
  private:
    uint8_t macAddress[6];

  public:
    // uint16_t phyReinits;

    UIPServer server{ZBX_AGENT_TCP_PORT};
    UIPClient client;
    TransportClass() {}
    ~TransportClass() {}
    inline uint16_t getRTR() { return 0; }
    inline uint8_t getRCR() { return 0; }
    inline uint16_t getPHYCFG() { return 0; }
    uint8_t netModuleCheck();
    int begin(uint8_t *mac_address);
    void begin(uint8_t *mac_address, IPAddress local_ip);
    void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server);
    void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway);
    void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);
};
#endif // TRANSPORT_ETH_ENC28J60
#endif // _ZABBUINO_TRANSPORT_H_

