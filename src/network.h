/*

   network.h : header file which make virtual network interface from various physical interface drivers

*/
#ifndef _ZABBUINO_NETWORK_H_
#define _ZABBUINO_NETWORK_H_

#include "../basic.h"
#include "NetworkAddress.h"
#include "network_hlp.h"
//#include "tune.h"
#include "service.h"

// Include headers for an network module
#if defined(NETWORK_ETH_WIZNET)
#include "wiznet/Ethernet.h" 
#include "wiznet/EthernetServer.h" 
#include "wiznet/EthernetClient.h" 
#include <SPI.h> 
class NetworkClass: public EthernetClass
{
  private:
    uint8_t useDHCP;
    uint8_t macAddress[6];
    NetworkAddress localAddress;
    NetworkAddress gwAddress;
    NetworkAddress netmask;

  public:
    EthernetServer server{ZBX_AGENT_TCP_PORT};  // NOTE: brace need when param is used
    EthernetClient client;
    NetworkClass() {}
    ~NetworkClass() {}
    uint8_t checkPHY();
    void showNetworkState();
    void showPHYState();
    void restart();
    void init(netconfig_t*);
    int begin(uint8_t*);
    void begin(uint8_t*, NetworkAddress, NetworkAddress, NetworkAddress, NetworkAddress);
};

#elif defined(NETWORK_ETH_ENC28J60) //NETWORK_ETH_WIZNET 
#include "enc28j60/UIPEthernet.h"
#include "enc28j60/enc28j60.h"
class NetworkClass: public UIPEthernetClass 
{
  private:
    uint8_t useDHCP;
    uint8_t macAddress[6];
    NetworkAddress localAddress;
    NetworkAddress gwAddress;
    NetworkAddress netmask;

  public:
    // uint16_t phyReinits;

    UIPServer server{ZBX_AGENT_TCP_PORT};
    UIPClient client;
    NetworkClass() {}
    ~NetworkClass() {}
    inline uint16_t getRTR() { return 0; }
    inline uint8_t getRCR() { return 0; }
    inline uint16_t getPHYCFG() { return 0; }
    uint8_t checkPHY();
    void showNetworkState();
    void showPHYState();
    void restart();
    void init(netconfig_t*);
    int begin(uint8_t*);
    void begin(uint8_t*, NetworkAddress, NetworkAddress, NetworkAddress, NetworkAddress);
};

#elif defined(NETWORK_RS485) //NETWORK_ETH_ENC28J60
#include "rs485/RS485.h"
#include "rs485/RS485Server.h"
#include "rs485/RS485Client.h"
class NetworkClass: public RS485Class
{
  private:
    uint8_t macAddress[6];
    NetworkAddress localAddress;
    NetworkAddress gwAddress;
    NetworkAddress netmask;

  public:
//    RS485ClientClass client{this};
  //  RS485ServerClass server{this};

    // uint16_t phyReinits;
    NetworkClass() {}
    ~NetworkClass() {}
    inline uint16_t getRTR() { return 0; }
    inline uint8_t getRCR() { return 0; }
    inline uint16_t getPHYCFG() { return 0; }
    uint8_t checkPHY();
    void showNetworkState();
    void showPHYState();
    void restart();
    void init(netconfig_t*);
    uint8_t begin(uint16_t, uint8_t, uint8_t, uint8_t);
    uint8_t begin(uint16_t, uint8_t, uint8_t, uint8_t, uint8_t);
};

#endif // NETWORK_RS485
#endif // _ZABBUINO_NETWORK_H_

