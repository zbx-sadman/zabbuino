#pragma once
/*

   network.h : header file which make virtual network interface from various physical interface drivers

*/

#include "net_platforms.h"
#include "sys_structs.h"

// Include headers for an network module
#if defined(NETWORK_ETH_WIZNET)
#include <SPI.h> 
#include "wiznet/Ethernet.h" 
#include "wiznet/EthernetServer.h" 
#include "wiznet/EthernetClient.h" 
class NetworkClass
{
  private:
    uint8_t useDHCP;
    uint8_t macAddress[6];
    NetworkAddress netDefaultIP;
    NetworkAddress netDefaultGW;
    NetworkAddress netDefaultNM;

  public:
    EthernetServer server{ZBX_AGENT_TCP_PORT};  // NOTE: brace need when param is used
    EthernetClient client;
    NetworkClass() {}
    ~NetworkClass() {}
    inline IPAddress localIP() { return Ethernet.localIP(); }
    inline IPAddress defaultIP() { return (IPAddress) netDefaultIP; }

    uint8_t checkPHY();
    inline void checkClient() { client = server.available(); }
    inline void stopClient() { client.stop(); }
    inline uint8_t isDHCPUsed() { return useDHCP; }
    inline uint8_t getRCR() { return 0; }
    inline uint16_t getRTR() { return 0; }
    inline uint16_t getPHYCFG() { return 0; }
    inline int maintain() {return Ethernet.maintain(); };
    int begin(uint8_t *_mac) { return Ethernet.begin(_mac); }
    void begin(uint8_t *_mac, NetworkAddress _ip, NetworkAddress _dns, NetworkAddress _gw, NetworkAddress _netmask) {
         Ethernet.begin(_mac, IPAddress(_ip), IPAddress(_dns), IPAddress(_gw), IPAddress(_netmask));
    }
    void showNetworkState();
    void showPHYState();
    void restart();

    void init(netconfig_t*);
};

#elif defined(NETWORK_ETH_ENC28J60) //NETWORK_ETH_WIZNET 
#include "enc28j60/UIPEthernet.h"
#include "enc28j60/enc28j60.h"
class NetworkClass
{
  private:
    uint8_t useDHCP;
    uint8_t macAddress[6];
    NetworkAddress netDefaultIP;
    NetworkAddress netDefaultGW;
    NetworkAddress netDefaultNM;

  public:
    // uint16_t phyReinits;

    UIPServer server{ZBX_AGENT_TCP_PORT};
    UIPClient client;
    NetworkClass() {}
    ~NetworkClass() {}
    uint8_t checkPHY();
    inline void checkClient() { client = server.available(); }
    inline void stopClient() { client.stop(); }
    inline IPAddress localIP() { return UIPEthernet.localIP(); }
    inline IPAddress defaultIP() { return (IPAddress) netDefaultIP; }
    inline void tick() { UIPEthernet.tick(); }
    inline uint8_t getRCR() { return 0; }
    inline uint16_t getRTR() { return 0; }
    inline uint16_t getPHYCFG() { return 0; }
    inline int maintain() {return UIPEthernet.maintain(); };
    inline int begin(uint8_t *_mac) { return UIPEthernet.begin(_mac); }
    inline void begin(uint8_t *_mac, NetworkAddress _ip, NetworkAddress _dns, NetworkAddress _gw, NetworkAddress _netmask) {
         UIPEthernet.begin(_mac, IPAddress(_ip), IPAddress(_dns), IPAddress(_gw), IPAddress(_netmask));
    }
    void showNetworkState();
    void showPHYState();
    void restart();
    void init(netconfig_t*);
};

#endif // NETWORK_ETH_ENC28J60
