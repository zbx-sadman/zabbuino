#pragma once
/*

   network.h : header file which make virtual network interface from various physical interface drivers

*/

#include "net_platforms.h"
#include "sys_structs.h"
#include <SPI.h> 

#define NET_OK                      (+0x01)
#define NET_FAIL                    (+0x00)
#define NET_DHCP_PROBLEM            (-0x01)
#define NET_PHY_PROBLEM             (-0x02)

#define NET_ZERO_IP_ADDRESS         (0x00UL)

#define WIZNET_WARMING_UP_TIME      (300UL)


// Include headers for an network module
#if defined(NETWORK_ETH_WIZNET)
#include "wiznet/Ethernet.h" 
//typedef EthernetClass ParentEthernetClass;
#define ParentEthernetClass EthernetClass


#elif defined(NETWORK_ETH_ENC28J60) //NETWORK_ETH_WIZNET 
#include "enc28j60/UIPEthernet.h"
//typedef UIPEthernetClass ParentEthernetClass;
#define ParentEthernetClass UIPEthernetClass

#endif // NETWORK_ETH_ENC28J60

typedef EthernetClient NetworkClient;
typedef EthernetServer NetworkServer;

class NetworkClass : public ParentEthernetClass {
  private:
    uint8_t useDhcp;
    uint8_t phyConfigured;
    uint8_t macAddress[6];
    uint32_t defaultIpAddress;
    uint32_t defaultGateway;
    uint32_t defaultNetmask;
    uint32_t defaultDns;

  public:
    NetworkClass();
    ~NetworkClass() {}
    uint8_t isPhyOk(void);
    uint8_t isPhyConfigured(void);
    int16_t maintain(void);
    void printNetworkInfo(void);
    void printPHYState(void);
//    void resetPhy(void);
    inline uint8_t isDHCPUsed() { return useDhcp; }
    void init(const uint8_t*, const uint32_t, const uint32_t, const uint32_t, const uint32_t, const uint8_t);
    uint8_t relaunch(void);
    void resetDefaults(const uint32_t, const uint32_t, const uint32_t, const uint32_t);
    void tick();

//    NetworkClient getClient(void);
};

