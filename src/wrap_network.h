#pragma once
/*

   wrap_network.h : header file which make virtual network interface from various physical interface drivers

*/

#include "net_platforms.h"
#include "sys_structs.h"
#include <SPI.h> 

#define DHCP_CHECK_NONE         (0)
#define DHCP_CHECK_RENEW_FAIL   (1)
#define DHCP_CHECK_RENEW_OK     (2)
#define DHCP_CHECK_REBIND_FAIL  (3)
#define DHCP_CHECK_REBIND_OK    (4)


#define NET_OK                      (+0x01)
#define NET_FAIL                    (+0x00)
#define NET_DHCP_PROBLEM            (-0x01)
#define NET_PHY_PROBLEM             (-0x02)

#define NET_ZERO_IP_ADDRESS         (0x00UL)

#define WIZNET_WARMING_UP_TIME      (300UL)

// Include headers for an network module
#if defined(NETWORK_ETHERNET_WIZNET)
  #include "wiznet/Ethernet.h" 
  #define ParentEthernetClass EthernetClass
  #define NetworkTransport Ethernet
  typedef EthernetClient NetworkClient;
  typedef EthernetServer NetworkServer;

#elif defined(NETWORK_ETHERNET_ENC28J60)  
  #include "enc28j60/UIPEthernet.h"
  #define ParentEthernetClass UIPEthernetClass
  #define NetworkTransport Ethernet
  typedef EthernetClient NetworkClient;
  typedef EthernetServer NetworkServer;

#elif defined(NETWORK_WIRELESS_ESP_NATIVE)
  #if defined(ARDUINO_ARCH_ESP8266) 
    #include <ESP8266WiFi.h>
  #elif defined(ARDUINO_ARCH_ESP32)
    #include <WiFi.h>
  #endif

  #define ParentEthernetClass WiFiServer
  #define NetworkTransport WiFi
  typedef WiFiClient NetworkClient;
  typedef WiFiServer NetworkServer;
#endif


//class Network : public ParentEthernetClass {
class Network {
  private:
    static uint8_t useDhcp;
     // isPhyOk returns error state if detect it
    static uint8_t phyConfigured;
    static uint8_t macAddress[6];
    static uint32_t defaultIpAddress;
    static uint32_t defaultGateway;
    static uint32_t defaultNetmask;
    static uint32_t defaultDns;

  public:
    static uint8_t isPhyOk(void);
    static uint8_t isPhyConfigured(void);
    static int16_t maintain(void);
    static void printNetworkInfo(void);
    static void printPHYState(void);
//    void resetPhy(void);
    static inline uint8_t isDHCPUsed() { return useDhcp; }
    static void init(const uint8_t*, const uint32_t, const uint32_t, const uint32_t, const uint32_t, const uint8_t);
    static uint8_t relaunch(void);
    static void resetDefaults(const uint32_t, const uint32_t, const uint32_t, const uint32_t);
    static void tick();

//    NetworkClient getClient(void);
};

/*****************************************************************************************************************************
*
*
*
*****************************************************************************************************************************/
void setWifiDefaults();
