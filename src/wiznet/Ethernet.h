#include "../net_platforms.h"
#ifdef NETWORK_ETHERNET_WIZNET

#pragma once

/*
 modified 12 Aug 2013
 by Soohwan Kim (suhwan@wiznet.co.kr)
*/

//#include <inttypes.h>
//#include <IPAddress.h>


#include "w5100.h"

#include "EthernetClient.h"
#include "EthernetServer.h"
#include "Dhcp.h"



class EthernetClass {
private:
  IPAddress _dnsServerAddress;
  DhcpClass* _dhcp;
public:
  static uint8_t _state[WIZNET_SOCKETS_USED];
  static uint16_t _server_port[WIZNET_SOCKETS_USED];

  // Initialize the Ethernet shield to use the provided MAC address and gain the rest of the
  // configuration through DHCP.
  // Returns 0 if the DHCP configuration failed, and 1 if it succeeded
  int begin(uint8_t *mac_address);
  void begin(uint8_t *mac_address, IPAddress local_ip);
  void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server);
  void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway);
  void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);
  
  int maintain();

  IPAddress localIP();
  IPAddress subnetMask();
  IPAddress gatewayIP();
  IPAddress dnsServerIP();
  //
  uint16_t getRTR();
  uint8_t getRCR();
  uint8_t getPHYCFG();
  inline uint8_t getMaxSocketNumber() const { return W5100.getMaxSocketNumber(); }
  inline uint8_t hardwareStatus() { return W5100.hardwareStatus(); }
  //

  friend class EthernetClient;
  friend class EthernetServer;
};

extern EthernetClass Ethernet;

#endif // NETWORK_ETHERNET_WIZNET
