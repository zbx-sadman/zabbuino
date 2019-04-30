#include "../net_platforms.h"
#ifdef NETWORK_ETH_WIZNET

#pragma once

#include "Server.h"

//#include <string.h>

#include "Ethernet.h"
#include "EthernetClient.h"
#include "w5100.h"
#include "socket.h"

class EthernetClient;

class EthernetServer : 
public Server {
private:
  uint16_t _port;
public:

  EthernetServer(uint16_t);
  EthernetClient available();
  EthernetClient accept();
  EthernetClient getWorkSocket(uint8_t _withDataAvailable/*=false*/);
  virtual void begin();
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buf, size_t size);
  using Print::write;
};

#endif // NETWORK_ETH_WIZNET
