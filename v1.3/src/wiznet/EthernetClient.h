#include "../net_platforms.h"
#ifdef NETWORK_ETH_WIZNET

#ifndef _WIZNET_ETHERNETCLIENT_H_
#define _WIZNET_ETHERNETCLIENT_H_

//#include <Arduino.h>
#include <Print.h>
#include <Client.h>
#include <IPAddress.h>
//#include <string.h>

#include "w5100.h"
#include "socket.h"
#include "Ethernet.h"
#include "EthernetServer.h"
#include "Dns.h"

class EthernetClient : public Client {

public:
  EthernetClient();
  EthernetClient(uint8_t sock);

  uint8_t status();
  virtual int connect(IPAddress ip, uint16_t port);
  virtual int connect(const char *host, uint16_t port);
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buf, size_t size);
  virtual int available();
  virtual int read();
  virtual int read(uint8_t *buf, size_t size);
  virtual int peek();
  virtual void flush();
  virtual void stop();

  // added to stock lib by sadman
  IPAddress getRemoteIP();
  uint16_t getLocalPort();
  uint16_t getRemotePort();
  //

  virtual uint8_t connected();
  virtual operator bool();
  virtual bool operator==(const EthernetClient&);
  virtual bool operator!=(const EthernetClient& rhs) { return !this->operator==(rhs); };

  friend class EthernetServer;
  
  using Print::write;

private:
  static uint16_t _srcport;
  uint8_t _sock;
};

#endif // _WIZNET_ETHERNETCLIENT_H_
#endif // NETWORK_ETH_WIZNET
