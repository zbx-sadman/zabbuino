#include "../net_platforms.h"
#ifdef NETWORK_ETHERNET_WIZNET

#include "EthernetServer.h"

EthernetServer::EthernetServer(uint16_t port)
{
  _port = port;
}

void EthernetServer::begin() {
//  static uint8_t startSocket = 0x00;

  for (uint8_t currentSocket = 0x00; currentSocket < WIZNET_SOCKETS_USED; currentSocket++) { 
    uint8_t socketStatus = W5100.readSnSR(currentSocket);
    if (socketStatus == SnSR::CLOSED) {
      socket(currentSocket, SnMR::TCP, _port, 0);
      listen(currentSocket);
      EthernetClass::_server_port[currentSocket] = _port;
      break;
    }
  }  

/*
  for (int sock = 0; sock < WIZNET_SOCKETS_USED; sock++) {
    EthernetClient client(sock);
    if (client.status() == SnSR::CLOSED) {
      socket(sock, SnMR::TCP, _port, 0);
      listen(sock);
      EthernetClass::_server_port[sock] = _port;
      break;
    }
  }  
*/
}

/*
EthernetClient EthernetServer::getWorkSocket(uint8_t _withDataAvailable = false) {
  static uint8_t socketNo = 0x00;
  uint8_t listening = false;
  uint8_t workSocketNo = WIZNET_SOCKETS_USED;

  for (uint8_t i = 0x00; i < WIZNET_SOCKETS_USED; i++) {
    EthernetClient ethClient(socketNo);

    if (EthernetClass::_server_port[socketNo] == _port) {
       uint8_t ethClientStatus = ethClient.status();

       if (SnSR::LISTEN == ethClientStatus) {
          listening = true;

       } else if (SnSR::CLOSED == ethClientStatus) {
          EthernetClass::_server_port[socketNo] = 0x00; // ???

       } else if (SnSR::CLOSE_WAIT == ethClientStatus) {
         // ::available()
         if (_withDataAvailable) {
            if (ethClient.available()) {           
               workSocketNo = socketNo; 
            } else {         
              ethClient.stop();
            }
         // ::accept()
         } else {
            workSocketNo = socketNo; 
            EthernetClass::_server_port[socketNo] = 0x00; //only return the client once

         }
       // ... else if (SnSR::CLOSE_WAIT == ethClientStatus)
       } else if (SnSR::ESTABLISHED == ethClientStatus) {
         // ::available()
         if (_withDataAvailable) {
            if (ethClient.available()) {           
               workSocketNo = socketNo; 
            }
         // ::accept()
         } else {
            workSocketNo = socketNo; 
            EthernetClass::_server_port[socketNo] = 0x00; //only return the client once
         }
       } // ... else if (SnSR::ESTABLISHED == ethClientStatus)
    }

    socketNo++; if (socketNo == WIZNET_SOCKETS_USED) { socketNo = 0x00; }
  } // for

  if (!listening) { begin(); }
  return EthernetClient(workSocketNo);
}
*/
EthernetClient EthernetServer::getWorkSocket(uint8_t _withDataAvailable = false) {
  uint8_t listening = false;
  uint8_t workSocketNo = WIZNET_SOCKETS_USED;

  for (uint8_t socketNo = 0x00; socketNo < WIZNET_SOCKETS_USED; socketNo++) {

    EthernetClient ethClient(socketNo);

    if (EthernetClass::_server_port[socketNo] == _port) {
       uint8_t ethClientStatus = ethClient.status();
       uint16_t ethClientAvailable = ethClient.available();
         
       if (SnSR::LISTEN == ethClientStatus) {
          listening = true;

       } else if (SnSR::CLOSED == ethClientStatus) {
          EthernetClass::_server_port[socketNo] = 0x00; // ???

       } else if (SnSR::CLOSE_WAIT == ethClientStatus) {
         // ::available()
         if (_withDataAvailable) {
            if (ethClientAvailable) {           
               workSocketNo = socketNo; 
            } else {         
              ethClient.stop();
            }
         // ::accept()
         } else {
            workSocketNo = socketNo; 
            EthernetClass::_server_port[socketNo] = 0x00; //only return the client once

         }
       // ... else if (SnSR::CLOSE_WAIT == ethClientStatus)
       } else if (SnSR::ESTABLISHED == ethClientStatus) {
         // ::available()
         if (_withDataAvailable) {
            if (ethClientAvailable) {           
               workSocketNo = socketNo; 
            }
         // ::accept()
         } else {
            workSocketNo = socketNo; 
            EthernetClass::_server_port[socketNo] = 0x00; //only return the client once
         }
       } // ... else if (SnSR::ESTABLISHED == ethClientStatus)
    }
 
  } // for

  if (!listening) { begin(); }
  return EthernetClient(workSocketNo);
}

EthernetClient EthernetServer::available(){
  return getWorkSocket(true);

}

EthernetClient EthernetServer::accept(){
  return getWorkSocket(false);
}

/*
void EthernetServer::accept()
{
  int listening = 0;

  for (int sock = 0; sock < WIZNET_SOCKETS_USED; sock++) {
    EthernetClient client(sock);

    if (EthernetClass::_server_port[sock] == _port) {
      if (client.status() == SnSR::LISTEN) {
        listening = 1;
      } 
      else if (client.status() == SnSR::CLOSE_WAIT && !client.available()) {
        client.stop();
      }
    } 
  }

  if (!listening) { begin(); }
}


EthernetClient EthernetServer::available()
{
  accept();

  for (int sock = 0; sock < WIZNET_SOCKETS_USED; sock++) {
    EthernetClient client(sock);
    if (EthernetClass::_server_port[sock] == _port &&
        (client.status() == SnSR::ESTABLISHED ||
         client.status() == SnSR::CLOSE_WAIT)) {
      if (client.available()) {
        // XXX: don't always pick the lowest numbered socket.
        return client;
      }
    }
  }

  return EthernetClient(WIZNET_SOCKETS_USED);
}

*/
size_t EthernetServer::write(uint8_t b) 
{
  return write(&b, 1);
}

size_t EthernetServer::write(const uint8_t *buffer, size_t size) 
{
  size_t n = 0;
  
  accept();

  for (int sock = 0; sock < WIZNET_SOCKETS_USED; sock++) {
    EthernetClient client(sock);

    if (EthernetClass::_server_port[sock] == _port &&
      client.status() == SnSR::ESTABLISHED) {
      n += client.write(buffer, size);
    }
  }
  
  return n;
}

#endif // NETWORK_ETHERNET_WIZNET
