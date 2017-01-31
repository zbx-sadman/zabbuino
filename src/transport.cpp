#include "transport.h"

int TransportClass::begin(uint8_t *mac_address) {
   //memcpy(macAddress, mac_address, 6*sizeof(uint8_t));
   return ETHERNET_CLASS::begin(mac_address);
}

void TransportClass::begin(uint8_t *mac_address, IPAddress local_ip) {
   //memcpy(macAddress, mac_address, 6*sizeof(uint8_t));
   ETHERNET_CLASS::begin(mac_address, local_ip);
}

void TransportClass::begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server) {
   //memcpy(macAddress, mac_address, 6*sizeof(uint8_t));
   ETHERNET_CLASS::begin(mac_address, local_ip, dns_server);
}

void TransportClass::begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway) {
   //memcpy(macAddress, mac_address, 6*sizeof(uint8_t));
   ETHERNET_CLASS::begin(mac_address, local_ip, dns_server, gateway);
}

void TransportClass::begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet) {
   //memcpy(macAddress, mac_address, 6*sizeof(uint8_t));
   ETHERNET_CLASS::begin(mac_address, local_ip, dns_server, gateway, subnet);
}

uint8_t TransportClass::netModuleCheck() {
#if defined(TRANSPORT_ETH_WIZNET)
   return false;
#elif defined(TRANSPORT_ETH_ENC28J60) //TRANSPORT_ETH_WIZNET 
   uint8_t stateEconRxen = Enc28J60.readReg((uint8_t) ECON1) & ECON1_RXEN;
   uint8_t stateEirRxerif = Enc28J60.readReg((uint8_t) EIR) & EIR_RXERIF;
   uint8_t stateEstatBuffer = Enc28J60.readReg((uint8_t) ESTAT) & ESTAT_BUFFER;
   if (!stateEconRxen || (stateEstatBuffer && stateEirRxerif)) {
      Enc28J60.init(macAddress); 
      //phyReinits++;
      return true;
   }
   return false;
#endif
}
