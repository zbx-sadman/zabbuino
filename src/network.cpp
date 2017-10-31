// Config & common included files
#include "sys_includes.h"

#include "service.h"

#include "NetworkAddress.h"
#include "net_platforms.h"

#include "network.h"

void NetworkClass::init(netconfig_t* _netConfig) {
  memcpy(macAddress, _netConfig->macAddress, 6*sizeof(uint8_t));
  netDefaultIP = _netConfig->ipAddress;
  netDefaultGW = _netConfig->ipGateway;
  netDefaultNM = _netConfig->ipNetmask;

#ifdef FEATURE_NET_DHCP_ENABLE
#ifdef FEATURE_NET_DHCP_FORCE
  useDHCP = true;
#else // FEATURE_NET_DHCP_FORCE
  useDHCP = _netConfig->useDHCP;
#endif // FEATURE_NET_DHCP_FORCE
#else  // FEATURE_NET_DHCP_ENABLE
  useDHCP = false;
#endif //FEATURE_NET_DHCP_ENABLE
/*
#if defined(NETWORK_RS485)
  useDHCP = false;
#endif
*/
}


uint8_t NetworkClass::checkPHY() {
  uint8_t rc;
  DTSL( PRINTLN_PSTR("Checking PHY..."); )   
  rc = false;

// Persistent IP address does not exist when DHCP used, and testing can not executed
#ifndef FEATURE_NET_DHCP_ENABLE
  // need "restart" PHY module if it settings is losted (may be PHY power problem?)
  if (localIP() != netDefaultIP) { 
     restart(); 
     rc = true; 
  }
#endif

#if defined(NETWORK_ETH_WIZNET)
  //rc = false;
#elif defined(NETWORK_ETH_ENC28J60) //NETWORK_ETH_WIZNET 
  uint8_t stateEconRxen = Enc28J60.readReg((uint8_t) ECON1) & ECON1_RXEN;
  uint8_t stateEirRxerif = Enc28J60.readReg((uint8_t) EIR) & EIR_RXERIF;
  uint8_t stateEstatBuffer = Enc28J60.readReg((uint8_t) ESTAT) & ESTAT_BUFFER;
  if (!stateEconRxen || (stateEstatBuffer && stateEirRxerif)) {
     Enc28J60.init(macAddress);  
     //phyReinits++;
     rc = true;
  }
#elif defined(NETWORK_RS485)
  //rc = false;
#endif

  return rc;
}

void NetworkClass::showNetworkState() {
#if defined(NETWORK_RS485)
  PRINT_PSTR("Address : "); Serial.println(localIP()); 
#else
  PRINT_PSTR("MAC     : "); printArray(macAddress, sizeof(macAddress), DBG_PRINT_AS_MAC); 
  PRINT_PSTR("IP      : "); Serial.println(Ethernet.localIP()); 
  PRINT_PSTR("Subnet  : "); Serial.println(Ethernet.subnetMask()); 
  PRINT_PSTR("Gateway : "); Serial.println(Ethernet.gatewayIP()); 
#endif
#ifdef NETWORK_ETH_ENC28J60
  DTSL( PRINT_PSTR("ENC28J60: rev "); Serial.println(Enc28J60.getrev()); )
#endif
}

void NetworkClass::showPHYState() {
#ifdef NETWORK_ETH_ENC28J60
   PRINT_PSTR("ECON1.RXEN: ")  ; Serial.println(Enc28J60.readReg((uint8_t) ECON1), BIN); 
   PRINT_PSTR("EIR.RXERIF: ")  ; Serial.println(Enc28J60.readReg((uint8_t) EIR), BIN); 
   PRINT_PSTR("ESTAT.BUFFER: "); Serial.println(Enc28J60.readReg((uint8_t) ESTAT), BIN); 
#endif
}

void NetworkClass::restart() {
  uint8_t useDefaultIP;

#ifdef FEATURE_NET_DHCP_ENABLE
  start:
  // User want to use DHCP with Zabbuino?
 //Serial.println("p3");

  if (useDHCP) {
     DTSM( PRINTLN_PSTR("Obtaining address from DHCP..."); )
     // Try to ask DHCP server & turn off DHCP feature for that session if no offer recieved
     if (0 == begin(macAddress)) {
        DTSM( PRINTLN_PSTR("No success"); )
#ifdef FEATURE_NET_DHCP_FORCE
        // infinitive loop here on "DHCP force" setting
#ifdef ON_ALARM_STATE_BLINK
      digitalWrite(constStateLedPin, millis() % 1000 < constBlinkDhcpProblem);
#else
      digitalWrite(constStateLedPin, HIGH);
#endif // ON_ALARM_STATE_BLINK

        goto start;
#endif  // FEATURE_NET_DHCP_FORCE
        useDefaultIP = true;
     } else {
        // IP address cannot be obtained, switch to default IP
        useDefaultIP = false;
    }
  }
#else // FEATURE_NET_DHCP_ENABLE
  // No DHCP required
  useDefaultIP = true;
#endif // FEATURE_NET_DHCP_ENABLE


 //Serial.println("p4");
<<<<<<< HEAD
  // No DHCP offer recieved or no DHCP need - start with stored/netDefault IP config
  if (useDefaultIP) {
     DTSM( PRINTLN_PSTR("Use default IP"); )
     useDHCP = false;
=======
  // No DHCP offer recieved or no DHCP need - start with stored/default IP config
  if (useStaticIP) {
     DTSM( PRINTLN_PSTR("Use static IP"); )
>>>>>>> origin/experimental

#if defined(NETWORK_ETH_ENC28J60) || defined(NETWORK_ETH_WIZNET)
     // That overloaded .begin() function return nothing
     // Second netConfig.ipAddress used as dns-address
     begin(macAddress, netDefaultIP, netDefaultIP, netDefaultGW, netDefaultNM);
     server.begin();

#elif defined(NETWORK_RS485)
     begin(RS485_SPEED, RS485_RX_PIN, RS485_TX_PIN, RS485_BUSMODE_PIN);
     server.begin(netDefaultIP);
#endif
  }

  // Start listen sockets by ethernet modules only
#if !defined(NETWORK_RS485)
//  server.begin();
#endif

}
