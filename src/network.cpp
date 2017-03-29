#include "network.h"

void NetworkClass::init(netconfig_t* _netConfig) {
  memcpy(macAddress, _netConfig->macAddress, 6*sizeof(uint8_t));
  localAddress = _netConfig->ipAddress;
  gwAddress = _netConfig->ipGateway;
  netmask = _netConfig->ipNetmask;
  useDHCP = _netConfig->useDHCP;
#ifdef FEATURE_NET_DHCP_FORCE
  useDHCP = true;
#endif

#if defined(NETWORK_RS485)
  useDHCP = false;
#endif
}


uint8_t NetworkClass::checkPHY() {
  uint8_t rc;
  DTSL( SerialPrintln_P(PSTR("Checking PHY...")); )   
  rc = false;

// Persistent IP address does not exist when DHCP used, and testing can not executed
#ifndef FEATURE_NET_DHCP_ENABLE
  // need "restart" PHY module if it settings is losted (may be PHY power problem?)
  if (localIP() != localAddress) { 
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
  SerialPrint_P(PSTR("Address : ")); Serial.println(localIP()); 
#else
  SerialPrint_P(PSTR("MAC     : ")); printArray(macAddress, sizeof(macAddress), DBG_PRINT_AS_MAC); 
  SerialPrint_P(PSTR("IP      : ")); Serial.println(Ethernet.localIP()); 
  SerialPrint_P(PSTR("Subnet  : ")); Serial.println(Ethernet.subnetMask()); 
  SerialPrint_P(PSTR("Gateway : ")); Serial.println(Ethernet.gatewayIP()); 
#endif
#ifdef NETWORK_ETH_ENC28J60
  DTSL( SerialPrint_P(PSTR("ENC28J60: rev ")); Serial.println(Enc28J60.getrev()); )
#endif
}

void NetworkClass::showPHYState() {
#ifdef NETWORK_ETH_ENC28J60
   SerialPrint_P(PSTR("ECON1.RXEN: ")); Serial.println(Enc28J60.readReg((uint8_t) ECON1), BIN); 
   SerialPrint_P(PSTR("EIR.RXERIF: ")); Serial.println(Enc28J60.readReg((uint8_t) EIR), BIN); 
   SerialPrint_P(PSTR("ESTAT.BUFFER: ")); Serial.println(Enc28J60.readReg((uint8_t) ESTAT), BIN); 
#endif
}

void NetworkClass::restart() {
  uint8_t useStaticIP;

#ifdef FEATURE_NET_DHCP_ENABLE
  start:
  // User want to use DHCP with Zabbuino?
 //Serial.println("p3");

  if (useDHCP) {
     DTSM( SerialPrintln_P(PSTR("Obtaining address from DHCP...")); )
     // Try to ask DHCP server & turn off DHCP feature for that session if no offer recieved
     if (0 == begin(macAddress)) {
        DTSM( SerialPrintln_P(PSTR("No success")); )
#ifdef FEATURE_NET_DHCP_FORCE
        // infinitive loop here on "DHCP force" setting
#ifdef ON_ALARM_STATE_BLINK
      digitalWrite(constStateLedPin, millis() % 1000 < constBlinkDhcpProblem);
#else
      digitalWrite(constStateLedPin, HIGH);
#endif

        goto start;
#endif  // FEATURE_NET_DHCP_FORCE
        useStaticIP = true;
     } else {
        useStaticIP = false;
    }
  }
#else // FEATURE_NET_DHCP_ENABLE
  // No DHCP required
  useStaticIP = true;
#endif // FEATURE_NET_DHCP_ENABLE


 //Serial.println("p4");
  // No DHCP offer recieved or no DHCP need - start with stored/default IP config
  if (useStaticIP) {
     DTSM( SerialPrintln_P(PSTR("Use static IP")); )

#if defined(NETWORK_ETH_ENC28J60) || defined(NETWORK_ETH_WIZNET)
     // That overloaded .begin() function return nothing
     // Second netConfig.ipAddress used as dns-address
     begin(macAddress, localAddress, localAddress, gwAddress, netmask);
     server.begin();

#elif defined(NETWORK_RS485)
     begin(RS485_SPEED, RS485_RX_PIN, RS485_TX_PIN, RS485_BUSMODE_PIN);
     server.begin(localAddress);
#endif
  }

  // Start listen sockets by ethernet modules only
#if !defined(NETWORK_RS485)
//  server.begin();
#endif

}
