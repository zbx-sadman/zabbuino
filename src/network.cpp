// Config & common included files
#include "sys_includes.h"

#include "service.h"

#include "net_platforms.h"

#include "network.h"

uint8_t  Network::useDhcp          = false;
uint8_t  Network::phyConfigured    = false;
uint8_t  Network::macAddress[6]    = {0};
uint32_t Network::defaultIpAddress = 0x00000000UL;
uint32_t Network::defaultGateway   = 0x00000000UL;
uint32_t Network::defaultNetmask   = 0x00000000UL;
uint32_t Network::defaultDns       = 0x00000000UL;


void Network::tick() {
#if defined(NETWORK_ETHERNET_ENC28J60) 
   // tick() subroutine is very important for UIPEthernet, and must be called often (every ~250ms). 
   // If ENC28J60 driver not used - this subroutine do nothing
   Ethernet.tick();
#endif
}

uint8_t Network::isPhyConfigured() { 
#if defined(NETWORK_ETHERNET_ENC28J60) || defined(NETWORK_ETHERNET_WIZNET)
  uint32_t localIpAddress = (uint32_t)Ethernet.localIP();
  // NET_ZERO_IP_ADDRESS != defaultIpAddress -> network is not configured after start
  // localIpAddress == defaultIpAddress -> Netcard is missconfigured (may be drop configuration on power problem or something?)
  phyConfigured = (phyConfigured && (NET_ZERO_IP_ADDRESS != defaultIpAddress) && (localIpAddress == defaultIpAddress));
#elif defined(NETWORK_SERIAL_INTERFACE)
  phyConfigured = true;
#endif
  return phyConfigured;
}

int16_t Network::maintain() {
  int16_t rc = DHCP_CHECK_NONE;
#if (defined(NETWORK_ETHERNET_ENC28J60) || defined(NETWORK_ETHERNET_WIZNET)) && defined(FEATURE_NET_DHCP_ENABLE)
  if (useDhcp) {
     // maintain() subroutine adds more fat to WIZnet drivers, because includes DHCP functionality to firmware even it not need
     rc = Ethernet.maintain();
     if (rc) { resetDefaults((uint32_t)Ethernet.localIP(), (uint32_t)Ethernet.dnsServerIP(), (uint32_t)Ethernet.gatewayIP(), (uint32_t)Ethernet.subnetMask()); }
  }
#endif // FEATURE_NET_DHCP_ENABLE
  return rc;
}

uint8_t Network::isPhyOk() {
  uint8_t phyState = true;
  //__DMLH( DEBUG_PORT.print(FSH_P(STRING_Checking_PHY)); DEBUG_PORT.print(FSH_P(STRING_3xDot_Space)); )   

#if defined(NETWORK_ETHERNET_WIZNET) //NETWORK_ETHERNET_WIZNET 
  // hardwareStatus() returns 0x00 if wrong hardware or no hardware answer
//  DTSL( DEBUG_PORT.print("HW stat:"); DEBUG_PORT.println(Ethernet.hardwareStatus()); )   
  //phyState = !!Ethernet.hardwareStatus();

#elif defined(NETWORK_ETHERNET_ENC28J60) //NETWORK_ETHERNET_WIZNET 
  phyState = !!Enc28J60.getrev();
  // ECON1.RXEN   - Recieve enable bit
  // EIR.RXERIF   - Recieve error interrupt bit
  // ESTAT.BUFFER - Ethernet buffer error status bit
  // ESTAT.CLKRDY - Clock Ready bit
  // uint8_t stateEconRxen    = !!(Enc28J60.readReg((uint8_t) ECON1) & ECON1_RXEN);
  uint8_t stateEirRxerif   = !!(Enc28J60.readReg((uint8_t) EIR) & EIR_RXERIF);
  uint8_t stateEstatBuffer = !!(Enc28J60.readReg((uint8_t) ESTAT) & ESTAT_BUFFER);
  uint8_t stateEstatClkRdy = !!(Enc28J60.readReg((uint8_t) ESTAT) & ESTAT_CLKRDY);
  
  //DEBUG_PORT.print(F("ECON1.RXEN  : ")); DEBUG_PORT.println(stateEconRxen); 
  //DEBUG_PORT.print(F("ESTAT.BUFFER: ")); DEBUG_PORT.println(stateEstatBuffer); 
  //DEBUG_PORT.print(F("EIR.RXERIF  : ")); DEBUG_PORT.println(stateEirRxerif); 
  //DEBUG_PORT.print(F("ESTAT.CLKRDY: ")); DEBUG_PORT.println(stateEstatClkRdy); 
  // Error on
  // 1) Recieve bit is __not__ set  (really need to be alarmed?)
  // 2) Ethernet buffer error status bit is __not__ dropped  AND Recieve error interrupt bit is set
  // 3) Clock Ready bit is __not_ set

  // The CLKRDY does not work. See Rev. B4 Silicon Errata point page #1:
  // After issuing the Reset command, wait for at least 1 ms in firmware for the device to be ready. 
  // Implemented into UIP_Ethernet driver
  if (stateEstatBuffer && stateEirRxerif) {
     // ENC28J60 rise error flag
     phyState = false;
  }
#endif
//  __DMLH( DEBUG_PORT.println(FSH_P(phyState ? STRING_ok : STRING_fail)); )
  return phyState;
}


void Network::init(const uint8_t* _mac, const uint32_t _ip, const uint32_t _dns, const uint32_t _gw, const uint32_t _netmask, const uint8_t _useDhcp) {
  // MAC never changes, but IP/DNS/GW/... can be changed by DHCP
  memcpy(macAddress, _mac, sizeof(macAddress));
  useDhcp = _useDhcp;
  resetDefaults(_ip, _dns, _gw, _netmask);
}

void Network::resetDefaults(const uint32_t _ip, const uint32_t _dns, const uint32_t _gw, const uint32_t _netmask) {
  defaultIpAddress = _ip;
  defaultDns = _dns;
  defaultGateway = _gw;
  defaultNetmask = _netmask;
}

uint8_t Network::relaunch() {
  uint8_t rc = ERROR_NONE;
#if defined(NETWORK_ETHERNET_ENC28J60) || defined(NETWORK_ETHERNET_WIZNET)
  if (!isPhyOk()) { return ERROR_NET; }

  __DMLL( DEBUG_PORT.print(FSH_P(STRING_Network_relaunch_with)); 
        DEBUG_PORT.print(FSH_P(useDhcp ? STRING_DHCP : STRING_static_ip));
        DEBUG_PORT.print(FSH_P(STRING_3xDot_Space)); 
      )

  // !!! All begin() procedures must be make soft reset & init network chip registers

  // this part of code must be excluded if not FEATURE_NET_DHCP_ENABLE to avoid including DHCP-related procedures from the Ethernet library
#if defined(FEATURE_NET_DHCP_ENABLE)
  if (useDhcp) {
    // beginWithDHCP() returns 1 on success, and 0 on fail.
    if (Ethernet.begin(macAddress)) {
      resetDefaults((uint32_t)Ethernet.localIP(), (uint32_t)Ethernet.dnsServerIP(), (uint32_t)Ethernet.gatewayIP(), (uint32_t)Ethernet.subnetMask()); 
    } else {
      rc = ERROR_DHCP;
    }
  } else {
#else // <<= If (DHCP feature not enabled) or (DHCP feature enabled, but useDhcp is not set)
    Ethernet.begin(macAddress, IPAddress(defaultIpAddress), IPAddress(defaultDns), IPAddress(defaultGateway), IPAddress(defaultNetmask));
#endif

#if defined(FEATURE_NET_DHCP_ENABLE)
  }
#endif
  if (ERROR_NONE == rc) {
     __DMLL( DEBUG_PORT.println(FSH_P(STRING_ok)); )
     phyConfigured = true;
  } else {
     __DMLL( DEBUG_PORT.println(FSH_P(STRING_fail)); )
  }
#elif defined(NETWORK_SERIAL_INTERFACE) // #if defined(NETWORK_ETHERNET_ENC28J60) || defined(NETWORK_ETHERNET_WIZNET)
  NETWORK_SERIAL_INTERFACE_PORT.begin(constNetworkSerialInterfaceSpeed, constNetworkSerialInterfaceMode);

#endif // #if defined(NETWORK_ETHERNET_ENC28J60) || defined(NETWORK_ETHERNET_WIZNET)
  return rc;
}

void Network::printNetworkInfo() {

#if defined(NETWORK_ETHERNET_ENC28J60) || defined(NETWORK_ETHERNET_WIZNET)
  //DEBUG_PORT.print(F("MAC     : ")); printArray(macAddress, sizeof(macAddress), &Serial, MAC_ADDRESS);
  DEBUG_PORT.print(F("IP\t: ")); DEBUG_PORT.println(Ethernet.localIP());
  DEBUG_PORT.print(F("Subnet\t: ")); DEBUG_PORT.println(Ethernet.subnetMask());
  DEBUG_PORT.print(F("Gateway\t: ")); DEBUG_PORT.println(Ethernet.gatewayIP());
#ifdef NETWORK_ETHERNET_ENC28J60
  DEBUG_PORT.print(F("ENC28J60: rev ")); DEBUG_PORT.println(Enc28J60.getrev());
#endif
#elif defined(NETWORK_SERIAL_INTERFACE) // #if defined(NETWORK_ETHERNET_ENC28J60) || defined(NETWORK_ETHERNET_WIZNET)
  DEBUG_PORT.print(F("Serial speed: ")); DEBUG_PORT.println(constNetworkSerialInterfaceSpeed);
#endif

}

void Network::printPHYState() {
#ifdef NETWORK_ETHERNET_ENC28J60
   DEBUG_PORT.print(F("ECON1.RXEN  : ")); DEBUG_PORT.println(Enc28J60.readReg((uint8_t) ECON1), BIN); 
   DEBUG_PORT.print(F("EIR.RXERIF  : ")); DEBUG_PORT.println(Enc28J60.readReg((uint8_t) EIR), BIN); 
   DEBUG_PORT.print(F("ESTAT.BUFFER: ")); DEBUG_PORT.println(Enc28J60.readReg((uint8_t) ESTAT), BIN); 
#endif
}

