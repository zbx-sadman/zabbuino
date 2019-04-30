// Config & common included files
#include "sys_includes.h"

#include "service.h"

#include "net_platforms.h"

#include "network.h"

NetworkClass::NetworkClass() {
  // isPhyOk returns error state if detect it
  phyConfigured = false;
}


int16_t NetworkClass::maintain() {
  int16_t rc = DHCP_CHECK_NONE;

#if defined(FEATURE_NET_DHCP_ENABLE)
  if (useDhcp) {
     // maintain() subroutine adds more fat to WIZnet drivers, because includes DHCP functionality to firmware even it not need
     rc = Ethernet.maintain();
     if (rc) { resetDefaults((uint32_t) Ethernet.localIP(), (uint32_t) Ethernet.dnsServerIP(), (uint32_t) Ethernet.gatewayIP(), (uint32_t) Ethernet.subnetMask()); }
  }
#endif // FEATURE_NET_DHCP_ENABLE
  return rc;
}

void NetworkClass::tick() {
#if defined(NETWORK_ETH_ENC28J60) 
    // tick() subroutine is very important for UIPEthernet, and must be called often (every ~250ms). If ENC28J60 driver not used - this subroutine do nothing
   Ethernet.tick();
#endif
}

uint8_t NetworkClass::isPhyConfigured() { 
  uint32_t localIpAddress = (uint32_t) Ethernet.localIP();
  // NET_ZERO_IP_ADDRESS != defaultIpAddress -> network is not configured after start
  // localIpAddress == defaultIpAddress -> Netcard is missconfigured (may be drop configuration on power problem or something?)
  phyConfigured = (phyConfigured && (NET_ZERO_IP_ADDRESS != defaultIpAddress) && (localIpAddress == defaultIpAddress));
  return phyConfigured;
}


uint8_t NetworkClass::isPhyOk() {
  uint8_t phyState = true;

  DTSL( Serial.print(FSH_P(STRING_Checking_PHY)); Serial.print(FSH_P(STRING_3xDot_Space)); )   

#if defined(NETWORK_ETH_WIZNET) //NETWORK_ETH_WIZNET 
  // hardwareStatus() returns 0x00 if wrong hardware or no hardware answer
//  DTSL( Serial.print("HW stat:"); Serial.println(Ethernet.hardwareStatus()); )   
  phyState = !!Ethernet.hardwareStatus();

#elif defined(NETWORK_ETH_ENC28J60) //NETWORK_ETH_WIZNET 
  phyState = !!Enc28J60.getrev();
  uint8_t stateEconRxen    = Enc28J60.readReg((uint8_t) ECON1) & ECON1_RXEN;
  uint8_t stateEirRxerif   = Enc28J60.readReg((uint8_t) EIR) & EIR_RXERIF;
  uint8_t stateEstatBuffer = Enc28J60.readReg((uint8_t) ESTAT) & ESTAT_BUFFER;
  if (!stateEconRxen || (stateEstatBuffer && stateEirRxerif)) {
     // ENC28J60 rise error flag
     phyState = false;
  }
#endif

  DTSL( Serial.println(FSH_P(phyState ? STRING_ok : STRING_fail)); )

  return phyState;
}


void NetworkClass::init(const uint8_t* _mac, const uint32_t _ip, const uint32_t _dns, const uint32_t _gw, const uint32_t _netmask, const uint8_t _useDhcp) {
  // MAC never changes, but IP/DNS/GW/... can be changed by DHCP
  memcpy(macAddress, _mac, sizeof(macAddress));
  useDhcp = _useDhcp;
  resetDefaults(_ip, _dns, _gw, _netmask);
#if defined(NETWORK_ETH_WIZNET)
   // let chip warming up
  //if (WIZNET_WARMING_UP_TIME > millis() ) { delay(millis()-WIZNET_WARMING_UP_TIME); }
  //if (WIZNET_WARMING_UP_TIME > millis() ) { delay(WIZNET_WARMING_UP_TIME); }
#endif
}

void NetworkClass::resetDefaults(const uint32_t _ip, const uint32_t _dns, const uint32_t _gw, const uint32_t _netmask) {
  defaultIpAddress = _ip;
  defaultDns = _dns;
  defaultGateway = _gw;
  defaultNetmask = _netmask;
}

uint8_t NetworkClass::relaunch() {
  uint8_t rc = ERROR_NONE;
  if (!isPhyOk()) { return ERROR_NET; }

  DTSL( Serial.print(FSH_P(STRING_Network_relaunch_with)); 
        Serial.print(FSH_P(useDhcp ? STRING_DHCP : STRING_static_ip));
        Serial.print(FSH_P(STRING_3xDot_Space)); 
      )

  // !!! All begin() procedures must be make soft reset & init network chip registers

  // this part of code must be excluded if not FEATURE_NET_DHCP_ENABLE to avoid including DHCP-related procedures from the Ethernet library
#if defined(FEATURE_NET_DHCP_ENABLE)
  if (useDhcp) {
    // beginWithDHCP() returns 1 on success, and 0 on fail.
    if (Ethernet.begin(macAddress)) {
      resetDefaults((uint32_t) Ethernet.localIP(), (uint32_t) Ethernet.dnsServerIP(), (uint32_t) Ethernet.gatewayIP(), (uint32_t) Ethernet.subnetMask()); 
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
     DTSL( Serial.println(FSH_P(STRING_ok)); )
     phyConfigured = true;
  } else {
     DTSL( Serial.println(FSH_P(STRING_fail)); )
  }

  return rc;
}

void NetworkClass::printNetworkInfo() {
  //Serial.print(F("MAC     : ")); printArray(macAddress, sizeof(macAddress), &Serial, MAC_ADDRESS);
  Serial.print(F("IP\t: ")); Serial.println(localIP());
  Serial.print(F("Subnet\t: ")); Serial.println(subnetMask());
  Serial.print(F("Gateway\t: ")); Serial.println(gatewayIP());
#ifdef NETWORK_ETH_ENC28J60
  Serial.print(F("ENC28J60: rev ")); Serial.println(Enc28J60.getrev());
#endif
}

void NetworkClass::printPHYState() {
#ifdef NETWORK_ETH_ENC28J60
   Serial.print(F("ECON1.RXEN  : ")); Serial.println(Enc28J60.readReg((uint8_t) ECON1), BIN); 
   Serial.print(F("EIR.RXERIF  : ")); Serial.println(Enc28J60.readReg((uint8_t) EIR), BIN); 
   Serial.print(F("ESTAT.BUFFER: ")); Serial.println(Enc28J60.readReg((uint8_t) ESTAT), BIN); 
#endif
}


/*

void NetworkClass::restart() {
  uint8_t useDefaultIP = true;

#ifdef FEATURE_NET_DHCP_ENABLE
  start:
  // User want to use DHCP with Zabbuino?
  if (useDHCP) {
     DTSM( Serial.println(F("Obtaining address from DHCP...")); )
     // Try to ask DHCP server & turn off DHCP feature for that session if no offer recieved
     if (0 == begin(macAddress)) {
        DTSM( Serial.println(F("No success")); )
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


 //Serial.println("p4"));
  // No DHCP offer recieved or no DHCP need - start with stored/netDefault IP config
  if (useDefaultIP) {
     DTSM( Serial.println(F("Use default IP")); )
     useDHCP = false;

#if defined(NETWORK_ETH_ENC28J60) || defined(NETWORK_ETH_WIZNET)
     // That overloaded .begin() function return nothing
     // Second netConfig.ipAddress used as dns-address
     begin(macAddress, netDefaultIP, netDefaultIP, netDefaultGW, netDefaultNM);
     server.begin();

  }
}
*/