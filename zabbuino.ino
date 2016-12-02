/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 PROGRAMM FEATURES SECTION
                   
   Please refer to the "zabbuino.h" file for enabling or disabling Zabbuino's features and "src/defaults.h" to deep tuning.
   if connected sensors seems not work - first check setting in port_protect[], port_mode[], port_pullup[] arrays in I/O PORTS SETTING SECTION of "src/defaults.h" file
*/

#include "zabbuino.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 GLOBAL VARIABLES SECTION
*/

netconfig_t* netConfig;

#ifdef INTERRUPT_USE
// EXTERNAL_NUM_INTERRUPTS its a macro from <wiring_private.h>
volatile extInterrupt_t *extInterrupt;
#endif


EthernetServer ethServer(10050);

EthernetClient ethClient;

static char cBuffer[BUFFER_SIZE+1]; // +1 for trailing \0
// some array items used into timer's interrupt
volatile int32_t *sysMetrics;


/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                      STARTUP SECTION
*/

void setup() {
  uint8_t i;
  // Q: What to do if netConfig or sysMetrics is NULL?
  netConfig = new netconfig_t;
  // Last idx is not the same that array size
  sysMetrics = new int32_t[IDX_METRICS_LAST+1];
  //delay(3000);
  pinMode(PIN_FACTORY_RESET, INPUT_PULLUP);
  pinMode(PIN_STATE_LED, OUTPUT);

#ifdef ADVANCED_BLINKING
  // blink on start
  blinkMore(6, 50, 500);
#endif

 // Init metrics
#ifdef SERIAL_USE
  Serial.begin(9600);
#endif // SERIAL_USE

  DTSL( SerialPrint_P(PSTR(ZBX_AGENT_VERISON)); )
  DTSL( SerialPrintln_P(PSTR(" waked up")); )
  
  sysMetrics[IDX_METRIC_SYS_VCCMIN] = sysMetrics[IDX_METRIC_SYS_VCCMAX] = getADCVoltage(ANALOG_CHAN_VBG);
  sysMetrics[IDX_METRIC_SYS_RAM_FREE] = sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] = (int32_t) getRamFree();
  sysMetrics[IDX_METRIC_SYS_CMD_LAST] = sysMetrics[IDX_METRIC_SYS_CMD_COUNT] = sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N] = 0;
  sysMetrics[IDX_METRIC_NET_ENC_REINIT_REASON] = sysMetrics[IDX_METRIC_NET_ENC_REINITS] = sysMetrics[IDX_METRIC_NET_ENC_PKTCNT_MAX] = 0;

//  uint32_t startSerial = millis();
/*
On ATMega32u4 (may be breadboard and dupont wires is bad)

wait for serial with timeout: 
64 bytes from 172.16.100.226: icmp_req=382 ttl=64 time=755 ms
64 bytes from 172.16.100.226: icmp_req=383 ttl=64 time=1076 ms
...
64 bytes from 172.16.100.226: icmp_req=389 ttl=64 time=1270 ms
64 bytes from 172.16.100.226: icmp_req=390 ttl=64 time=1104 ms

no wait for serial: 
64 bytes from 172.16.100.226: icmp_req=400 ttl=64 time=1.29 ms
64 bytes from 172.16.100.226: icmp_req=401 ttl=64 time=1.25 ms
...
64 bytes from 172.16.100.226: icmp_req=407 ttl=64 time=1.25 ms
64 bytes from 172.16.100.226: icmp_req=408 ttl=64 time=1.27 ms

So... no debug with Serial Monitor at this time
*/
//  while (!Serial && (SERIAL_WAIT_TIMEOUT < (millis() - startSerial) )) {;}   // Leonardo: wait for serial monitor a little bit
//  while (!Serial);   // Leonardo: wait for serial monitor a little bit

#ifdef FEATURE_EEPROM_ENABLE

/* -=-=-=-=-=-=-=-=-=-=-=-
    FACTORY RESET BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */

  // Set mode of PIN_FACTORY_RESET and turn on internal pull resistor
  digitalWrite(PIN_STATE_LED, LOW);
  // Check for PIN_FACTORY_RESET shorting to ground?
  // (when pulled INPUT pin shorted to GND - digitalRead() return LOW)
  if (LOW == digitalRead(PIN_FACTORY_RESET)){
  DTSM( SerialPrintln_P(PSTR("The factory reset button is pressed")); )
    // Fire up state LED
    digitalWrite(PIN_STATE_LED, HIGH);
    // Wait some msecs
    delay(HOLD_TIME_TO_FACTORY_RESET);
    // PIN_FACTORY_RESET still shorted?
    if (LOW == digitalRead(PIN_FACTORY_RESET)){
       DTSM( SerialPrintln_P(PSTR("Rewrite EEPROM with defaults...")); )
       setConfigDefaults(netConfig);
       saveConfigToEEPROM(netConfig);
       // Blink fast while PIN_FACTORY_RESET shorted to GND
       DTSM( SerialPrintln_P(PSTR("Done. Release the factory reset button now")); )
       while (LOW == digitalRead(PIN_FACTORY_RESET)) {
          digitalWrite(PIN_STATE_LED, millis() % 100 < 50);
      }
    }
    digitalWrite(PIN_STATE_LED, LOW);
  } // if (LOW == digitalRead(PIN_FACTORY_RESET))

/* -=-=-=-=-=-=-=-=-=-=-=-
    CONFIGURATION LOAD BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */
  // Try to load configuration from EEPROM
  if (false == loadConfigFromEEPROM(netConfig)) {
     DTSM( SerialPrintln_P(PSTR("Load error")); )
     // bad CRC detected, use default values for this run
     setConfigDefaults(netConfig);
     if (!saveConfigToEEPROM(netConfig)) {
      // what to do with saving error?     
     }
  }
#else // FEATURE_EEPROM_ENABLE
     DTSM( SerialPrintln_P(PSTR("Use default network settings")); )
     // Use hardcoded values if EEPROM feature disabled
     setConfigDefaults(netConfig);
#endif // FEATURE_EEPROM_ENABLE

#ifdef FEATURE_NET_DHCP_FORCE
     netConfig->useDHCP = true;
#endif

/* -=-=-=-=-=-=-=-=-=-=-=-
    NETWORK START BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */
#ifdef FEATURE_NET_DHCP_ENABLE
  // User want to use DHCP with Zabbuino?
  if (true == netConfig->useDHCP) {
     DTSM( SerialPrintln_P(PSTR("Obtaining address from DHCP...")); )
      // Try to ask DHCP server
     if (0 == Ethernet.begin(netConfig->macAddress)) {
        DTSM( SerialPrintln_P(PSTR("No success")); )
         // No offer recieved - switch off DHCP feature for that session
         netConfig->useDHCP = false;
      }
  }
#else // FEATURE_NET_DHCP_ENABLE
  netConfig->useDHCP=false;
#endif // FEATURE_NET_DHCP_ENABLE

  // No DHCP offer recieved or no DHCP need - start with stored/default IP config
  if (false == netConfig->useDHCP) {
     DTSM( SerialPrintln_P(PSTR("Use static IP")); )
     // That overloaded .begin() function return nothing
     // Second netConfig->ipAddress used as dns-address
     Ethernet.begin(netConfig->macAddress, netConfig->ipAddress, netConfig->ipAddress, netConfig->ipGateway, netConfig->ipNetmask);
  }
  
  DTSL( SerialPrintln_P(PSTR("Serving on:")); )
  DTSL( SerialPrint_P(PSTR("MAC     : ")); printArray(netConfig->macAddress, sizeof(netConfig->macAddress), DBG_PRINT_AS_MAC); )
  DTSL( SerialPrint_P(PSTR("Hostname: ")); Serial.println(netConfig->hostname); )
  DTSL( SerialPrint_P(PSTR("IP      : ")); Serial.println(Ethernet.localIP()); )
  DTSL( SerialPrint_P(PSTR("Subnet  : ")); Serial.println(Ethernet.subnetMask()); )
  DTSL( SerialPrint_P(PSTR("Gateway : ")); Serial.println(Ethernet.gatewayIP()); )
  DTSL( SerialPrint_P(PSTR("Password: ")); Serial.println(netConfig->password, DEC); )
  // This codeblock is compiled if UIPethernet.h is included
#ifdef UIPETHERNET_H
  DTSL( SerialPrint_P(PSTR("ENC28J60: rev ")); Serial.println(Enc28J60.getrev()); )
#endif

  // Start listen sockets
  ethServer.begin();

/* -=-=-=-=-=-=-=-=-=-=-=-
    OTHER STUFF INIT BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */

  // I/O ports initialization. Refer to "I/O PORTS SETTING SECTION" in zabbuino.h
  for (i = PORTS_NUM; 0 != i;) {
    // experimental: variable decrement that place outside for() save a little progspace. 
    i--;
    setPortMode(i, (uint8_t) pgm_read_word(&(port_mode[i])), (uint8_t) pgm_read_word(&(port_pullup[i])));
  }
#ifdef INTERRUPT_USE
  // Init external interrupts info structure
  // Q: What to do if extInterrupt is NULL?
  extInterrupt = new extInterrupt_t[EXTERNAL_NUM_INTERRUPTS];
  // Init external interrupts info structure
  for (i = EXTERNAL_NUM_INTERRUPTS; 0 != i;) {
  //while (i) {
    i--;
    // -1 - interrupt is detached
    // just use NOT_AN_INTERRUPT = -1 macro from Arduino.h
    extInterrupt[i].mode = NOT_AN_INTERRUPT;
    extInterrupt[i].owner = OWNER_IS_NOBODY;
    extInterrupt[i].count = 0;    
  }
#endif

  // Uncomment to force protect (enable even useProtection is false) your system from illegal access for change runtime settings and reboots 
#ifdef FEATURE_PASSWORD_PROTECTION_FORCE
  netConfig->useProtection = true;
#endif

#ifdef LIBWIRE_USE
   Wire.begin();
#endif

#ifdef FEATURE_WATCHDOG_ENABLE
  // Watchdog activation
  wdt_enable(WTD_TIMEOUT);
#endif

#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
   // need to analyze return code?
   initTimerOne(SYS_METRIC_RENEW_PERIOD);
#endif

#ifdef ADVANCED_BLINKING
  // blink on init end
  blinkMore(2, 1000, 1000);
#endif
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                      RUN SECTION
*/
void loop() {
  uint8_t result, encPktCnt,
          errorCode = ERROR_NONE;
  uint16_t blinkType = BLINK_NOPE;
  int16_t *_argOffset;
  uint32_t nowTime, processStartTime, processEndTime, prevDHCPRenewTime, prevENCReInitTime, prevNetProblemTime, prevSysMetricGatherTime, clientConnectTime, netDebugPrintTime;
  uint32_t ramBefore;

  _argOffset = new int16_t[ARGS_MAX];
    
  // Correcting timestamps
  prevDHCPRenewTime = prevENCReInitTime = prevNetProblemTime = prevSysMetricGatherTime = netDebugPrintTime = millis();

  // if no exist while() here - netProblemTime must be global or static - its will be 0 every loop() and time-related cases will be processeed abnormally
  // ...and while save some cpu ticks because do not call everytime from "hidden main()" subroutine, and do not init var, and so.
  while (true) { 
#ifdef FEATURE_WATCHDOG_ENABLE
    // reset watchdog every loop
    wdt_reset();
#endif
    nowTime = millis();

      // correctVCCMetrics() must be always inline compiled
    
    // Gather internal metrics periodically
    if (SYS_METRIC_RENEW_PERIOD <= (uint32_t) (nowTime - prevSysMetricGatherTime)) { 

// When FEATURE_DEBUG_COMMANDS_ENABLE is disabled, compiler can be omit gatherSystemMetrics() sub (due find no operators inside) and trow exception
#ifndef GATHER_METRIC_USING_TIMER_INTERRUPT
       gatherSystemMetrics();
#endif 
       sysMetrics[IDX_METRIC_SYS_VCC] = getADCVoltage(ANALOG_CHAN_VBG);
       correctVCCMetrics(sysMetrics[IDX_METRIC_SYS_VCC]);
       prevSysMetricGatherTime = millis();
    }

#ifdef FEATURE_NET_DHCP_ENABLE
    // DHCP used in this session and time to renew lease?
    if (true == netConfig->useDHCP && (NET_DHCP_RENEW_PERIOD <= (uint32_t) (nowTime - prevDHCPRenewTime))) {
       // Ethernet library's manual say that Ethernet.maintain() can be called every loop for DHCP renew, but i won't do this so often
       // ...and how many overhead give Ethernet.maintain() ?
       result = Ethernet.maintain();
       // Renew procedure finished with success
       switch (result) {
          case DHCP_CHECK_NONE:
          case DHCP_CHECK_RENEW_OK:
          case DHCP_CHECK_REBIND_OK:
            blinkType = BLINK_NOPE;
            errorCode = ERROR_NONE;
           // No alarm blink  need, network activity registred, renewal period restarted
            prevDHCPRenewTime = prevNetProblemTime = millis();
            break;
          default: 
            // Got some errors - blink with "DHCP problem message"
            blinkType = BLINK_DHCP_PROBLEM;    
            errorCode = ERROR_DHCP;
            DTSM( SerialPrintln_P(PSTR("DHCP renew problem occured")); )
       }
    }
#endif // FEATURE_NET_DHCP_ENABLE

    // No DHCP problem found, but no data recieved or network activity for a long time
    if (ERROR_NONE == errorCode && (NET_IDLE_TIMEOUT <= (uint32_t) (nowTime - prevNetProblemTime))) { 
       blinkType = BLINK_NET_PROBLEM; 
       errorCode = ERROR_NET;
    }

#ifdef USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE
    encPktCnt = Enc28J60.readReg((uint8_t) NET_ENC28J60_EPKTCNT);
    if (sysMetrics[IDX_METRIC_NET_ENC_PKTCNT_MAX] < encPktCnt) { sysMetrics[IDX_METRIC_NET_ENC_PKTCNT_MAX] = encPktCnt; }

    // Time to reinit ENC28J60?
/*
    if (NET_ENC28J60_REINIT_PERIOD <= (uint32_t) (nowTime - prevENCReInitTime)) {
       // if EIR.TXERIF or EIR.RXERIF is set - ENC28J60 detect error, if ECON1.RXEN is clear - ENC28J60's filter feature drop all packets. 
       // To resolve this situation need to re-init module 
       uint8_t stateEconRxen = Enc28J60.readReg((uint8_t) NET_ENC28J60_ECON1) & NET_ENC28J60_ECON1_RXEN;
       uint8_t stateEirTxerif = Enc28J60.readReg((uint8_t) NET_ENC28J60_EIR) & NET_ENC28J60_EIR_TXERIF;
       uint8_t stateEstatBuffer = Enc28J60.readReg((uint8_t) NET_ENC28J60_ESTAT) & NET_ENC28J60_ESTAT_BUFFER;
       if (!stateEconRxen || (stateEstatBuffer & stateEirTxerif))
       //if (!stateEconRxen || (stateEir & NET_ENC28J60_EIR_TXERIF))
       //if (!stateEconRxen)
       {
          // just for debug. the code must be removed on release
          if (!stateEconRxen) {
              sysMetrics[IDX_METRIC_NET_ENC_REINIT_REASON] = 0x01;
          } else {
              sysMetrics[IDX_METRIC_NET_ENC_REINIT_REASON] = 0x02;
          } 

          DTSM( SerialPrintln_P(PSTR("ENC28J60 reinit")); )
          Enc28J60.init(netConfig->macAddress); 
          sysMetrics[IDX_METRIC_NET_ENC_REINITS]++;
          sysMetrics[IDX_METRIC_NET_ENC_PKTCNT_MAX] = 0;
          //delay(NET_STABILIZATION_DELAY);
       } 
       prevENCReInitTime = millis();
    }
  */  
#endif // USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE

 #ifdef FEATURE_NET_DEBUG_TO_SERIAL
       // Print debug data every... 5 seconds
        if ((5000UL <= (uint32_t) (nowTime - netDebugPrintTime))) {
           NDTS( SerialPrint_P(PSTR("Millis: "));  Serial.println(nowTime); )
//           NDTS( SerialPrint_P(PSTR("  ECON1: ")); Serial.println(Enc28J60.readReg((uint8_t) NET_ENC28J60_ECON1), BIN); )
           if (ERROR_NONE != errorCode) {
              NDTS( SerialPrint_P(PSTR("Error code: ")); Serial.println(errorCode); )
              NDTS( SerialPrint_P(PSTR("Last executed command: ")); Serial.println(sysMetrics[IDX_METRIC_SYS_CMD_LAST], HEX); )
              NDTS( SerialPrint_P(PSTR("Memory free: ")); Serial.println(sysMetrics[IDX_METRIC_SYS_RAM_FREE]); )
              NDTS( SerialPrint_P(PSTR("Memory free (min): ")); ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { Serial.println(sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN]); } )
              NDTS( SerialPrint_P(PSTR("Client: ")); Serial.println(ethClient, HEX); )
              NDTS( SerialPrint_P(PSTR("MAC     : ")); printArray(netConfig->macAddress, sizeof(netConfig->macAddress), DBG_PRINT_AS_MAC); )
              NDTS( SerialPrint_P(PSTR("IP      : ")); Serial.println(Ethernet.localIP()); )
              NDTS( SerialPrint_P(PSTR("Subnet  : ")); Serial.println(Ethernet.subnetMask()); )
              NDTS( SerialPrint_P(PSTR("Gateway : ")); Serial.println(Ethernet.gatewayIP()); )
#ifdef ENC28J60_ETHERNET_SHIELD
              NDTS( SerialPrintln_P(PSTR("ENC28J60")); )
              NDTS( SerialPrint_P(PSTR("reinits: ")); Serial.println(sysMetrics[IDX_METRIC_NET_ENC_REINITS]); )
              NDTS( SerialPrint_P(PSTR("  ESTAT: ")); Serial.println(Enc28J60.readReg((uint8_t) NET_ENC28J60_ESTAT), BIN); )
              NDTS( SerialPrint_P(PSTR("    EIE: ")); Serial.println(Enc28J60.readReg((uint8_t) NET_ENC28J60_EIE), BIN); )
              NDTS( SerialPrint_P(PSTR("    EIR: ")); Serial.println(Enc28J60.readReg((uint8_t) NET_ENC28J60_EIR), BIN); )
              NDTS( SerialPrint_P(PSTR("  ECON1: ")); Serial.println(Enc28J60.readReg((uint8_t) NET_ENC28J60_ECON1), BIN); )
              NDTS( SerialPrint_P(PSTR("  ECON2: ")); Serial.println(Enc28J60.readReg((uint8_t) NET_ENC28J60_ECON2), BIN); )
              NDTS( SerialPrint_P(PSTR("EPKTCNT: ")); Serial.println(Enc28J60.readReg((uint8_t) NET_ENC28J60_EPKTCNT)); )
              NDTS( SerialPrint_P(PSTR("  ERXST: ")); Serial.println(Enc28J60.readReg((uint8_t) NET_ENC28J60_ERXST)); )
              NDTS( SerialPrint_P(PSTR("  ERXND: ")); Serial.println(Enc28J60.readReg((uint8_t) NET_ENC28J60_ERXND)); )
             
#endif // #ifdef ENC28J60_ETHERNET_SHIELD
           }
           netDebugPrintTime = millis();
           Serial.println();
        }
#endif

 
    // Turn off state led if no errors occured in the current loop.
    // Otherwise - make LED blinked or just turn on
    if (ERROR_NONE == errorCode) {
       digitalWrite(PIN_STATE_LED, LOW);
    } else {
#ifdef ON_ALARM_STATE_BLINK
      digitalWrite(PIN_STATE_LED, nowTime % 1000 < blinkType);
#else
      digitalWrite(PIN_STATE_LED, HIGH);
#endif
    } // if (ERROR_NONE == errorCode) ... else 

    // No active session is exist. Looking for new connection.
    if (!ethClient.connected()) {
       ethClient = ethServer.available();
       if (ethClient) { 
           clientConnectTime = millis(); 
       }
    } else {
       // Drop clients with slow connection
       if (NET_ACTIVE_CLIENT_CONNECTION_TIMEOUT <= (uint32_t) (nowTime - clientConnectTime)) { 
          // analyzeStream set internal read/write pointer to begin of buiffer after '\n' detection
          analyzeStream('\n', _argOffset); 
          ethClient.stop(); continue; 
       } 
    }
    
#ifdef FEATURE_SERIAL_LISTEN_TOO
    // A lot of chars wait for reading
    if (!ethClient.available() && !Serial.available()) { continue; }
    // ethClient have more priority. Serial & Ethernet data may be mixed, so Serial input must be used for debug only.
    result = ethClient.available() ? ethClient.read() : ( Serial.available() ?  Serial.read() : false);
    Serial.println((char) result);
#else
    // ethClient.available() return available bytes in frame. Request can be splitted to few frames and .available() can return 0 on end of first frame
    // But this does not mean that the request is complete, just need to wait next frame until ethClient.connected() == true.
    if (!ethClient.available()) { continue; }
    result = ethClient.read();
#endif              
    result = analyzeStream((char) result, _argOffset);    
    // result is true if analyzeStream() do not finished and need more data
    // result is false if EOL or trailing char detected or there no room in buffer or max number or args parsed...   
    if (true == result) { continue; }
    // ethClient.connected() returns true even client is disconnected, but leave the data in the buffer. 
    // Check this scenario... 
    // But checking must be disable for commands which coming in from the Serial. Otherwise commands will be never executed if no active network client exist.
#ifndef FEATURE_SERIAL_LISTEN_TOO
    if (!ethClient.connected()) { continue; }
#endif              
    /*****  processing command *****/
    // Destroy unused client's data (if trailed symbol detected, but other data is still in the buffer).
    ethClient.flush(); 
#ifdef FEATURE_SERIAL_LISTEN_TOO
    // Just flush buffer without current stream detection (serial or ethernet) because serial input used for debug purposes (when no ethernet avaiable) only. 
    Serial.flush(); 
#endif              

    // Fire up State led, than will be turned off on next loop
    digitalWrite(PIN_STATE_LED, HIGH);
    // may be need test for client.connected()? 
    processStartTime = millis();
    DTSM( ramBefore = getRamFree(); )
    sysMetrics[IDX_METRIC_SYS_CMD_LAST] = executeCommand(_argOffset);
    // system.run[] recieved, need to run another command, which taken from option #0 by cmdIdx() sub
    if (RUN_NEW_COMMAND == sysMetrics[IDX_METRIC_SYS_CMD_LAST]) {
       int16_t k = 0;
       // simulate command recieving to properly string parsing
       while (analyzeStream(cBuffer[k], _argOffset)) { k++; }
       DTSM( SerialPrintln_P(PSTR("Run new command")); )
       sysMetrics[IDX_METRIC_SYS_CMD_LAST] = executeCommand(_argOffset);
    }
    processEndTime = millis();
    // use processEndTime as processDurationTime
    processEndTime = processEndTime - processStartTime ;
    DTSM( SerialPrint_P(PSTR("Execute time:")); Serial.println(processEndTime );
          SerialPrint_P(PSTR("Memory bytes leak: ")); Serial.println((ramBefore - getRamFree()));
          Serial.println(); 
    )
    
    if (sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] < processEndTime) {
       sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = processEndTime;
       sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N] = sysMetrics[IDX_METRIC_SYS_CMD_LAST];
    }

    // Wait some time to finishing answer send, close connection, and restart network activity control cycle
    //delay(NET_STABILIZATION_DELAY);
    ethClient.stop(); 
    // analyzeStream set internal read/write pointer to begin of buiffer after '\n' detection
    analyzeStream('\n', _argOffset); 
    prevENCReInitTime = prevNetProblemTime = millis();
    blinkType = BLINK_NOPE;
    errorCode = ERROR_NONE;
 } // while(true)
}

/* ****************************************************************************************************************************
*
*  Stream analyzing subroutine
*  Detect Zabbix packets, on-fly spit incoming stream to command & arguments
*
**************************************************************************************************************************** */
static uint8_t analyzeStream(char _charFromClient, int16_t* _argOffset) {
  uint8_t static needSkipZabbix2Header = 0, 
                 cmdSliceNumber        = 0,
                 isEscapedChar         = 0,
                 doubleQuotedString    = false;
  uint16_t static bufferWritePosition;

  // If there is not room in buffer - simulate EOL recieving
  if (BUFFER_SIZE <= bufferWritePosition ) { _charFromClient = '\n'; }
  
  // Put next char to buffer
  cBuffer[bufferWritePosition] = (doubleQuotedString) ? _charFromClient : tolower(_charFromClient); 
  // no SerialPrint_P(PSTR(...)) used to avoid slow perfomance on analyze loops
  // Development mode only debug message level used
  DTSD( Serial.print("rcv: "); 
       Serial.print(cBuffer[bufferWritePosition], HEX);  
       Serial.print(" '"); 
       Serial.print((char) cBuffer[bufferWritePosition]); Serial.println("' "); 
  )
  // When ZBX_HEADER_PREFIX_LENGTH chars is saved to buffer - test its for Zabbix2 protocol header prefix ("ZBXD\01") presence
  if (ZBX_HEADER_PREFIX_LENGTH == bufferWritePosition) {
     if (0 == memcmp(&cBuffer, (ZBX_HEADER_PREFIX), ZBX_HEADER_PREFIX_LENGTH)) {
        // If packet have prefix - set 'skip whole header' flag
        needSkipZabbix2Header = true;
        DTSD( Serial.println("ZBX header detected"); )
     }
  }

  // When ZBX_HEADER_LENGTH chars is saved to buffer - check 'skip whole header' flag and just begin write new data from begin of buffer.
  // This operation 'drops' Zabbix2 header
  if (ZBX_HEADER_LENGTH == bufferWritePosition && needSkipZabbix2Header) {
     bufferWritePosition = 0;
     needSkipZabbix2Header = false;
     DTSD( Serial.println("ZBX header dropped"); )
     // Return 'Need next char' and save a lot cpu time 
     return true;
  }

  // Process all chars if its not from header data
  if (!needSkipZabbix2Header) {
     // char is not escaped
     switch (_charFromClient) {
        // Doublequote sign is arrived
        case '"':
          if (!isEscapedChar) {
             // Doublequote is not escaped - just drop it and toggle "string is doublequoted" mode (do not convert char case,
             //  skip action on space, ']', '[', ',' detection). Then jump out from subroutine to get next char from client
             doubleQuotedString = !doubleQuotedString;
             return true;
          }
          // Doublequote is escaped. Move write position backward to one step and write doublequote sign to '\' position
          bufferWritePosition--;
          cBuffer[bufferWritePosition] = '"';
          isEscapedChar = false;
          break;

        // Backslash sign is arrived. If next char will be doublequote - its consider as escaped. But backslash is still in buffer as non-escape char
        case '\\':
          if (!isEscapedChar) {
             isEscapedChar = true;
          }            
          break;

        // Space found. Do nothing if its reached not in doublequoted string, and next char will be written to same position. 
        case 0x20:
          // Return 'Need next char'
          if (!doubleQuotedString) { return true; }
          break;

        // Delimiter or separator found.
        case '[':
        case ',':
          // If its reached not in doublequoted string - process it as control char.
          if (!doubleQuotedString) { 
             //  If '_argOffset' array is not exhausted - push begin of next argument (bufferWritePosition+1) on buffer to arguments offset array. 
            if (ARGS_MAX > cmdSliceNumber) { _argOffset[cmdSliceNumber] = bufferWritePosition + 1; }
               cmdSliceNumber++; 
               // Make current buffer segment like C-string
               cBuffer[bufferWritePosition] = '\0'; 
            }
          break;

        // Final square bracket found. Do nothing and next char will be written to same position. 
        case ']':
          // If its reached in doublequoted string - just leave its as regular character
          //    ...otherwise - process as 'EOL sign'
          if (doubleQuotedString) { break; }

        // EOL detected
        case '\n':
          // Save last argIndex that pointed to <null> item. All unused _argOffset[] items must be pointed to this <null> item too.
          cBuffer[bufferWritePosition] = '\0'; 
          while (ARGS_MAX > cmdSliceNumber) { _argOffset[cmdSliceNumber++] = bufferWritePosition;}
          // Change argIndex value to pass (ARGS_MAX < argIndex) condition 
          cmdSliceNumber = ARGS_MAX+1; break;
          break;

        // All next chars is non-escaped
        default: 
            isEscapedChar = false; 
     }

     // EOL reached or there is not room to store args. Stop stream analyzing and do command executing
     if (ARGS_MAX < cmdSliceNumber) {
        // Clear vars for next round, and return false as 'Do not need next char'
        bufferWritePosition = cmdSliceNumber = isEscapedChar = doubleQuotedString = 0;
        needSkipZabbix2Header = false;
        return false;
     }             
  }
  // 
  bufferWritePosition++;
  // Return 'Need next char' and save a lot cpu time 
  return true;
}

/* ****************************************************************************************************************************
*
*  
**************************************************************************************************************************** */
static int16_t executeCommand(int16_t* _argOffset)
{
  int8_t result = RESULT_IS_FAIL;
  uint8_t accessGranted, i, i2CAddress, i2COption, i2CValue[4];
  int16_t i2CRegister, cmdIdx = -1;
  // duration option in the tone[] command is ulong
  uint32_t argv[ARGS_MAX];
  //int64_t value = 0;  // Zabbix use 64-bit numbers, but we can use only -uint32_t...+uint32_t range. Error can be occurs on ltoa() call with value > long_int_max 
  long_ulong_t value;

  //DTS( Serial.print("[0] "); Serial.println(millis()); )

  value.longvar = 0;
  
  sysMetrics[IDX_METRIC_SYS_CMD_COUNT]++;

  i = arraySize(commands);
  for (i = arraySize(commands); 0 != i;) {
  //while (i) {
    i--;
    DTSD( Serial.print("# ");  Serial.print(i, HEX); Serial.print(" => "); SerialPrintln_P((char*)pgm_read_word(&(commands[i]))); )
    if (0 == strcmp_P(cBuffer, (char*)pgm_read_word(&(commands[i])))) {cmdIdx = i; break;}
  }

  //DTS( Serial.print("[1] "); Serial.println(millis()); )
 
  DTSM( SerialPrint_P(PSTR("Execute command #")); Serial.print(cmdIdx, HEX); SerialPrint_P(PSTR(" => `")); Serial.print(cBuffer); Serial.println("`"); )

  // batch convert args to number values
  // first _argOffset item have index 0
  for (i = ARGS_MAX; 0 != i;) {
  //  while (i) {
     i--;
     argv[i] = ('\0' == cBuffer[_argOffset[i]]) ? 0 : strtoul(&cBuffer[_argOffset[i]], NULL,0);
     DTSH( 
        SerialPrint_P(PSTR("argv[")); Serial.print(i); SerialPrint_P(PSTR("] => \"")); 
        if ('\0' == cBuffer[_argOffset[i]]) {
           SerialPrint_P(PSTR("<null>")); 
        } else {
           Serial.print(&cBuffer[_argOffset[i]]); 
        }
        SerialPrint_P(PSTR("\" => ")); Serial.print(argv[i]);
        SerialPrint_P(PSTR(", offset =")); Serial.println(_argOffset[i]);
     )
  }

//   DTS( Serial.print("[2] "); Serial.println(millis()); )

  // 0.012 sec to switch(). need to use second switch?
  // Check rights for password protected commands
  accessGranted = (!netConfig->useProtection || argv[0] == netConfig->password); 

  i2CAddress = (uint8_t) argv[2];
  i2CRegister = (('\0' != cBuffer[_argOffset[3]]) ? (int16_t) argv[3] : I2C_NO_REG_SPECIFIED);
  // i2COption can be used as 'length', 'bitNumber' or 'data' variable
  i2COption = (uint8_t) argv[4];

  //DTS( Serial.print("[3] "); Serial.println(millis()); )

   switch (cmdIdx) {
//  case  CMD_ZBX_NOPE: 
//        break;
    case CMD_ZBX_AGENT_PING:
      /*/
      //   agent.ping 
      /=*/
      result = RESULT_IS_OK;
      break;

    case CMD_ZBX_AGENT_HOSTNAME:
      /*/
      /=/   agent.hostname
      /*/
      strcpy(cBuffer, netConfig->hostname);
      result = RESULT_IN_BUFFER;
      break;
         
    case CMD_ZBX_AGENT_VERSION:
      /*/
      /=/  agent.version
      /*/
      strcpy_P(cBuffer, PSTR(ZBX_AGENT_VERISON));
      result = RESULT_IN_BUFFER;
      break;

    case CMD_SYSTEM_RUN:
      /*/
      /=/  system.run[newCommand
      /*/
      if ('\0' == cBuffer[_argOffset[0]]) { break; }

      // take length of 0-th arg + 1 byte for '\0'
      i = (_argOffset[1] - _argOffset[0]) + 1;
      // move it to begin of buffer to using as new incoming command
      // Note: ~8bytes can be saved with copying bytes in while() cycle. But source code will not beauty
      memmove(cBuffer, &cBuffer[_argOffset[0]], i);
      cBuffer[i] = '\n';
      return RUN_NEW_COMMAND;
      break;

    case CMD_SYS_UPTIME:
      /*/
      /=/  sys.uptime
      /*/
      value.ulongvar  = (uint32_t) millis() / 1000;
      result = RESULT_IN_ULONGVAR;
      break;

    case CMD_NET_ENC_REINITS:
      /*/
      /=/  enc.reinits
      /*/
      value.ulongvar  = (uint32_t) sysMetrics[IDX_METRIC_NET_ENC_REINITS];
      result = RESULT_IN_ULONGVAR;
      break;

    case CMD_NET_ENC_REINIT_REASON:
      /*/
      /=/  enc.reinits
      /*/
      value.ulongvar  = (uint32_t) sysMetrics[IDX_METRIC_NET_ENC_REINIT_REASON];
      result = RESULT_IN_ULONGVAR;
      break;

    case CMD_NET_ENC_PKTCNT_MAX:
      /*/
      /=/  enc.pktcntmax
      /*/
      value.ulongvar  = (uint32_t) sysMetrics[IDX_METRIC_NET_ENC_PKTCNT_MAX];
      result = RESULT_IN_ULONGVAR;
      break;



    case CMD_ARDUINO_ANALOGWRITE:
      /*/
      /=/  analogWrite[pin, value]
      /*/
      if (! isSafePin(argv[0])) { break; }

      analogWrite(argv[0], argv[1]);
      result = RESULT_IS_OK;
      break;
      
    case CMD_ARDUINO_ANALOGREAD:
      /*/
      /=/  analogRead[pin, analogReferenceSource, mapToLow, mapToHigh]
      /*/
#ifdef FEATURE_AREF_ENABLE
      // change source of the reference voltage if its given
      if ('\0' != cBuffer[_argOffset[1]]) {
         analogReference(argv[1]);
         delayMicroseconds(2000);
      }
#endif
 
      if (! isSafePin(argv[0])) { break; }
      
      value.longvar = (int64_t) analogRead(argv[0]);
      if ('\0' != cBuffer[_argOffset[2]] && '\0' != cBuffer[_argOffset[3]]) {
         value.ulongvar = (uint32_t) map(result, 0, 1023, argv[2], argv[3]);
      }
      result = RESULT_IN_ULONGVAR;
      break;      
  
#ifdef FEATURE_AREF_ENABLE
    case CMD_ARDUINO_ANALOGREFERENCE:
      /*/
      /=/  analogReference[source]
      /*/
      analogReference(argv[0]);
      result = RESULT_IS_OK;
      break;
#endif
  
    
    case CMD_ARDUINO_DELAY:
      /*/
      /=/  delay[time]
      /*/
      delay(argv[0]);
      result = RESULT_IS_OK;
      break;
      

    case CMD_ARDUINO_DIGITALWRITE:
      /*/
      /=/  digitalWrite[pin, value, testPin, testValue]
      /*/
      // if testPin defined - check both pin to safety
      result = ('\0' == cBuffer[_argOffset[2]]) ? isSafePin(argv[0]) : (isSafePin(argv[0]) && isSafePin(argv[2]));
      if (!result) { break; }

      // turn on or turn off logic on pin
      digitalWrite(argv[0], argv[1]);
      result = RESULT_IS_OK; 

      if ('\0' == cBuffer[_argOffset[2]]) { break; }
      // when testPin defined - switch testPin mode to input, wait a lot, and check testPin state.
      // if readed value not equal testValue - return FAIL
      pinMode(argv[2], INPUT_PULLUP);
      delay(10);
      if (digitalRead(argv[2]) != argv[3]){ result = RESULT_IS_FAIL; }

      break;

    case CMD_ARDUINO_DIGITALREAD:
      /*/
      /=/  digitalRead[pin]
      /*/
      value.ulongvar = (int64_t) digitalRead(argv[0]);
      result = RESULT_IN_ULONGVAR;
      break;


#ifdef FEATURE_TONE_ENABLE
    case CMD_ARDUINO_TONE:
      /*/
      /=/  tone[pin, frequency, duration]
      /*/
      if (! isSafePin(argv[0])) { break; }

      result = RESULT_IS_OK;
      if ('\0' != cBuffer[_argOffset[2]]) { tone(argv[0], argv[1], argv[2]); break;} 
      tone(argv[0], argv[1]);
      break;
  
    case CMD_ARDUINO_NOTONE:
      /*/
      /*/
      if (! isSafePin(argv[0])) { break; }

      result = RESULT_IS_OK;
      noTone(argv[0]);
      break;
  
#endif

#ifdef FEATURE_RANDOM_ENABLE
    case CMD_ARDUINO_RANDOMSEED:
      /*/
      /=/  randomSeed[value]
      /*/
      randomSeed((0 == argv[0]) ? (int32_t) millis() : argv[0]);
      result = RESULT_IS_OK;
      break;
   
    case CMD_ARDUINO_RANDOM:
      /*/
      /=/  random[min, max]
      /*/
      //  !! random return long
      value.ulongvar = (int64_t) ('\0' == cBuffer[_argOffset[1]]) ? (int32_t) random(argv[0]) : (int32_t) random(argv[0], argv[1]);
      result = RESULT_IN_ULONGVAR;

      break;
#endif // FEATURE_RANDOM_ENABLE


#ifdef FEATURE_EEPROM_ENABLE
    case CMD_SET_HOSTNAME:
      /*/
      /=/  set.hostname[password, hostname]
      /*/
      if (!accessGranted) { break; }
      // need check for arg existsience?
      // cBuffer[_argOffset[1]] != \0 if argument #2 given
      if ('0' == cBuffer[_argOffset[1]]) { break; }
      
      // copy <1-th arg length> bytes from 1-th arg of buffer (0-th arg contain password) to hostname 
      i = (uint8_t) _argOffset[2]-_argOffset[1];
      if (i > ZBX_AGENT_HOSTNAME_MAXLEN) { i = ZBX_AGENT_HOSTNAME_MAXLEN; }
      //copy 0 .. (ZBX_AGENT_HOSTNAME_MAXLEN-1) chars from buffer to hostname
      memcpy(netConfig->hostname, &cBuffer[_argOffset[1]], i);
      // Terminate string
      netConfig->hostname[ZBX_AGENT_HOSTNAME_MAXLEN]='\0';
      saveConfigToEEPROM(netConfig);
      result = RESULT_IS_OK;
      break;
  
    case CMD_SET_PASSWORD:
      /*/
      /=/  set.password[oldPassword, newPassword]
      /*/
      if (!accessGranted) { break; }
      if ('\0' == cBuffer[_argOffset[1]]) { break; }

      // take new password from argument #2
      netConfig->password = argv[1];
      saveConfigToEEPROM(netConfig);
      result = RESULT_IS_OK;
      break;
  
    case CMD_SET_SYSPROTECT:
      /*/
      /=/  set.sysprotect[password, protection]
      /*/
      if (!accessGranted) { break; }
      if ('\0' == cBuffer[_argOffset[1]]) { break; }

      netConfig->useProtection = (1 == argv[1]) ? true : false;
      saveConfigToEEPROM(netConfig);
      result = RESULT_IS_OK;
      break;
   
    case CMD_SET_NETWORK:
      /*/
      /=/  set.network[password, useDHCP, macAddress, ipAddress, ipNetmask, ipGateway]
      /*/
      if (!accessGranted) { break; }
      uint8_t ip[4], mac[6], success;
      success = true;
      // useDHCP flag coming from first argument and must be numeric (boolean) - 1 or 0, 
      // argv[0] data contain in cBuffer[_argOffset[1]] placed from _argOffset[0]
      netConfig->useDHCP = (uint8_t) argv[1];
      // ip, netmask and gateway have one structure - 4 byte
      // take 6 bytes from second argument of command and use as new MAC-address
      // if convertation is failed (return false) succes variable must be falsed too via logic & operator
      success &= hstoba((uint8_t*) mac, &cBuffer[_argOffset[2]], arraySize(netConfig->macAddress));
      memcpy(netConfig->macAddress, &mac, arraySize(netConfig->macAddress));

      // We need to make sure that 'success' is true before hstoba() calling, but seems that any testing here just get progspace without profit
      // success = success && hstoba(..) <- can inline test 'success' and call hstoba(..) only on 'true' value
      
      // use 4 bytes from third argument of command as new IP-address. sizeof(IPAddress) returns 6 instead 4
      success &= hstoba((uint8_t*) &ip, &cBuffer[_argOffset[3]], 4);
      netConfig->ipAddress = IPAddress(ip);
  
      // take 4 bytes from third argument of command an use as new IP Netmask
      success &= hstoba((uint8_t*) &ip, &cBuffer[_argOffset[4]], 4);
      netConfig->ipNetmask = IPAddress(ip);
  
      // convert 4 bytes from fourth argument to default gateway
      success &= hstoba((uint8_t*) &ip, &cBuffer[_argOffset[5]], 4);
      netConfig->ipGateway = IPAddress(ip);
 
      if (!success) { break; }
      // Save config to EEProm if success
      result = saveConfigToEEPROM(netConfig) ? RESULT_IS_OK : DEVICE_ERROR_EEPROM_CORRUPTED;
      break;
#endif // FEATURE_EEPROM_ENABLE

    case CMD_SYS_PORTWRITE:
      /*/
      /=/  portWrite[port, value]
      /*/
      if (PORTS_NUM >= (argv[0] - 96)) { break; }

      writeToPort((byte) argv[0] - 96, argv[1]);
      result = RESULT_IS_OK;

      break;
  
#ifdef FEATURE_SHIFTOUT_ENABLE
    case CMD_SYS_SHIFTOUT:
      /*/
      /=/  shiftOut[dataPin, clockPin, latchPin, bitOrder, data]
      /*/
      // i variable used as latchPinDefined
      i = ('\0' != argv[2]) && isSafePin(argv[2]);  
      if (isSafePin(argv[0]) &&  isSafePin(argv[1])) {
         if (i) { digitalWrite(argv[2], LOW); }
         shiftOutAdvanced(argv[0], argv[1], argv[3], &cBuffer[_argOffset[4]]);
         if (i) { digitalWrite(argv[2], HIGH);}
         result = RESULT_IS_OK;
      }
      break;
#endif

#ifdef FEATURE_WS2812_ENABLE
    case CMD_WS2812_SENDRAW:
      /*/
      /=/  WS2812.sendRaw[dataPin, data]
      /=/  >> need to increase ARGS_PART_SIZE, because every encoded LED color take _six_ HEX-chars => 10 leds stripe take 302 (2+50*6) byte of incoming buffer only
      /*/
      // Tested on ATmega328@16 and 8 pcs WS2812 5050 RGB LED bar
      if (isSafePin(argv[0])) {
         WS2812Out(argv[0], &cBuffer[_argOffset[1]]);
         result = RESULT_IS_OK;
      }
      break;
#endif // FEATURE_WS2812_ENABLE

    case CMD_SYS_REBOOT:
      /*/
      /=/  reboot[password]
      /*/
      if (! accessGranted) { break; }

      ethClient.println("1");
      // hang-up if no delay
      delay(NET_STABILIZATION_DELAY);
      ethClient.stop();
#ifdef FEATURE_WATCHDOG_ENABLE
      // Watchdog deactivation
      wdt_disable();
#endif
      asm volatile ("jmp 0");  
      break;

#ifdef FEATURE_DEBUG_COMMANDS_ENABLE
    case CMD_SYS_MCU_NAME:
      /*/
      /=/  sys.mcu.name
      /*/
      strcpy_P(cBuffer, PSTR(_AVR_CPU_NAME_));
      result = RESULT_IN_BUFFER;
      break;
   
    case CMD_SYS_MCU_ID:
      /*/
      /=/  sys.mcu.id
      /*/
      // Read 10 bytes with step 1 (0x0E..0x17) of the signature row <= http://www.avrfreaks.net/forum/unique-id-atmega328pb
      getBootSignatureBytes(cBuffer, 0x0E, 10, 1);
      result = RESULT_IN_BUFFER;
      break;

    case CMD_SYS_MCU_SIGN:
      /*/
      /=/  sys.mcu.sign
      /*/
      // Read 3 bytes with step 2 (0x00, 0x02, 0x04) of the signature row <= http://www.avrfreaks.net/forum/device-signatures
      getBootSignatureBytes(cBuffer, 0x00, 3, 2);
      result = RESULT_IN_BUFFER;
      break;

    case CMD_SYS_NET_MODULE:
      /*/
      /=/  sys.net.module
      /*/
      strcpy_P(cBuffer, PSTR(NET_MODULE_NAME));
      result = RESULT_IN_BUFFER;
      break;

    case CMD_SYS_CMD_COUNT:
      /*/
      /=/  sys.cmd.count
      /*/
      if (argv[0]) { sysMetrics[IDX_METRIC_SYS_CMD_COUNT] = 0; } 
      value.ulongvar = (uint32_t) sysMetrics[IDX_METRIC_SYS_CMD_COUNT];
      result = RESULT_IN_ULONGVAR;
      break;

    case CMD_SYS_CMD_TIMEMAX:
      /*/
      /=/  sys.cmd.timemax[resetCounter]
      /*/
      if (cBuffer[_argOffset[0]]) { sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = 0; sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N] = 0; } 
      value.ulongvar = (uint32_t) sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX];
      result = RESULT_IN_ULONGVAR;
      break;

    case CMD_SYS_CMD_TIMEMAX_N:
      /*/
      /=/  sys.cmd.timemax.n
      /*/
      value.ulongvar = (uint32_t) sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N];
      ethClient.println(value.ulongvar, HEX);
      result = RESULT_IS_PRINTED;
      break;
   
    case CMD_SYS_RAM_FREE:
      /*/
      /=/  sys.ram.free
      /*/
      //  That metric must be collected periodically to avoid returns always same data
      value.ulongvar = (uint32_t) sysMetrics[IDX_METRIC_SYS_RAM_FREE];
      result = RESULT_IN_ULONGVAR;
      break;

    case CMD_SYS_RAM_FREEMIN:
      /*/
      /=/  sys.ram.freemin
      /*/
      // Without ATOMIC_BLOCK block using sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] variable can be changed in interrupt on reading
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
         value.ulongvar = (uint32_t) sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN];
      }
         result = RESULT_IN_ULONGVAR;
      break;

#endif


    case CMD_SYS_VCC:
      /*/
      /=/ sys.vcc
      /*/
      // Take VCC
      value.ulongvar = (uint32_t) getADCVoltage(ANALOG_CHAN_VBG);
      // VCC may be bigger than max or smaller than min. 
      // To avoid wrong results and graphs in monitoring system - correct min/max metrics
      correctVCCMetrics(value.ulongvar);
      result = RESULT_IN_ULONGVAR;
      break;
  
    case CMD_SYS_VCCMIN:
      /*/
      /=/ sys.vccMin
      /*/
      value.ulongvar = (uint32_t) sysMetrics[IDX_METRIC_SYS_VCCMIN];
      result = RESULT_IN_ULONGVAR;
      break;
  
    case CMD_SYS_VCCMAX:
      /*/
      /=/ sys.vccMax
      /*/
      value.ulongvar = (uint32_t) sysMetrics[IDX_METRIC_SYS_VCCMAX];
      result = RESULT_IN_ULONGVAR;
      break;
  

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
    case CMD_EXTINT_COUNT:
      /*/
      /=/  extInt.count[intPin, mode]
      /=/  
      /=/  Unfortunately, (result == RESULT_IN_ULONGVAR && value.ulongvar == 0) and (result == RESULT_IS_FAIL) are looks equal for zabbix -> '0'
      /*/
      if (! isSafePin(argv[0])) { break; }
      result = manageExtInt(&value.ulongvar, argv[0], argv[1]);
      break;
      
#endif // FEATURE_EXTERNAL_INTERRUPT_ENABLE

#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE
    case CMD_INCENC_VALUE:
      /*/
      /=/  incEnc.value[terminalAPin, terminalBPin, initialValue]
      /*/
      if (! isSafePin(argv[0]) || ! isSafePin(argv[1])) { break; }
      // argv[3] (intNumber) currently not used
      result = manageIncEnc(&value.longvar, argv[0], argv[1], argv[2]);
      break;
#endif // FEATURE_INCREMENTAL_ENCODER_ENABLE
     


#ifdef FEATURE_OW_ENABLE
    case CMD_OW_SCAN:
      /*/
      /=/  OW.scan[pin]
      /*/
      if (! isSafePin(argv[0])) { break; }
      result = scanOneWire(argv[0], &ethClient);
      break;
#endif // FEATURE_ONEWIRE_ENABLE


#ifdef FEATURE_I2C_ENABLE
    case CMD_I2C_SCAN:
      /*/
      /=/  I2C.scan[sdaPin, sclPin]
      /*/
      if (! isSafePin(argv[0]) || ! isSafePin(argv[1])) { break;}
      result = scanI2C(&ethClient);
      break;

    case CMD_I2C_WRITE:
      /*/
      /=/ i2c.write(sdaPin, sclPin, i2cAddress, register, data, numBytes)
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // i2COption used as 'data'
      argv[5] = constrain(argv[5], 1, 4);
      i = argv[5];
      while(i){
         i--;
         i2CValue[i] = argv[4] & 0xFF;
         argv[4] = argv[4] >> 8;
      }
      result = writeBytesToI2C(i2CAddress, i2CRegister, i2CValue, argv[5]);
      result = (0 == result) ? RESULT_IS_OK : RESULT_IS_FAIL;
      break;

    case CMD_I2C_READ:
      /*/
      /=/ i2c.read(sdaPin, sclPin, i2cAddress, register, length, numberOfReadings)
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }

      // i2COption used as 'length' - how much bytes must be read: 0..4 byte
      i2COption = constrain(i2COption, 1, 4);
      // numberOfReadings parameter is not specified. Do once reading only
      // Otherwise make ahead reading for re-run sensor conversion to flush old data
      if ('\0' == cBuffer[_argOffset[5]]) { 
          argv[5] = 1; 
      } else {
          // Just discard result
            readBytesFromi2C(i2CAddress, i2CRegister, i2CValue, i2COption);
      }
      // One reading at least must be done
      if (1 > argv[5]){ argv[5] = 1; }
         
      uint8_t readNumber;
      int32_t accResult, tmpResult;
      
      readNumber = argv[5]; accResult = 0;

      // Will be RESULT_IS_FAIL if readBytesFromi2C() not returns 0
      result = RESULT_IS_OK;
      while (readNumber) {
        readNumber--;
        delayMicroseconds(ADC_STABILIZATION_DELAY);
        if (0 != readBytesFromi2C(i2CAddress, i2CRegister, i2CValue, i2COption)) { result = RESULT_IS_FAIL; break; }
           // make int32 from i2C's bytes
           tmpResult = 0;
           for (i=0; i < i2COption; i++) {
             tmpResult <<= 8;
             tmpResult |= i2CValue[i];
           }
           accResult += tmpResult;
      }
 
      if (RESULT_IS_FAIL != result) {
         value.longvar = (int32_t) (accResult / argv[5]);
         result = RESULT_IN_LONGVAR;
      }
      break;

    case CMD_I2C_BITWRITE:
      /*/
      /=/  i2c.bitWrite(sdaPin, sclPin, i2cAddress, register, bitNumber, value)
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }

      // i2COption used as 'bit number'
      if (0 > i2COption || 7 < i2COption){
            result = RESULT_IS_FAIL;
            break;
         }
      // Use device's register if specified, read 1 byte
      result = readBytesFromi2C(i2CAddress, i2CRegister, i2CValue, 1);
      // "!!" convert value 0100 to 1.
      bitWrite (i2CValue[0], i2COption, (!!argv[5]));
      // Use device's register if specified, write 1 byte, returns Wire lib state
      result = writeByteToI2C(i2CAddress, i2CRegister, i2CValue[0]);
      result = (0 == result) ? RESULT_IS_OK : RESULT_IS_FAIL;
      break;
      
    case CMD_I2C_BITREAD:
      /*/
      /=/  i2c.bitRead(sdaPin, sclPin, i2cAddress, register, bit)
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // i2COption used as 'bit number'
      if (0 > i2COption || 7 < i2COption) { break; }
      // Use device's register if specified, read 1 byte
      result = readBytesFromi2C(i2CAddress, i2CRegister, i2CValue, 1);
      if (0 == result) { break ;}
      value.ulongvar = (uint32_t) bitRead(i2CValue[0], i2COption);
      result = RESULT_IN_ULONGVAR;
      break;


#endif // FEATURE_I2C_ENABLE

#ifdef FEATURE_DS18X20_ENABLE
    case CMD_DS18X20_TEMPERATURE:
      /*/
      /=/  DS18x20.temperature[pin, resolution, id]
      /*/
      if (! isSafePin(argv[0])) { break; }
      result = getDS18X20Metric(argv[0], argv[1], &cBuffer[_argOffset[2]], cBuffer);
      break;
#endif // FEATURE_DS18X20_ENABLE

#ifdef FEATURE_DHT_ENABLE
    case CMD_DHT_HUMIDITY:
      /*/
      /=/  DHT.humidity[pin, model]
      /*/
      if (! isSafePin(argv[0])) { break; }
      result = getDHTMetric(argv[0], argv[1], SENS_READ_HUMD, cBuffer);
      break;

    case CMD_DHT_TEMPERATURE:
      /*/
      /=/  DHT.temperature[pin, model]
      /*/
      if (! isSafePin(argv[0])) { break; }
      result = getDHTMetric(argv[0], argv[1], SENS_READ_TEMP, cBuffer);
      break;
   
#endif // FEATURE_DHT_ENABLE

       
#ifdef FEATURE_BMP_ENABLE
    case CMD_BMP_PRESSURE:
      /*/
      /=/  BMP.Pressure[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getBMPMetric(argv[0], argv[1], i2CAddress, argv[3], argv[4], SENS_READ_PRSS, cBuffer);
      break;

    case CMD_BMP_TEMPERATURE:
      /*/
      /=/ BMP.Temperature[sdaPin, sclPin, i2cAddress, overSampling]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getBMPMetric(argv[0], argv[1], i2CAddress, argv[3], argv[4], SENS_READ_TEMP, cBuffer);
      break;

      
#ifdef SUPPORT_BME280_INCLUDE
      case CMD_BME_HUMIDITY:
      /*/
      /=/  BME.Humidity[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getBMPMetric(argv[0], argv[1], i2CAddress, argv[3], argv[4], SENS_READ_HUMD, cBuffer);
      break;      
#endif // SUPPORT_BME280_INCLUDE 

#endif // FEATURE_BMP_ENABLE  


#ifdef FEATURE_BH1750_ENABLE
    case CMD_BH1750_LIGHT:
      /*/
      /=/  BH1750.light[sdaPin, sclPin, i2cAddress, mode]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getBH1750Metric(argv[0], argv[1], i2CAddress, argv[3], SENS_READ_LUX, cBuffer);
      break;
#endif // FEATURE_BH1750_ENABLE

#ifdef FEATURE_MAX7219_ENABLE
    case CMD_MAX7219_WRITE:
      /*/
      /=/  MAX7219.write[dataPin, clockPin, loadPin, intensity, data]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1]) || !isSafePin(argv[2])) { break; }
      writeToMAX7219(argv[0], argv[1], argv[2], argv[3], &cBuffer[_argOffset[4]]);
      result = RESULT_IS_OK;
      break;
#endif // FEATURE_MAX7219_ENABLE

#ifdef FEATURE_PCF8574_LCD_ENABLE
    case CMD_PCF8574_LCDPRINT:
      /*/
      /=/  PCF8574.LCDPrint[sdaPin, sclPin, i2cAddress, lcdBacklight, lcdType, data]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = printToPCF8574LCD(argv[0], argv[1], i2CAddress, argv[3], argv[4], &cBuffer[_argOffset[5]]);
      break;

#endif // FEATURE_PCF8574_LCD_ENABLE


#ifdef FEATURE_SHT2X_ENABLE
    case CMD_SHT2X_HUMIDITY:
      /*/
      /=/  SHT2X.Humidity[sdaPin, sclPin, i2cAddress]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getSHT2XMetric(argv[0], argv[1], i2CAddress, SENS_READ_HUMD, cBuffer);
      break;

    case CMD_SHT2X_TEMPERATURE:
      /*/
      /=/  SHT2X.Temperature[sdaPin, sclPin, i2cAddress]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getSHT2XMetric(argv[0], argv[1], i2CAddress, SENS_READ_TEMP, cBuffer);
      break;
#endif // FEATURE_SHT2X_ENABLE  

#ifdef FEATURE_ACS7XX_ENABLE
    case CMD_ACS7XX_ZC:
      /*/
      /=/  acs7xx.zc[sensorPin, refVoltage]
      /*/
      if (!isSafePin(argv[0])) { break; }
         /*
            for ATmega1280, ATmega2560, ATmega1284, ATmega1284P, ATmega644, ATmega644A, ATmega644P, ATmega644PA
               INTERNAL1V1 	2  - 1,1V
               INTERNAL2V56 	3  - 2,56V
            ATmega328 and so
               INTERNAL 	3  - 1,1V
            Both
               DEFAULT 	        1  - VCC
               EXTERNAL 	0  - AREF << mV on AREF, more than '3'
*/
         // if refVoltage skipped - use DEFAULT source
       if ('\0' != cBuffer[_argOffset[1]]) {
          argv[1] = DEFAULT;
       }
       result = getACS7XXMetric(argv[0], argv[1], SENS_READ_ZC, 0, 0, cBuffer);
      break;

    case CMD_ACS7XX_AC:
      /*/
      /=/  acs7xx.ac[sensorPin, refVoltage, sensitivity, zeroPoint] 
      /*/
      if (!isSafePin(argv[0])) { break; }
      // if refVoltage skipped - use DEFAULT source
      if ('\0' != cBuffer[_argOffset[1]]) {
         argv[1] = DEFAULT;
      }
      result = getACS7XXMetric(argv[0], argv[1], SENS_READ_AC, argv[2], (int32_t) argv[3], cBuffer);
      break;

    case CMD_ACS7XX_DC:
      /*/
      /=/  acs7xx.dc[sensorPin, refVoltage, sensitivity, zeroPoint] 
      /*/
      if (!isSafePin(argv[0])) { break; }
         // if refVoltage skipped - use DEFAULT source
      if ('\0' != cBuffer[_argOffset[1]]) {
         argv[1] = DEFAULT;
      }
      result = getACS7XXMetric(argv[0], argv[1], SENS_READ_DC, argv[2], (int32_t) argv[3], cBuffer);
      break;

#endif // FEATURE_ACS7XX_ENABLE


#ifdef FEATURE_ULTRASONIC_ENABLE
    case CMD_ULTRASONIC_DISTANCE:
      /*/
      /=/  ultrasonic.distance[triggerPin, echoPin]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      value.ulongvar = (uint32_t) getUltrasonicMetric(argv[0], argv[1]);
      result = RESULT_IN_ULONGVAR;
      break;
#endif // FEATURE_ULTRASONIC_ENABLE


#ifdef FEATURE_IR_ENABLE
    case CMD_IR_SEND:
      /*/
      /=/  ir.send[pwmPin, irPacketType, nBits, data, repeat, address]
      /*/
      // ATmega328: Use D3 only at this time
      // Refer to other Arduino's pinouts to find OC2B pin
//      if (isSafePin(argv[0]) && TIMER2B == digitalPinToTimer(argv[0])) {
         // irPWMPin - global wariable that replace IRremote's TIMER_PWM_PIN
//         irPWMPin = argv[0];
      result = sendCommandByIR(argv[1], argv[2], argv[3], argv[4], argv[5]);
//      }
      break;

    case CMD_IR_SENDRAW:
      /*/
      /=/  ir.sendRaw[pwmPin, irFrequency, nBits, data]
      /=/  >> need to increase ARGS_PART_SIZE, because every data`s Integer number take _four_ HEX-chars => 70 RAW array items take 282 (2+70*4) byte of incoming buffer only
      /*/
      // ATmega328: Use D3 only at this time
      // Refer to other Arduino's pinouts to find OC2B pin
 //     if (isSafePin(argv[0]) && TIMER2B == digitalPinToTimer(argv[0])) {
         // irPWMPin - global wariable that replace IRremote's TIMER_PWM_PIN
//         irPWMPin = argv[0];
      result = sendRawByIR(argv[1], argv[2], &cBuffer[_argOffset[3]]);
 //     }
      break;
#endif // FEATURE_IR_ENABLE


#ifdef FEATURE_PZEM004_ENABLE
    //
    //  0x0101A8C0 - an IP address for PZEM (192.168.1.1)
    //
    case CMD_PZEM004_CURRENT:
      /*/
      /=/  pzem004.current[rxPin, txPin, ip]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // cBuffer cast to (uint8_t*) to use with subroutine math and SoftwareSerial subs, because used instead sub's internal buffer and save a little RAM size.
      // Its will be casted to char* inside at moment when its need
      result = getPZEM004Metric(argv[0], argv[1], SENS_READ_AC, &cBuffer[_argOffset[2]], (uint8_t*) cBuffer);
      break;
    case CMD_PZEM004_VOLTAGE:
      /*/
      /=/  pzem004.voltage[rxPin, txPin, ip]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getPZEM004Metric(argv[0], argv[1], SENS_READ_VOLTAGE, &cBuffer[_argOffset[2]], (uint8_t*) cBuffer);
      break;
    case CMD_PZEM004_POWER:
      /*/
      /=/  pzem004.power[rxPin, txPin, ip]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getPZEM004Metric(argv[0], argv[1], SENS_READ_POWER, &cBuffer[_argOffset[2]], (uint8_t*) cBuffer);
      break;
    case CMD_PZEM004_ENERGY:
      /*/
      /=/  pzem004.energy[rxPin, txPin, ip]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getPZEM004Metric(argv[0], argv[1], SENS_READ_ENERGY, &cBuffer[_argOffset[2]], (uint8_t*) cBuffer);
      break;
#endif // FEATURE_PZEM004_ENABLE

#ifdef FEATURE_UPS_APCSMART_ENABLE

    case CMD_UPS_APCSMART:
      /*/
      /=/  ups.apcsmart[rxPin, txPin, command]
      /=/    command - HEX or ASCII
      /*/  
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getAPCSmartUPSMetric(argv[0], argv[1], (uint8_t*) &cBuffer[_argOffset[2]], (_argOffset[3] - _argOffset[2]) , (uint8_t*) cBuffer);
      break;

#endif // FEATURE_UPS_APCSMART_ENABLE

#ifdef FEATURE_UPS_MEGATEC_ENABLE
    case CMD_UPS_MEGATEC:
      /*/
      /=/  ups.megatec[rxPin, txPin, command, fieldNumber]
      /=/    command - HEX or ASCII
      /*/  
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getMegatecUPSMetric(argv[0], argv[1], (uint8_t*) &cBuffer[_argOffset[2]], argv[3], (uint8_t*) cBuffer);
      break;

#endif // FEATURE_UPS_APCSMART_ENABLE


    default:
      // Early increased command counter is decremented
      sysMetrics[IDX_METRIC_SYS_CMD_COUNT]--;
      // In default case command  is considered unknown.
      strcpy_P(cBuffer, PSTR((MSG_ZBX_NOTSUPPORTED)));
      result = RESULT_IN_BUFFER;
      break;
   }

   // Form the output buffer routine

//   DTS( Serial.print("[4] "); Serial.println(millis()); )
   // The result is not printed or already placed in the buffer
   if (RESULT_IS_PRINTED != result) {
      switch (result) {
         case RESULT_IN_BUFFER:
            break;
         case RESULT_IS_OK:
            //  '1' must be returned
            cBuffer[0] = '1';
            cBuffer[1] = '\0';
            break;
         case RESULT_IS_FAIL:
            // or '0'
            cBuffer[0] = '0';
            cBuffer[1] = '\0';
            break;
         case RESULT_IN_LONGVAR:
            //  or result value placed in 'value' variable and must be converted to C-string.
            ltoa(value.longvar, cBuffer, 10);
            break;
         case RESULT_IN_ULONGVAR:
            //  or result value placed in 'value' variable and must be converted to C-string.
            ultoa(value.ulongvar, cBuffer, 10);
            break;
         case DEVICE_ERROR_CONNECT:
            strcpy_P(cBuffer, PSTR((MSG_DEVICE_ERROR_CONNECT)));
            break;
         case DEVICE_ERROR_ACK_L:
            strcpy_P(cBuffer, PSTR((MSG_DEVICE_ERROR_ACK_L)));
            break;
         case DEVICE_ERROR_ACK_H:
            strcpy_P(cBuffer, PSTR((MSG_DEVICE_ERROR_ACK_H)));
            break;
         case DEVICE_ERROR_CHECKSUM:
            strcpy_P(cBuffer, PSTR((MSG_DEVICE_ERROR_CHECKSUM)));
            break;
         case DEVICE_ERROR_TIMEOUT:
            strcpy_P(cBuffer, PSTR((MSG_DEVICE_ERROR_TIMEOUT)));
            break;
         case DEVICE_ERROR_WRONG_ANSWER:
            strcpy_P(cBuffer, PSTR((MSG_DEVICE_ERROR_WRONG_ANSWER)));
            break;
         case DEVICE_ERROR_EEPROM_CORRUPTED:
            strcpy_P(cBuffer, PSTR((MSG_DEVICE_ERROR_EEPROM)));
            break;
      }
      //  Push out the buffer to the client
      ethClient.println(cBuffer);
   }
   DTSM( SerialPrint_P(PSTR("Result: ")); Serial.println(cBuffer); )
//   DTSH( Serial.print("[5] "); Serial.println(millis()); )
   return cmdIdx;
}

