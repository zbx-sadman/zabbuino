/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
*                                                                 PROGRAMM FEATURES SECTION
*
*   Please refer to the "basic.h" file for enabling or disabling Zabbuino's features and refer to the "src/tune.h" to deep tuning.
*   if connected sensors seems not work - first check setting in port_protect[], port_mode[], port_pullup[] arrays in I/O PORTS SETTING SECTION of "src/tune.h" file
*/
#include "src/dispatcher.h"
#include "src/transport.h" 

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
*                                                                 GLOBAL VARIABLES SECTION
*/

netconfig_t netConfig;

// some array items used into timer's interrupt
// too hard to calculate "enough to non-hung work" ram size if *sysMetrics & sysMetrics = new int32_t[IDX_METRICS_LAST+1] is used
volatile int32_t sysMetrics[IDX_METRICS_LAST+1];
TransportClass Transport;

#ifdef INTERRUPT_USE
 // Init external interrupts info structure
 // EXTERNAL_NUM_INTERRUPTS its a macro from <wiring_private.h>
volatile extInterrupt_t extInterrupt[EXTERNAL_NUM_INTERRUPTS];
#endif


/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
*                                                                      STARTUP SECTION
*/

void setup() {
  uint8_t i;

#ifdef SERIAL_USE
  Serial.begin(115200);
#endif // SERIAL_USE

  DTSL( SerialPrint_P(PSTR(ZBX_AGENT_VERISON)); )
  DTSL( SerialPrintln_P(PSTR(" waked up")); )
  
  sysMetrics[IDX_METRIC_SYS_VCCMIN] = sysMetrics[IDX_METRIC_SYS_VCCMAX] = getADCVoltage(ANALOG_CHAN_VBG);
  sysMetrics[IDX_METRIC_SYS_RAM_FREE] = sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] = (int32_t) getRamFree();
  sysMetrics[IDX_METRIC_SYS_CMD_LAST] = sysMetrics[IDX_METRIC_SYS_CMD_COUNT] = sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N] = 0;
  sysMetrics[IDX_METRIC_SYS_NET_REINITS] = 0;
  
//  uint32_t startSerial = millis();
/*
On ATMega32u4 (may be breadboard and dupont wires is bad)

>>> wait for serial with timeout: 

64 bytes from 172.16.100.226: icmp_req=382 ttl=64 time=755 ms
64 bytes from 172.16.100.226: icmp_req=383 ttl=64 time=1076 ms
...
64 bytes from 172.16.100.226: icmp_req=389 ttl=64 time=1270 ms
64 bytes from 172.16.100.226: icmp_req=390 ttl=64 time=1104 ms

>>> no wait for serial: 
64 bytes from 172.16.100.226: icmp_req=400 ttl=64 time=1.29 ms
64 bytes from 172.16.100.226: icmp_req=401 ttl=64 time=1.25 ms
...
64 bytes from 172.16.100.226: icmp_req=407 ttl=64 time=1.25 ms
64 bytes from 172.16.100.226: icmp_req=408 ttl=64 time=1.27 ms

So... no debug with Serial Monitor at this time
*/
//  while (!Serial && (SERIAL_WAIT_TIMEOUT < (millis() - startSerial) )) {;}   // Leonardo: wait for serial monitor a little bit
//  while (!Serial);   // Leonardo: wait for serial monitor a little bit

  pinMode(constFactoryResetButtonPin, INPUT_PULLUP);
  pinMode(constStateLedPin, OUTPUT);

#ifdef ADVANCED_BLINKING
  // blink on start
  blinkMore(6, 50, 500);
#endif

#ifdef FEATURE_EEPROM_ENABLE

/* -=-=-=-=-=-=-=-=-=-=-=-   FACTORY RESET BLOCK   -=-=-=-=-=-=-=-=-=-=-=- */
  // Set mode of constFactoryResetButtonPin and turn on internal pull resistor
  digitalWrite(constStateLedPin, LOW);
    // Check for constFactoryResetButtonPin shorting to ground?
  // (when pulled INPUT pin shorted to GND - digitalRead() return LOW)
  if (LOW == digitalRead(constFactoryResetButtonPin)){
  DTSL( SerialPrintln_P(PSTR("The factory reset button is pressed")); )
    // Fire up state LED
    digitalWrite(constStateLedPin, HIGH);
    // Wait some msecs
    delay(constHoldTimeToFactoryReset);
    // constFactoryResetButtonPin still shorted?
    if (LOW == digitalRead(constFactoryResetButtonPin)){
       DTSL( SerialPrintln_P(PSTR("Rewrite EEPROM with defaults...")); )
       setConfigDefaults(&netConfig);
       saveConfigToEEPROM(&netConfig);
       // Blink fast while constFactoryResetButtonPin shorted to GND
       DTSL( SerialPrintln_P(PSTR("Done. Release the factory reset button now")); )
       while (LOW == digitalRead(constFactoryResetButtonPin)) {
          digitalWrite(constStateLedPin, millis() % 100 < 50);
      }
    }
    digitalWrite(constStateLedPin, LOW);

  } // if (LOW == digitalRead(constFactoryResetButtonPin))

/* -=-=-=-=-=-=-=-=-=-=-=-   CONFIGURATION LOAD BLOCK  -=-=-=-=-=-=-=-=-=-=-=- */
  // Try to load configuration from EEPROM
  if (false == loadConfigFromEEPROM(&netConfig)) {
     DTSM( SerialPrintln_P(PSTR("Load error")); )
     // bad CRC detected, use default values for this run
     setConfigDefaults(&netConfig);
     if (!saveConfigToEEPROM(&netConfig)) {
      // what to do with saving error?     
     }
  }
#else // FEATURE_EEPROM_ENABLE
     DTSM( SerialPrintln_P(PSTR("Use default network settings")); )
     // Use hardcoded values if EEPROM feature disabled
     setConfigDefaults(&netConfig);
#endif // FEATURE_EEPROM_ENABLE

#ifdef FEATURE_NET_DHCP_FORCE
     netConfig.useDHCP = true;
#endif

/* -=-=-=-=-=-=-=-=-=-=-=-   NETWORK START BLOCK  -=-=-=-=-=-=-=-=-=-=-=- */
#ifdef FEATURE_NET_DHCP_ENABLE
  // User want to use DHCP with Zabbuino?
  if (true == netConfig.useDHCP) {
     DTSM( SerialPrintln_P(PSTR("Obtaining address from DHCP...")); )
      // Try to ask DHCP server
     if (0 == Transport.begin(netConfig.macAddress)) {
        DTSM( SerialPrintln_P(PSTR("No success")); )
         // No offer recieved - switch off DHCP feature for that session
         netConfig.useDHCP = false;
      }
  }
#else // FEATURE_NET_DHCP_ENABLE
  netConfig.useDHCP=false;
#endif // FEATURE_NET_DHCP_ENABLE

  // No DHCP offer recieved or no DHCP need - start with stored/default IP config
  if (false == netConfig.useDHCP) {
     DTSM( SerialPrintln_P(PSTR("Use static IP")); )
     // That overloaded .begin() function return nothing
     // Second netConfig.ipAddress used as dns-address
     Transport.begin(netConfig.macAddress, netConfig.ipAddress, netConfig.ipAddress, netConfig.ipGateway, netConfig.ipNetmask);
  }
  
  DTSL( SerialPrintln_P(PSTR("Serving on:")); )
  DTSL( SerialPrint_P(PSTR("MAC     : ")); printArray(netConfig.macAddress, sizeof(netConfig.macAddress), DBG_PRINT_AS_MAC); )
  DTSL( SerialPrint_P(PSTR("Hostname: ")); Serial.println(netConfig.hostname); )
  DTSL( SerialPrint_P(PSTR("IP      : ")); Serial.println(Transport.localIP()); )
  DTSL( SerialPrint_P(PSTR("Subnet  : ")); Serial.println(Transport.subnetMask()); )
  DTSL( SerialPrint_P(PSTR("Gateway : ")); Serial.println(Transport.gatewayIP()); )
  DTSL( SerialPrint_P(PSTR("Password: ")); Serial.println(netConfig.password, DEC); )
  // This codeblock is compiled if UIPethernet.h is included
#ifdef TRANSPORT_ETH_ENC28J60
  DTSL( SerialPrint_P(PSTR("ENC28J60: rev ")); Serial.println(Enc28J60.getrev()); )
#endif

  // Start listen sockets
  Transport.server.begin();

/* -=-=-=-=-=-=-=-=-=-=-=-   OTHER STUFF INIT BLOCK    -=-=-=-=-=-=-=-=-=-=-=- */

  // I/O ports initialization. Refer to "I/O PORTS SETTING SECTION" in zabbuino.h
  for (i = PORTS_NUM; 0 != i;) {
    // experimental: variable decrement that place outside for() save a little progspace. 
    i--;
    setPortMode(i, (uint8_t) pgm_read_word(&(port_mode[i])), (uint8_t) pgm_read_word(&(port_pullup[i])));
  }
#ifdef INTERRUPT_USE
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
  netConfig.useProtection = true;
#endif

#ifdef LIBWIRE_USE
   Wire.begin();
#endif

#ifdef FEATURE_WATCHDOG_ENABLE
  // Watchdog activation
  wdt_enable(constWtdTimeout);
#endif

#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
   // need to analyze return code?
   initTimerOne(constSysMetricGatherPeriod);
#endif

#ifdef ADVANCED_BLINKING
  // blink on init end
  blinkMore(2, 1000, 1000);
#endif
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
*                                                                      RUN SECTION
*/
void loop() {
  uint8_t result, 
          errorCode = ERROR_NONE;
  char incomingData;
  uint16_t blinkType = constBlinkNope ;
  // Last idx is not the same that array size
  int16_t argOffset[constArgC];
  uint32_t nowTime, processStartTime, processEndTime, prevNetModuleCheckTime, prevNetProblemTime, prevSysMetricGatherTime, clientConnectTime, netDebugPrintTime;
  uint32_t ramBefore;
  char cBuffer[constBufferSize+1]; // +1 for trailing \0
    
  // Correcting timestamps
  prevNetModuleCheckTime = prevNetProblemTime = prevSysMetricGatherTime = netDebugPrintTime = clientConnectTime = millis();
  
  // if no exist while() here - netProblemTime must be global or static - its will be 0 every loop() and time-related cases will be processeed abnormally
  // ...and while save some cpu ticks because do not call everytime from "hidden main()" subroutine, and do not init var, and so.
  while (true) { 
#ifdef FEATURE_WATCHDOG_ENABLE
    // reset watchdog every loop
    wdt_reset();
#endif
    nowTime = millis();    
    // Gather internal metrics periodically
    if (constSysMetricGatherPeriod <= (uint32_t) (nowTime - prevSysMetricGatherTime)) { 

// When FEATURE_DEBUG_COMMANDS_ENABLE is disabled, compiler can be omit gatherSystemMetrics() sub (due find no operators inside) and trow exception
#ifndef GATHER_METRIC_USING_TIMER_INTERRUPT
       gatherSystemMetrics();
#endif 
       sysMetrics[IDX_METRIC_SYS_VCC] = getADCVoltage(ANALOG_CHAN_VBG);
       // correctVCCMetrics() must be always inline compiled
       correctVCCMetrics(sysMetrics[IDX_METRIC_SYS_VCC]);
       prevSysMetricGatherTime = millis();
    }

#if defined(FEATURE_NET_DHCP_ENABLE) || defined (TRANSPORT_ETH_ENC28J60)
    // maintain() is very important for UIPEthernet, because it call internal tick() subroutine
    // but this subroutine adds more fat to WIZnet drivers, because includes DHCP functionality to firmware
    result = Transport.maintain();
#endif
    
#if defined(FEATURE_NET_DHCP_ENABLE)
    if (true == netConfig.useDHCP) {
       // Renew procedure finished with success
       switch (result) {
         case DHCP_CHECK_NONE:
         case DHCP_CHECK_RENEW_OK:
         case DHCP_CHECK_REBIND_OK:
           // No alarm blink need, network activity registred
           blinkType = constBlinkNope;
           errorCode = ERROR_NONE;
           break;
         default: 
          // Got some errors - blink with "DHCP problem message"
          blinkType = constBlinkDhcpProblem;    
          errorCode = ERROR_DHCP;
          DTSM( SerialPrintln_P(PSTR("DHCP renew problem occured")); )
       }
    }
#endif // FEATURE_NET_DHCP_ENABLE

    // No DHCP problem found, but no data recieved or network activity for a long time
    if (ERROR_NONE == errorCode && (constNetIdleTimeout <= (uint32_t) (nowTime - prevNetProblemTime))) { 
       blinkType = constBlinkNetworkProblem; 
       errorCode = ERROR_NET;
    }

#if defined(FEATURE_NETWORK_MONITORING)
    if (constNetModuleCheckPeriod <= (uint32_t) (nowTime - prevNetModuleCheckTime)) {
       // netModuleCheck() subroutine returns true if detect network module error and reinit it. 
       if (Transport.netModuleCheck()) { 
          sysMetrics[IDX_METRIC_SYS_NET_REINITS]++; 
          DTSM( SerialPrint_P(PSTR("Network module reinit #")); Serial.println(sysMetrics[IDX_METRIC_SYS_NET_REINITS]); )
       }
       prevNetModuleCheckTime = millis();
    }
#endif // FEATURE_NETWORK_MONITORING

    // Turn off state led if no errors occured in the current loop.
    // Otherwise - make LED blinked or just turn on
    if (ERROR_NONE == errorCode) {
       digitalWrite(constStateLedPin, LOW);
    } else {
#ifdef ON_ALARM_STATE_BLINK
      digitalWrite(constStateLedPin, nowTime % 1000 < blinkType);
#else
      digitalWrite(constStateLedPin, HIGH);
#endif
    } // if (ERROR_NONE == errorCode) ... else 


#ifdef FEATURE_SERIAL_LISTEN_TOO
    // Network connections will processed if no data in Serial buffer exist
    if (Serial.available() <= 0) { 
#endif      
        if (!Transport.client) {
          Transport.client = Transport.server.available(); 
          if (!Transport.client) { continue;}
             // reinit analyzer because previous session can be dropped or losted
             analyzeStream('\0', cBuffer, argOffset, REINIT_ANALYZER);    
             clientConnectTime = millis(); 
             DTSH( SerialPrint_P(PSTR("New client #")); Serial.println(Transport.client); )
       }
    
       // Client will be dropped if its connection so slow. Then round must be restarted.
       if (constNetSessionTimeout <= (uint32_t) (millis() - clientConnectTime)) { 
          DTSH( SerialPrint_P(PSTR("Drop client #")); Serial.println(Transport.client); )
          Transport.client.stop(); 
          continue; 
       } 
       
       // Network data can be splitted to a number frames and ethClient.available() can return 0 on first frame processing. Need to ignore it.
       if (!Transport.client.available()) { continue; }
       incomingData = Transport.client.read();
#ifdef FEATURE_SERIAL_LISTEN_TOO
    } else {
      // If in Serial buffer is exist just read it
      incomingData = Serial.read();
    }
#endif      
       
    result = analyzeStream(incomingData, cBuffer, argOffset, NO_REINIT_ANALYZER);    
    // result is true if analyzeStream() do not finished and need more data
    // result is false if EOL or trailing char detected or there no room in buffer or max number or args parsed...   
    if (true == result) { continue; }
    // ethClient.connected() returns true even client is disconnected, but leave the data in the buffer. 
    // Data can be readed, command will executed, but why get answer? If no recipient - why need to load MCU?
    // But checking must be disable for commands which coming in from the Serial. Otherwise commands will be never executed if no active network client exist.
#ifndef FEATURE_SERIAL_LISTEN_TOO
    if (!Transport.client.connected()) { continue; }
#endif       
    // Fire up State led, than will be turned off on next loop
    digitalWrite(constStateLedPin, HIGH);
    // may be need test for client.connected()? 
    processStartTime = millis();
    DTSM( ramBefore = getRamFree(); )
    DTSL( Serial.print(cBuffer); SerialPrint_P(PSTR(" => ")); )
    sysMetrics[IDX_METRIC_SYS_CMD_LAST] = executeCommand(cBuffer, argOffset);
    // When system.run[] command is recieved, need to run another command, which taken from option #0 by cmdIdx() sub
    if (RUN_NEW_COMMAND == sysMetrics[IDX_METRIC_SYS_CMD_LAST]) {
       int16_t k = 0;
       // simulate command recieving to properly string parsing
       while (analyzeStream(cBuffer[k], cBuffer, argOffset, false)) { k++; }
       DTSL( SerialPrintln_P(PSTR("Run new command")); )
       DTSL( Serial.println(cBuffer); SerialPrint_P(PSTR(" => ")); )
       sysMetrics[IDX_METRIC_SYS_CMD_LAST] = executeCommand(cBuffer, argOffset);
    }
    processEndTime = millis();
    // use processEndTime as processDurationTime
    processEndTime = processEndTime - processStartTime ;
    DTSM( SerialPrint_P(PSTR("Execute time:")); Serial.println(processEndTime);
          SerialPrint_P(PSTR("Memory bytes leak: ")); Serial.println((ramBefore - getRamFree()));
          Serial.println(); 
    )
    // Change internal runtime metrics if need
    if ((uint32_t) sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] < processEndTime) {
       sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = processEndTime;
       sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N] = sysMetrics[IDX_METRIC_SYS_CMD_LAST];
    }

    // Wait some time to finishing answer send, close connection, and restart network activity control cycle
    //delay(constNetStabilizationDelay);
    // Actually Ethernet lib's flush() do nothing, but UIPEthernet flush() free ENC28J60 memory blocks where incoming (?) data stored
    Transport.client.flush(); 
    Transport.client.stop(); 
#ifdef FEATURE_SERIAL_LISTEN_TOO
    // Flush the incoming Serial buffer by reading because Serial object have no clear procedure.
    while (0 < Serial.available()) { Serial.read(); }
#endif
    prevNetModuleCheckTime = prevNetProblemTime = millis();
    blinkType = constBlinkNope;
    errorCode = ERROR_NONE;
 } // while(true)
}

/* ****************************************************************************************************************************
*
*  Stream analyzing subroutine
*  Detect Zabbix packets, on-fly spit incoming stream to command & arguments
*
**************************************************************************************************************************** */
static uint8_t analyzeStream(char _charFromClient, char* _dst, int16_t* _argOffset, uint8_t doReInit) {
  uint8_t static needSkipZabbix2Header = false, 
                 cmdSliceNumber        = 0,
                 isEscapedChar         = 0,
                 doubleQuotedString    = false;
  uint16_t static bufferWritePosition  = 0;

  // Jump into reInitStage procedure. This is a bad programming style, but the subroutine must be lightweight.
  if (doReInit) { 
     // Temporary clean code stub 
     memset(_argOffset, '\0', constArgC*sizeof(int16_t));
     *_dst = '\0';
     goto reInitStage; 
  }

  // If there is not room in buffer - simulate EOL recieving
  if (constBufferSize <= bufferWritePosition ) { _charFromClient = '\n'; }
  
  // Put next char to buffer
  _dst[bufferWritePosition] = (doubleQuotedString) ? _charFromClient : tolower(_charFromClient); 
  // no SerialPrint_P(PSTR(...)) used to avoid slow perfomance on analyze loops
  // Development mode only debug message level used
  DTSD( Serial.print("anl: "); 
       Serial.print(_dst[bufferWritePosition], HEX);  
       Serial.print(" '"); 
       Serial.print((char) _dst[bufferWritePosition]); Serial.println("' "); 
  )
  // When ZBX_HEADER_PREFIX_LENGTH chars is saved to buffer - test its for Zabbix2 protocol header prefix ("ZBXD\01") presence
  // (ZBX_HEADER_PREFIX_LENGTH-1) was used because bufferWritePosition is start count from 0, not from 1
  if ((ZBX_HEADER_PREFIX_LENGTH-1) == bufferWritePosition) {
     if (0 == memcmp(_dst, (ZBX_HEADER_PREFIX), ZBX_HEADER_PREFIX_LENGTH)) {
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
          _dst[bufferWritePosition] = '"';
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
            if (constArgC > cmdSliceNumber) { _argOffset[cmdSliceNumber] = bufferWritePosition + 1; }
               cmdSliceNumber++; 
               // Make current buffer segment like C-string
               _dst[bufferWritePosition] = '\0'; 
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
          _dst[bufferWritePosition] = '\0'; 
          while (constArgC > cmdSliceNumber) { _argOffset[cmdSliceNumber++] = bufferWritePosition;}
          // Change argIndex value to pass (constArgC < argIndex) condition 
          cmdSliceNumber = constArgC+1;
          break;

        // All next chars is non-escaped
        default: 
            isEscapedChar = false; 
     }

     // EOL reached or there is not room to store args. Stop stream analyzing and do command executing
     if (constArgC < cmdSliceNumber) {
  reInitStage:
        DTSH( SerialPrintln_P(PSTR("Reinit analyzer")); )
        // Clear vars for next round, and return false as 'Do not need next char'
        bufferWritePosition = cmdSliceNumber = isEscapedChar = doubleQuotedString = 0;
        needSkipZabbix2Header = doubleQuotedString = false;
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
static int16_t executeCommand(char* _dst, int16_t* _argOffset)
{
  int8_t result;
  uint8_t accessGranted, i, i2CAddress, i2COption, i2CValue[4];
  int16_t i2CRegister, cmdIdx;
  // duration option in the tone[] command is ulong
  uint32_t argv[constArgC];
  // Zabbix use 64-bit numbers, but we can use only -uint32_t...+uint32_t range. Error can be occurs on ltoa() call with value > long_int_max 
  long_ulong_t value;

  //DTSM( Serial.print("[0] "); Serial.println(millis()); )

  value.longvar = 0;
  result = RESULT_IS_FAIL;
  cmdIdx = -1;
  
  sysMetrics[IDX_METRIC_SYS_CMD_COUNT]++;
  i = arraySize(commands);
  // Search given related command in the list of implemented functions
  for (; 0 != i;) {
    i--;
    DTSD( Serial.print("# ");  Serial.print(i, HEX); Serial.print(" => "); SerialPrintln_P((char*)pgm_read_word(&(commands[i]))); )
    if (0 == strcmp_P(_dst, (char*)pgm_read_word(&(commands[i])))) {cmdIdx = i; break;}
/*
    DTSD( Serial.print(">> # ");  Serial.print(i, HEX); Serial.print(" => "); SerialPrint_P((char*)pgm_read_word(&(commands[i].name))); )
    DTSD( Serial.print(" >> "); Serial.println(pgm_read_byte(&(commands[i].idx))); )
    if (0 == strcmp_P(_dst, (char*)pgm_read_word(&(commands[i].name)))) {cmdIdx = pgm_read_byte(&(commands[i].idx)); break;}
*/    
  }

  // Do nothing if command not found and function not implemented
  if (0 < cmdIdx) {
     //DTSM( Serial.print("[1] "); Serial.println(millis()); )
     DTSM( SerialPrint_P(PSTR("Execute command #")); Serial.print(cmdIdx, HEX); SerialPrint_P(PSTR(" => `")); Serial.print(_dst); Serial.println("`"); )

     // batch convert args to number values
     // first _argOffset item have index 0
     for (i = constArgC; 0 != i;) {
         i--;
         argv[i] = ('\0' == _dst[_argOffset[i]]) ? 0 : strtoul(&_dst[_argOffset[i]], NULL,0);
         DTSH( 
               SerialPrint_P(PSTR("argv[")); Serial.print(i); SerialPrint_P(PSTR("] => \"")); 
               if ('\0' == _dst[_argOffset[i]]) {
                  SerialPrint_P(PSTR("<null>")); 
               } else {
                  Serial.print(&_dst[_argOffset[i]]); 
               }
               SerialPrint_P(PSTR("\" => ")); Serial.print(argv[i]);
               SerialPrint_P(PSTR(", offset =")); Serial.println(_argOffset[i]);
         )
     }

     //DTSM( Serial.print("[2] "); Serial.println(millis()); )

     // Check rights for password protected commands
     accessGranted = (!netConfig.useProtection || argv[0] == netConfig.password); 

     i2CAddress = (uint8_t) argv[2];
     i2CRegister = (('\0' != _dst[_argOffset[3]]) ? (int16_t) argv[3] : I2C_NO_REG_SPECIFIED);
     // i2COption can be used as 'length', 'bitNumber' or 'data' variable
     i2COption = (uint8_t) argv[4];

     //DTSM( Serial.print("[3] "); Serial.println(millis()); )
   }

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
      strcpy(_dst, netConfig.hostname);
      result = RESULT_IN_BUFFER;
      break;
         
    case CMD_ZBX_AGENT_VERSION:
      /*/
      /=/  agent.version
      /*/
      strcpy_P(_dst, PSTR(ZBX_AGENT_VERISON));
      result = RESULT_IN_BUFFER;
      break;

    case CMD_SYSTEM_RUN:
      /*/
      /=/  system.run[newCommand
      /*/
      if ('\0' == _dst[_argOffset[0]]) { break; }
      // take length of 0-th arg + 1 byte for '\0'
      i = (_argOffset[1] - _argOffset[0]) + 1;
      // move it to begin of buffer to using as new incoming command
      // Note: ~8bytes can be saved with copying bytes in while() cycle. But source code will not beauty
      memmove(_dst, &_dst[_argOffset[0]], i);
      _dst[i] = '\n';
      return RUN_NEW_COMMAND;
      break;

    case CMD_SYS_UPTIME:
      /*/
      /=/  sys.uptime
      /*/
      value.ulongvar  = (uint32_t) millis() / 1000;
      result = RESULT_IN_ULONGVAR;
      break;

    case CMD_SYS_NET_REINITS:
      /*/
      /=/  sys.net.reinits
      /*/
      value.ulongvar  = (uint32_t) sysMetrics[IDX_METRIC_SYS_NET_REINITS];
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
      if ('\0' != _dst[_argOffset[1]]) {
         analogReference(argv[1]);
         delayMicroseconds(2000);
      }
#endif
      if (! isSafePin(argv[0])) { break; }     
      value.longvar = (int64_t) analogRead(argv[0]);
      if ('\0' != _dst[_argOffset[2]] && '\0' != _dst[_argOffset[3]]) {
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
      result = ('\0' == _dst[_argOffset[2]]) ? isSafePin(argv[0]) : (isSafePin(argv[0]) && isSafePin(argv[2]));
      if (!result) { break; }
      // turn on or turn off logic on pin
      digitalWrite(argv[0], argv[1]);
      result = RESULT_IS_OK; 
      if ('\0' == _dst[_argOffset[2]]) { break; }
      // when testPin defined - switch testPin mode to input, wait a lot, and check testPin state.
      // if readed value not equal testValue - return FAIL
      pinMode(argv[2], INPUT_PULLUP);
      delay(10);
      if ((uint32_t) digitalRead(argv[2]) != argv[3]){ result = RESULT_IS_FAIL; }
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
      if ('\0' != _dst[_argOffset[2]]) { tone(argv[0], argv[1], argv[2]); break;} 
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
      value.ulongvar = (int64_t) ('\0' == _dst[_argOffset[1]]) ? (int32_t) random(argv[0]) : (int32_t) random(argv[0], argv[1]);
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
      // _dst[_argOffset[1]] != \0 if argument #2 given
      if ('0' == _dst[_argOffset[1]]) { break; }
      
      // copy <1-th arg length> bytes from 1-th arg of buffer (0-th arg contain password) to hostname 
      i = (uint8_t) _argOffset[2]-_argOffset[1];
      if (i > constAgentHostnameMaxLength) { i = constAgentHostnameMaxLength; }
      //copy 0 .. (constAgentHostnameMaxLength-1) chars from buffer to hostname
      memcpy(netConfig.hostname, &_dst[_argOffset[1]], i);
      // Terminate string
      netConfig.hostname[constAgentHostnameMaxLength]='\0';
      saveConfigToEEPROM(&netConfig);
      result = RESULT_IS_OK;
      break;
  
    case CMD_SET_PASSWORD:
      /*/
      /=/  set.password[oldPassword, newPassword]
      /*/
      if (!accessGranted) { break; }
      if ('\0' == _dst[_argOffset[1]]) { break; }
      // take new password from argument #2
      netConfig.password = argv[1];
      saveConfigToEEPROM(&netConfig);
      result = RESULT_IS_OK;
      break;
  
    case CMD_SET_SYSPROTECT:
      /*/
      /=/  set.sysprotect[password, protection]
      /*/
      if (!accessGranted) { break; }
      if ('\0' == _dst[_argOffset[1]]) { break; }
      netConfig.useProtection = (1 == argv[1]) ? true : false;
      saveConfigToEEPROM(&netConfig);
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
      // argv[0] data contain in _dst[_argOffset[1]] placed from _argOffset[0]
      netConfig.useDHCP = (uint8_t) argv[1];
      // ip, netmask and gateway have one structure - 4 byte
      // take 6 bytes from second argument of command and use as new MAC-address
      // if convertation is failed (return false) succes variable must be falsed too via logic & operator
      success &= hstoba((uint8_t*) mac, &_dst[_argOffset[2]], arraySize(netConfig.macAddress));
      memcpy(netConfig.macAddress, &mac, arraySize(netConfig.macAddress));

      // We need to make sure that 'success' is true before hstoba() calling, but seems that any testing here just get progspace without profit
      // success = success && hstoba(..) <- can inline test 'success' and call hstoba(..) only on 'true' value
      
      // use 4 bytes from third argument of command as new IP-address. sizeof(IPAddress) returns 6 instead 4
      success &= hstoba((uint8_t*) &ip, &_dst[_argOffset[3]], 4);
      netConfig.ipAddress = IPAddress(ip);
  
      // take 4 bytes from third argument of command an use as new IP Netmask
      success &= hstoba((uint8_t*) &ip, &_dst[_argOffset[4]], 4);
      netConfig.ipNetmask = IPAddress(ip);
  
      // convert 4 bytes from fourth argument to default gateway
      success &= hstoba((uint8_t*) &ip, &_dst[_argOffset[5]], 4);
      netConfig.ipGateway = IPAddress(ip);
 
      if (!success) { break; }
      // Save config to EEProm if success
      result = saveConfigToEEPROM(&netConfig) ? RESULT_IS_OK : DEVICE_ERROR_EEPROM_CORRUPTED;
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
         shiftOutAdvanced(argv[0], argv[1], argv[3], &_dst[_argOffset[4]]);
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
         WS2812Out(argv[0], &_dst[_argOffset[1]]);
         result = RESULT_IS_OK;
      }
      break;
#endif // FEATURE_WS2812_ENABLE

    case CMD_SYS_REBOOT:
      /*/
      /=/  reboot[password]
      /*/
      if (! accessGranted) { break; }
      if (Transport.client.connected()) { Transport.client.println(1); }
      // hang-up if no delay
      delay(constNetStabilizationDelay);
      Transport.client.stop();
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
      strcpy_P(_dst, PSTR(_AVR_CPU_NAME_));
      result = RESULT_IN_BUFFER;
      break;
   
    case CMD_SYS_MCU_ID:
      /*/
      /=/  sys.mcu.id
      /*/
      // Read 10 bytes with step 1 (0x0E..0x17) of the signature row <= http://www.avrfreaks.net/forum/unique-id-atmega328pb
      getBootSignatureBytes(_dst, 0x0E, 10, 1);
      result = RESULT_IN_BUFFER;
      break;

    case CMD_SYS_MCU_SIGN:
      /*/
      /=/  sys.mcu.sign
      /*/
      // Read 3 bytes with step 2 (0x00, 0x02, 0x04) of the signature row <= http://www.avrfreaks.net/forum/device-signatures
      getBootSignatureBytes(_dst, 0x00, 3, 2);
      result = RESULT_IN_BUFFER;
      break;

    case CMD_SYS_NET_MODULE:
      /*/
      /=/  sys.net.module
      /*/
      strcpy_P(_dst, PSTR(NET_MODULE_NAME));
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
      if (_dst[_argOffset[0]]) { sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = 0; sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N] = 0; } 
      value.ulongvar = (uint32_t) sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX];
      result = RESULT_IN_ULONGVAR;
      break;

    case CMD_SYS_CMD_TIMEMAX_N:
      /*/
      /=/  sys.cmd.timemax.n
      /*/
      value.ulongvar = (uint32_t) sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N];
      Transport.client.println(value.ulongvar, HEX);
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
      result = scanOneWire(argv[0], &Transport.client);
      break;
#endif // FEATURE_ONEWIRE_ENABLE

#ifdef FEATURE_I2C_ENABLE
    case CMD_I2C_SCAN:
      /*/
      /=/  I2C.scan[sdaPin, sclPin]
      /*/
      if (! isSafePin(argv[0]) || ! isSafePin(argv[1])) { break;}
      result = scanI2C(&Transport.client);
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
      if ('\0' == _dst[_argOffset[5]]) { 
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
        delayMicroseconds(constAdcStabilizationDelay);
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
      result = getDS18X20Metric(argv[0], argv[1], &_dst[_argOffset[2]], _dst);
      break;
#endif // FEATURE_DS18X20_ENABLE

#ifdef FEATURE_DHT_ENABLE
    case CMD_DHT_HUMIDITY:
      /*/
      /=/  DHT.humidity[pin, model]
      /*/
      if (! isSafePin(argv[0])) { break; }
      result = getDHTMetric(argv[0], argv[1], SENS_READ_HUMD, _dst);
      break;

    case CMD_DHT_TEMPERATURE:
      /*/
      /=/  DHT.temperature[pin, model]
      /*/
      if (! isSafePin(argv[0])) { break; }
      result = getDHTMetric(argv[0], argv[1], SENS_READ_TEMP, _dst);
      break;
   
#endif // FEATURE_DHT_ENABLE
       
#ifdef FEATURE_BMP_ENABLE
    case CMD_BMP_PRESSURE:
      /*/
      /=/  BMP.Pressure[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getBMPMetric(argv[0], argv[1], i2CAddress, argv[3], argv[4], SENS_READ_PRSS, _dst);
      break;

    case CMD_BMP_TEMPERATURE:
      /*/
      /=/ BMP.Temperature[sdaPin, sclPin, i2cAddress, overSampling]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getBMPMetric(argv[0], argv[1], i2CAddress, argv[3], argv[4], SENS_READ_TEMP, _dst);
      break;
      
#ifdef SUPPORT_BME280_INCLUDE
      case CMD_BME_HUMIDITY:
      /*/
      /=/  BME.Humidity[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getBMPMetric(argv[0], argv[1], i2CAddress, argv[3], argv[4], SENS_READ_HUMD, _dst);
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
      result = getBH1750Metric(argv[0], argv[1], i2CAddress, argv[3], SENS_READ_LUX, _dst);
      break;
#endif // FEATURE_BH1750_ENABLE

#ifdef FEATURE_MAX7219_ENABLE
    case CMD_MAX7219_WRITE:
      /*/
      /=/  MAX7219.write[dataPin, clockPin, loadPin, intensity, data]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1]) || !isSafePin(argv[2])) { break; }
      writeToMAX7219(argv[0], argv[1], argv[2], argv[3], &_dst[_argOffset[4]]);
      result = RESULT_IS_OK;
      break;
#endif // FEATURE_MAX7219_ENABLE

#ifdef FEATURE_PCF8574_LCD_ENABLE
    case CMD_PCF8574_LCDPRINT:
      /*/
      /=/  PCF8574.LCDPrint[sdaPin, sclPin, i2cAddress, lcdBacklight, lcdType, data]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = printToPCF8574LCD(argv[0], argv[1], i2CAddress, argv[3], argv[4], &_dst[_argOffset[5]]);
      break;

#endif // FEATURE_PCF8574_LCD_ENABLE

#ifdef FEATURE_SHT2X_ENABLE
    case CMD_SHT2X_HUMIDITY:
      /*/
      /=/  SHT2X.Humidity[sdaPin, sclPin, i2cAddress]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getSHT2XMetric(argv[0], argv[1], i2CAddress, SENS_READ_HUMD, _dst);
      break;

    case CMD_SHT2X_TEMPERATURE:
      /*/
      /=/  SHT2X.Temperature[sdaPin, sclPin, i2cAddress]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      // (uint8_t) argv[2] is i2c address, 7 bytes size
      result = getSHT2XMetric(argv[0], argv[1], i2CAddress, SENS_READ_TEMP, _dst);
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
       if ('\0' != _dst[_argOffset[1]]) {
          argv[1] = DEFAULT;
       }
       result = getACS7XXMetric(argv[0], argv[1], SENS_READ_ZC, 0, 0, _dst);
      break;

    case CMD_ACS7XX_AC:
      /*/
      /=/  acs7xx.ac[sensorPin, refVoltage, sensitivity, zeroPoint] 
      /*/
      if (!isSafePin(argv[0])) { break; }
      // if refVoltage skipped - use DEFAULT source
      if ('\0' != _dst[_argOffset[1]]) {
         argv[1] = DEFAULT;
      }
      result = getACS7XXMetric(argv[0], argv[1], SENS_READ_AC, argv[2], (int32_t) argv[3], _dst);
      break;

    case CMD_ACS7XX_DC:
      /*/
      /=/  acs7xx.dc[sensorPin, refVoltage, sensitivity, zeroPoint] 
      /*/
      if (!isSafePin(argv[0])) { break; }
         // if refVoltage skipped - use DEFAULT source
      if ('\0' != _dst[_argOffset[1]]) {
         argv[1] = DEFAULT;
      }
      result = getACS7XXMetric(argv[0], argv[1], SENS_READ_DC, argv[2], (int32_t) argv[3], _dst);
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
      result = sendRawByIR(argv[1], argv[2], &_dst[_argOffset[3]]);
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
      // _dst cast to (uint8_t*) to use with subroutine math and SoftwareSerial subs, because used instead sub's internal buffer and save a little RAM size.
      // Its will be casted to char* inside at moment when its need
      result = getPZEM004Metric(argv[0], argv[1], SENS_READ_AC, &_dst[_argOffset[2]], (uint8_t*) _dst);
      break;
    case CMD_PZEM004_VOLTAGE:
      /*/
      /=/  pzem004.voltage[rxPin, txPin, ip]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getPZEM004Metric(argv[0], argv[1], SENS_READ_VOLTAGE, &_dst[_argOffset[2]], (uint8_t*) _dst);
      break;
    case CMD_PZEM004_POWER:
      /*/
      /=/  pzem004.power[rxPin, txPin, ip]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getPZEM004Metric(argv[0], argv[1], SENS_READ_POWER, &_dst[_argOffset[2]], (uint8_t*) _dst);
      break;
    case CMD_PZEM004_ENERGY:
      /*/
      /=/  pzem004.energy[rxPin, txPin, ip]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getPZEM004Metric(argv[0], argv[1], SENS_READ_ENERGY, &_dst[_argOffset[2]], (uint8_t*) _dst);
      break;
#endif // FEATURE_PZEM004_ENABLE

#ifdef FEATURE_UPS_APCSMART_ENABLE

    case CMD_UPS_APCSMART:
      /*/
      /=/  ups.apcsmart[rxPin, txPin, command]
      /=/    command - HEX or ASCII
      /*/  
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getAPCSmartUPSMetric(argv[0], argv[1], (uint8_t*) &_dst[_argOffset[2]], (_argOffset[3] - _argOffset[2]) , (uint8_t*) _dst);
      break;
#endif // FEATURE_UPS_APCSMART_ENABLE

#ifdef FEATURE_UPS_MEGATEC_ENABLE
    case CMD_UPS_MEGATEC:
      /*/
      /=/  ups.megatec[rxPin, txPin, command, fieldNumber]
      /=/    command - HEX or ASCII
      /*/  
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getMegatecUPSMetric(argv[0], argv[1], (uint8_t*) &_dst[_argOffset[2]], argv[3], (uint8_t*) _dst);
      break;
#endif // FEATURE_UPS_APCSMART_ENABLE

#ifdef FEATURE_INA219_ENABLE
    case CMD_INA219_BUSVOLTAGE:
      /*/
      /=/  INA219.BusVoltage[sdaPin, sclPin, i2cAddress, maxVoltage, maxCurrent]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getINA219Metric(argv[0], argv[1], i2CAddress, SENS_READ_BUS_VOLTAGE, argv[3], argv[4], _dst);
      break;

    case CMD_INA219_CURRENT:
      /*/
      /=/  INA219.Current[sdaPin, sclPin, i2cAddress, maxVoltage, maxCurrent]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getINA219Metric(argv[0], argv[1], i2CAddress, SENS_READ_DC, argv[3], argv[4], _dst);
      break;

    case CMD_INA219_POWER:
      /*/
      /=/  INA219.Power[sdaPin, sclPin, i2cAddress, maxVoltage, maxCurrent]
      /*/
      if (!isSafePin(argv[0]) || !isSafePin(argv[1])) { break; }
      result = getINA219Metric(argv[0], argv[1], i2CAddress, SENS_READ_POWER, argv[3], argv[4], _dst);
      break;

#endif // FEATURE_INA219_ENABLE


    default:
      // Early increased command counter is decremented
      sysMetrics[IDX_METRIC_SYS_CMD_COUNT]--;
      // In default case command  is considered unknown.
      strcpy_P(_dst, PSTR((MSG_ZBX_NOTSUPPORTED)));
      result = RESULT_IN_BUFFER;
      break;
   }

   // Form the output buffer routine
   // DTS( Serial.print("[4] "); Serial.println(millis()); )
   // The result is not printed or already placed in the buffer
   if (RESULT_IS_PRINTED != result) {
      switch (result) {
         case RESULT_IN_BUFFER:
            break;
         case RESULT_IS_OK:
            //  '1' must be returned
            _dst[0] = '1';
            _dst[1] = '\0';
            break;
         case RESULT_IS_FAIL:
            // or '0'
            _dst[0] = '0';
            _dst[1] = '\0';
            break;
         case RESULT_IN_LONGVAR:
            //  or result value placed in 'value' variable and must be converted to C-string.
            ltoa(value.longvar, _dst, 10);
            break;
         case RESULT_IN_ULONGVAR:
            //  or result value placed in 'value' variable and must be converted to C-string.
            ultoa(value.ulongvar, _dst, 10);
            break;
         case DEVICE_ERROR_CONNECT:
            strcpy_P(_dst, PSTR((MSG_DEVICE_ERROR_CONNECT)));
            break;
         case DEVICE_ERROR_ACK_L:
            strcpy_P(_dst, PSTR((MSG_DEVICE_ERROR_ACK_L)));
            break;
         case DEVICE_ERROR_ACK_H:
            strcpy_P(_dst, PSTR((MSG_DEVICE_ERROR_ACK_H)));
            break;
         case DEVICE_ERROR_CHECKSUM:
            strcpy_P(_dst, PSTR((MSG_DEVICE_ERROR_CHECKSUM)));
            break;
         case DEVICE_ERROR_TIMEOUT:
            strcpy_P(_dst, PSTR((MSG_DEVICE_ERROR_TIMEOUT)));
            break;
         case DEVICE_ERROR_WRONG_ID:
            strcpy_P(_dst, PSTR((MSG_DEVICE_ERROR_WRONG_ID)));
            break;
         case DEVICE_ERROR_NOT_SUPPORTED:
            strcpy_P(_dst, PSTR((MSG_DEVICE_ERROR_NOT_SUPPORTED)));
            break;
         case DEVICE_ERROR_WRONG_ANSWER:
            strcpy_P(_dst, PSTR((MSG_DEVICE_ERROR_WRONG_ANSWER)));
            break;
         case DEVICE_ERROR_EEPROM_CORRUPTED:
            strcpy_P(_dst, PSTR((MSG_DEVICE_ERROR_EEPROM)));
            break;
         default:
            // otherwise subroutine return unexpected value, need to check its source code
            strcpy_P(_dst, PSTR("Unexpected retcode"));
            break;
      }
      //  Push out the buffer to the client
      if (Transport.client.connected()) { Transport.client.println(_dst); }
   }
   DTSL( Serial.println(_dst); )
//   DTSM( Serial.print("[5] "); Serial.println(millis()); )
   return cmdIdx;
}

