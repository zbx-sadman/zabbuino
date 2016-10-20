  
// My Freeduino is not listed in platforms.h, but is analogue to ARDUINO_AVR_DUEMILANOVE
#define ARDUINO_AVR_DUEMILANOVE

// Just for compilation with various default network configs, real address/gw/et switching is making in "zabbuino.h" file
//#define USE_NETWORK_192_168_0_0

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 PROGRAMM FEATURES SECTION
                   
   Please refer to the "zabbuino.h" file for enabling or disabling Zabbuino's features and "src/defaults.h" to deep tuning.
   if connected sensors seems not work - first check setting in port_protect[], port_mode[], port_pullup[] arrays in I/O PORTS SETTING SECTION of "src/defaults.h" file
*/
#include "zabbuino.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 GLOBAL VARIABLES SECTION
*/

netconfig_t* netConfig;

#if defined(FEATURE_EXTERNAL_INTERRUPT_ENABLE) || defined(FEATURE_INCREMENTAL_ENCODER_ENABLE)
// EXTERNAL_NUM_INTERRUPTS its a macro from <wiring_private.h>
volatile extInterrupt_t* extInterrupt;
#endif


EthernetServer ethServer(10050);
EthernetClient ethClient;

char cBuffer[BUFFER_SIZE+1]; // +1 for trailing \0
int32_t *sysMetrics;


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                      STARTUP SECTION
*/

void setup() {
  uint8_t i;
  netConfig = new netconfig_t;

  pinMode(PIN_FACTORY_RESET, INPUT_PULLUP);
  pinMode(PIN_STATE_LED, OUTPUT);

#ifdef ADVANCED_BLINKING
  // blink on start
  blinkMore(6, 50, 500);
#endif

  // Init metrics
  sysMetrics = new int32_t[IDX_METRICS_MAX];
  sysMetrics[IDX_METRIC_SYS_VCCMIN] = sysMetrics[IDX_METRIC_SYS_VCCMAX] = getADCVoltage(ANALOG_CHAN_VBG);
  sysMetrics[IDX_METRIC_SYS_RAM_FREE] = sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] = (int32_t) getRamFree();
  sysMetrics[IDX_METRIC_SYS_CMD_COUNT] = sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = 0;
  
#if defined (FEATURE_DEBUG_TO_SERIAL) || defined (FEATURE_SERIAL_LISTEN_TOO)
  Serial.begin(9600);
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrintln_P(PSTR("Zabbuino waked up"));
#endif
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

#endif

#ifdef FEATURE_EEPROM_ENABLE

/* -=-=-=-=-=-=-=-=-=-=-=-
    FACTORY RESET BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */

  // Set mode of PIN_FACTORY_RESET and turn on internal pull resistor
  digitalWrite(PIN_STATE_LED, LOW);
  // Check for PIN_FACTORY_RESET shorting to ground?
  // (when pulled INPUT pin shorted to GND - digitalRead() return LOW)
  if (LOW == digitalRead(PIN_FACTORY_RESET)){
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrintln_P(PSTR("The factory reset button is pressed"));
#endif
    // Fire up state LED
    digitalWrite(PIN_STATE_LED, HIGH);
    // Wait some msecs
    delay(HOLD_TIME_TO_FACTORY_RESET);
    // PIN_FACTORY_RESET still shorted?
    if (LOW == digitalRead(PIN_FACTORY_RESET)){
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrintln_P(PSTR("Rewrite EEPROM with defaults..."));
#endif
       setConfigDefaults(netConfig);
       saveConfigToEEPROM(netConfig);
       // Blink fast while PIN_FACTORY_RESET shorted to GND
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrintln_P(PSTR("Done. Release the factory reset button now"));
#endif
       while (LOW == digitalRead(PIN_FACTORY_RESET)) {
          digitalWrite(PIN_STATE_LED, millis() % 100 < 50);
      }
    }
    digitalWrite(PIN_STATE_LED, LOW);
  } // if (LOW == digitalRead(PIN_FACTORY_RESET))

/* -=-=-=-=-=-=-=-=-=-=-=-
    CONFIGURATION LOAD BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */
#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrintln_P(PSTR("Load configuration from EEPROM"));
#endif
  // Try to load configuration from EEPROM
  if (false == loadConfigFromEEPROM(netConfig)) {
     // bad CRC detected, use default values for this run
     setConfigDefaults(netConfig);
     saveConfigToEEPROM(netConfig);
  }
#else // FEATURE_EEPROM_ENABLE
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrintln_P(PSTR("Use default network settings"));
#endif
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
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrintln_P(PSTR("Obtaining address from DHCP..."));
#endif
       // Try to ask DHCP server
      if (0 == Ethernet.begin(netConfig->macAddress)) {
#ifdef FEATURE_DEBUG_TO_SERIAL
         SerialPrintln_P(PSTR("No success"));
#endif
         // No offer recieved - switch off DHCP feature for that session
         netConfig->useDHCP = false;
      }
  }
#else // FEATURE_NET_DHCP_ENABLE
  netConfig->useDHCP=false;
#endif // FEATURE_NET_DHCP_ENABLE

  // No DHCP offer recieved or no DHCP need - start with stored/default IP config
  if (false == netConfig->useDHCP) {
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrintln_P(PSTR("Use static IP"));
#endif
     // That overloaded .begin() function return nothing
     // Second netConfig->ipAddress used as dns-address
     Ethernet.begin(netConfig->macAddress, netConfig->ipAddress, netConfig->ipAddress, netConfig->ipGateway, netConfig->ipNetmask);
  }
  
#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrintln_P(PSTR("Serving on:"));
  SerialPrint_P(PSTR("MAC     : ")); printArray(netConfig->macAddress, sizeof(netConfig->macAddress), DBG_PRINT_AS_MAC);
  SerialPrint_P(PSTR("Hostname: ")); Serial.println(netConfig->hostname);
  SerialPrint_P(PSTR("IP      : ")); Serial.println(Ethernet.localIP());
  SerialPrint_P(PSTR("Subnet  : ")); Serial.println(Ethernet.subnetMask());
  SerialPrint_P(PSTR("Gateway : ")); Serial.println(Ethernet.gatewayIP());
  SerialPrint_P(PSTR("Password: ")); Serial.println(netConfig->password, DEC);
  // This codeblock is compiled if UIPethernet.h is included
#ifdef UIPETHERNET_H
  SerialPrint_P(PSTR("ENC28J60: rev ")); Serial.println(Enc28J60.getrev());
#endif

  //netConfig->ipGateway.printTo(Serial);
#endif

  // Start listen sockets
  ethServer.begin();

/* -=-=-=-=-=-=-=-=-=-=-=-
    OTHER STUFF INIT BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */

  // I/O ports initialization. Refer to "I/O PORTS SETTING SECTION" in zabbuino.h
  for (i = 0; i < PORTS_NUM; i++) { 
    setPortMode(i, port_mode[i], port_pullup[i]);
  }

#if defined(FEATURE_EXTERNAL_INTERRUPT_ENABLE) || defined(FEATURE_INCREMENTAL_ENCODER_ENABLE)
  extInterrupt = new extInterrupt_t[EXTERNAL_NUM_INTERRUPTS];
  // Init external interrupts info structure
  for (i = 0; i < EXTERNAL_NUM_INTERRUPTS; i++) { 
    // -1 - interrupt is detached
    // just use NOT_AN_INTERRUPT = -1 macro from Arduino.h
    extInterrupt[i].mode = NOT_AN_INTERRUPT;
    extInterrupt[i].count = 0;
  }
#endif

  // Uncomment to force protect (enable even useProtection is false) your system from illegal access for change runtime settings and reboots 
#ifdef FEATURE_PASSWORD_PROTECTION_FORCE
  netConfig->useProtection = true;
#endif

#ifdef FEATURE_WATCHDOG_ENABLE
  // Watchdog activation
  wdt_enable(WTD_TIMEOUT);
#endif

#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
   // need to analyze return code?
   initTimerOne(SYS_METRIC_RENEW_PERIOD);
#endif

#if defined(FEATURE_I2C_ENABLE) || defined(FEATURE_BMP_ENABLE) || defined(FEATURE_BH1750_ENABLE) || defined (FEATURE_PCF8574_LCD_ENABLE) || defined (FEATURE_SHT2X_ENABLE)
   Wire.begin();
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
  uint8_t result;
  uint16_t blinkType = BLINK_NOPE;
  int16_t *_argOffset;
  uint32_t nowTime, processStartTime, processEndTime, prevDHCPRenewTime, prevENCReInitTime, prevNetProblemTime, prevSysMetricGatherTime,clentConnectTime;

  _argOffset = new int16_t[ARGS_MAX];
    
  // Correcting timestamps
  prevDHCPRenewTime = prevENCReInitTime = prevNetProblemTime = prevSysMetricGatherTime = millis();

  // if no exist while() here - netProblemTime must be global or static - its will be 0 every loop() and time-related cases will be processeed abnormally
  // ...and while save some cpu ticks because do not call everytime from "hidden main()" subroutine, and do not init var, and so.
  while (true) { 
    nowTime = millis();

      // correctVCCMetrics() must be always inline compiled
    
    // Gather internal metrics periodically
    if (SYS_METRIC_RENEW_PERIOD <= (uint32_t) (nowTime - prevSysMetricGatherTime)) { 
#ifndef GATHER_METRIC_USING_TIMER_INTERRUPT
       gatherSystemMetrics();
#endif 
       sysMetrics[IDX_METRIC_SYS_VCC] = getADCVoltage(ANALOG_CHAN_VBG);
       correctVCCMetrics(sysMetrics[IDX_METRIC_SYS_VCC]);
       prevSysMetricGatherTime = millis();
    }

#ifdef FEATURE_NET_DHCP_ENABLE
    // DHCP used in this session and time to renew lease?
      // how many overhead give Ethernet.maintain() ?
    if (true == netConfig->useDHCP && (NET_DHCP_RENEW_PERIOD <= (uint32_t) (nowTime - prevDHCPRenewTime))) {
       // Ethernet library's manual say that Ethernet.maintain() can be called every loop for DHCP renew, but i won't do this so often
       result = Ethernet.maintain();
       // Renew procedure finished with success
       if (DHCP_CHECK_RENEW_OK == result || DHCP_CHECK_REBIND_OK  == result) { 
          // No alarm blink  need, network activity registred, renewal period restarted
          blinkType = BLINK_NOPE;
//          prevDHCPRenewTime = prevNetProblemTime = nowTime;
       } else {
          // Got some errors - blink with "DHCP problem message"
          blinkType = BLINK_DHCP_PROBLEM;
#ifdef FEATURE_DEBUG_TO_SERIAL
//            SerialPrintln_P(PSTR("DHCP renew problem occured"));
#endif 
       }
//    }
#endif // FEATURE_NET_DHCP_ENABLE

    // No DHCP problem found but no data recieved or network activity for a long time
    if (BLINK_NOPE == blinkType && (NET_IDLE_TIMEOUT <= (uint32_t) (nowTime - prevNetProblemTime))) { 
#ifdef FEATURE_DEBUG_TO_SERIAL
//            SerialPrintln_P(PSTR("No data recieved for a long time"));
#endif 
       blinkType = BLINK_NET_PROBLEM; 
    }

#ifdef USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE
    // Time to reinit ENC28J60?
    if (NET_ENC28J60_REINIT_PERIOD <= (uint32_t) (nowTime - prevENCReInitTime)) {
       // if EIR.TXERIF or EIR.RXERIF is set - ENC28J60 detect error, re-init module 
       if (Enc28J60.readReg(EIR) & (EIR_TXERIF | EIR_RXERIF)) {
#ifdef FEATURE_DEBUG_TO_SERIAL
          SerialPrintln_P(PSTR("ENC28J60 reinit"));
#endif
          Enc28J60.init(netConfig->macAddress); 
          delay(NET_STABILIZATION_DELAY);
       } 
         prevENCReInitTime = nowTime;

    }
#endif // USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE

    // No errors in the loop?
    if (BLINK_NOPE == blinkType) {
       // Switch off state led
       digitalWrite(PIN_STATE_LED, LOW);
    } else {
      // Error caused
#ifdef ON_ALARM_STATE_BLINK
      // Do LED blink...
      digitalWrite(PIN_STATE_LED, nowTime % 1000 < blinkType);
#else
      // ...or just fired up
      digitalWrite(PIN_STATE_LED, HIGH);
#endif
    }

#ifdef FEATURE_SERIAL_LISTEN_TOO
    // Test state of active session: client still connected or unread data is exist in buffer?
    if (ethClient.connected() || Serial.available()) {
       if ((NET_ACTIVE_CLIENT_CONNECTION_TIMEOUT <= (uint32_t) (nowTime - clentConnectTime)) && !Serial.available())  {
           ethClient.stop(); 
       } else {
           // A lot of chars wait for reading
           if (ethClient.available() || Serial.available()) {
              // Do not need next char to analyze - EOL detected or there no room in buffer or max number or args parsed...   
              result = Serial.read();
              if (ethClient.available()) { result = ethClient.read(); }
#else
    // Test state of active session: client still connected or unread data is exist in buffer?
    if (ethClient.connected()) {
       if (NET_ACTIVE_CLIENT_CONNECTION_TIMEOUT <= (uint32_t) (nowTime - clentConnectTime)) {
           ethClient.stop(); 
       } else {
           // A lot of chars wait for reading
           if (ethClient.available()) {
              // Do not need next char to analyze - EOL detected or there no room in buffer or max number or args parsed...   
              result = ethClient.read();
#endif              

//              Serial.print("incoming: "); Serial.print((char) result); Serial.print(" => "); Serial.println(result, HEX); 
              result = analyzeStream((char) result, _argOffset);
              if (false == result) {
                 /*****  processing command *****/
                 // Destroy unused client's data 
                 ethClient.flush(); 
#ifdef FEATURE_SERIAL_LISTEN_TOO
                 Serial.flush(); 
#endif              
                 // Fire up State led, than will be turned off on next loop
                 digitalWrite(PIN_STATE_LED, HIGH);
                 //
                 // may be need test for client.connected()? 
                 processStartTime = millis();
                 int16_t cmdIdx = executeCommand(_argOffset);
                 // system.run[] recieved, need to run another command, which taken from option #0 by cmdIdx() sub
                 if (RUN_NEW_COMMAND == cmdIdx) {
                     int16_t i = 0;
                     // simulate command recievig to properly string parsing
                     while( analyzeStream(cBuffer[i], _argOffset) ) { i++; }
                     cmdIdx = executeCommand(_argOffset);
                 }
                 processEndTime = millis();
                 // use processEndTime as processDurationTime
                 processEndTime = processEndTime - processStartTime ;
                 if (sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] < processEndTime) {
                    sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = processEndTime;
                    sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N] = cmdIdx;
                 }
  //             ethClient.println("1");
                 // Wait some time to finishing answer send
                 delay(NET_STABILIZATION_DELAY);
                // close connection           
                ethClient.stop(); 
              }
              // Restart network activity control cycle
              prevENCReInitTime = prevNetProblemTime = millis();
              blinkType = BLINK_NOPE;
           }
        }
    } else {
       // Active session is not exist. Try to take new for processing.
       ethClient = ethServer.available();
       if (ethClient) {
         clentConnectTime = millis();
       }
    }
#ifdef FEATURE_WATCHDOG_ENABLE
    // reset watchdog every loop
    wdt_reset();
#endif
  }
}

/* ****************************************************************************************************************************
*
*  Stream analyzing subroutine
*  Detect Zabbix packets, on-fly spit incoming stream to command & arguments
*
**************************************************************************************************************************** */
uint8_t analyzeStream(char _charFromClient, int16_t* _argOffset) {
  uint8_t static needSkipZabbix2Header = 0, 
                 cmdSliceNumber        = 0,
                 isEscapedChar         = 0,
                 doubleQuotedString    = false;
  uint16_t static bufferWritePosition;

  // If there is not room in buffer - simulate EOL recieving
  if (BUFFER_SIZE <= bufferWritePosition ) { _charFromClient = '\n'; }
  
  // Put next char to buffer
  cBuffer[bufferWritePosition] = (doubleQuotedString) ? _charFromClient : tolower(_charFromClient); 
  //Serial.print("Char: "); Serial.println(cBuffer[bufferWritePosition]);
    
  // When ZBX_HEADER_PREFIX_LENGTH chars is saved to buffer - test its for Zabbix2 protocol header prefix ("ZBXD\01") presence
  if (ZBX_HEADER_PREFIX_LENGTH == bufferWritePosition) {
     if (0 == memcmp(&cBuffer, (ZBX_HEADER_PREFIX), ZBX_HEADER_PREFIX_LENGTH)) {
        // If packet have prefix - set 'skip whole header' flag
        needSkipZabbix2Header = true;
     }
  }

  // When ZBX_HEADER_LENGTH chars is saved to buffer - ckeck 'skip whole header' flag
  if (ZBX_HEADER_LENGTH == bufferWritePosition && needSkipZabbix2Header) {
     // If is setted - just begin write new data from begin of buffer. It's operation 'drop' Zabbix2 header
     bufferWritePosition = 0;
     needSkipZabbix2Header = false;
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
             // Doublequote is not escaped. 
             // Just drop it and toggle doublequoted mode
             // Doublequoted mode: do not convert char case, skip action on space, ']', '[', ','
             doubleQuotedString = !doubleQuotedString;
             // Jump out from subroutine to get next char from client
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
          if (doubleQuotedString) { break; }
          //   ...otherwise - process as 'EOL sign'
        case '\n':
          // EOL detected
          // Save last argIndex that pointed to <null> item. All unused _argOffset[] items must be pointed to this <null> item too.
          // Serial.println("[*3]");
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
        // clear vars for next round
        bufferWritePosition = cmdSliceNumber = isEscapedChar = doubleQuotedString = 0;
        needSkipZabbix2Header = false;
        // Return 'Do not need next char'
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
int16_t executeCommand(int16_t* _argOffset)
{
  int8_t result = RESULT_IS_FAIL;
  uint8_t accessGranted, i, i2CAddress, i2COption, i2CValue[4];
  int16_t i2CRegister, cmdIdx = -1;
  // duration option in the tone[] command is ulong
  uint32_t arg[ARGS_MAX];
  int64_t value = 0;  // Zabbix use 64-bit numbers, but we can use only -uint32_t...+uint32_t range. Error can be occurs on ltoa() call with value > long_int_max 
 
  sysMetrics[IDX_METRIC_SYS_CMD_COUNT]++;

  i = sizeof(commands);
  while (i) {
//    Serial.print(i, HEX); Serial.print(": ");
//    SerialPrintln_P((char*)pgm_read_word(&(commands[i])));
    if (0 == strcmp_P(cBuffer, (char*)pgm_read_word(&(commands[i])))) {cmdIdx = i; break;}
    i--;
  }
#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrint_P(PSTR("Execute command #")); Serial.print(cmdIdx, HEX); SerialPrint_P(PSTR(" => `")); Serial.print(cBuffer); Serial.println("`");
#endif 

  // first _argOffset item have index 0
  i = ARGS_MAX;
  // batch convert args to number values
  while (i) {
     i--;
     arg[i] = ('\0' == cBuffer[_argOffset[i]]) ? 0 : strtoul(&cBuffer[_argOffset[i]], NULL,0);

#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrint_P(PSTR("arg[")); Serial.print(i); SerialPrint_P(PSTR("] => \"")); 
     if ('\0' == cBuffer[_argOffset[i]]) {
        SerialPrint_P(PSTR("<null>")); 
     } else {
        Serial.print(&cBuffer[_argOffset[i]]); 
     }
     SerialPrint_P(PSTR("\" => ")); Serial.print(arg[i]);
     SerialPrint_P(PSTR(", offset =")); Serial.println(_argOffset[i]);
#endif 
  }
   
  // Check rights for password protected commands
  accessGranted = (!netConfig->useProtection || arg[0] == netConfig->password); 


  i2CAddress = (uint8_t) arg[2];
  i2CRegister = (('\0' != cBuffer[_argOffset[3]]) ? (int16_t) arg[3] : I2C_NO_REG_SPECIFIED);
  // i2COption can be length, bitNumber or data
  i2COption = (uint8_t) arg[4];

 
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
      if ('\0' == cBuffer[_argOffset[0]]) { break; }

      // take length of 0-th arg + 1 byte for '\0'
      i = (_argOffset[1] - _argOffset[0]) + 1;
      // move it to begin of buffer to using as new incoming command
      // Note: ~8bytes can be saved with copying bytes in while() cycle. But source code will not beauty
      memmove(cBuffer, &cBuffer[_argOffset[0]], i);
#ifdef FEATURE_DEBUG_TO_SERIAL
      SerialPrint_P(PSTR("Run new command: ")); Serial.println(cBuffer);
#endif
      cBuffer[i] = '\n';
      return RUN_NEW_COMMAND;
      break;

    case CMD_SYS_UPTIME:
      /*/
      /=/  sys.uptime
      /*/
      value = (int64_t) millis() / 1000;
      result = RESULT_IN_VARIABLE;
      break;

    case CMD_ARDUINO_ANALOGWRITE:
      /*/
      /=/  analogWrite[pin, value]
      /*/
      if (! isSafePin(arg[0])) { break; }

      analogWrite(arg[0], arg[1]);
      result = RESULT_IS_OK;
      break;
      
    case CMD_ARDUINO_ANALOGREAD:
      /*/
      /=/  analogRead[pin, analogReferenceSource, mapToLow, mapToHigh]
      /*/
#ifdef FEATURE_AREF_ENABLE
      // change source of the reference voltage if its given
      if ('\0' != cBuffer[_argOffset[1]]) {
         analogReference(arg[1]);
         delayMicroseconds(2000);
      }
#endif
 
      if (! isSafePin(arg[0])) { break; }
      
      value = (int64_t) analogRead(arg[0]);
      if ('\0' != cBuffer[_argOffset[2]] && '\0' != cBuffer[_argOffset[3]]) {
         value = (int64_t) map(result, 0, 1023, arg[2], arg[3]);
      }
      result = RESULT_IN_VARIABLE;

      break;      
  
#ifdef FEATURE_AREF_ENABLE
    case CMD_ARDUINO_ANALOGREFERENCE:
      /*/
      /=/  analogReference[source]
      /*/
      analogReference(arg[0]);
      result = RESULT_IS_OK;
      break;
#endif
  
    
    case CMD_ARDUINO_DELAY:
      /*/
      /=/  delay[time]
      /*/
      delay(arg[0]);
      result = RESULT_IS_OK;
      break;
      

    case CMD_ARDUINO_DIGITALWRITE:
      /*/
      /=/  digitalWrite[pin, value, testPin, testValue]
      /*/
      // if testPin defined - check both pin to safety
      result = ('\0' == cBuffer[_argOffset[2]]) ? isSafePin(arg[0]) : (isSafePin(arg[0]) && isSafePin(arg[2]));
      if (!result) { break; }

      // turn on or turn off logic on pin
      digitalWrite(arg[0], arg[1]);
      result = RESULT_IS_OK; 

      if ('\0' == cBuffer[_argOffset[2]]) { break; }
      // when testPin defined - switch testPin mode to input, wait a lot, and check testPin state.
      // if readed value not equal testValue - return FAIL
      pinMode(arg[2], INPUT_PULLUP);
      delay(10);
      if (digitalRead(arg[2]) != arg[3]){ result = RESULT_IS_FAIL; }

      break;

    case CMD_ARDUINO_DIGITALREAD:
      /*/
      /=/  digitalRead[pin]
      /*/
      value = (int64_t) digitalRead(arg[0]);
      result = RESULT_IN_VARIABLE;
      break;


#ifdef FEATURE_TONE_ENABLE
    case CMD_ARDUINO_TONE:
      /*/
      /=/  tone[pin, frequency, duration]
      /*/
      if (! isSafePin(arg[0])) { break; }

      result = RESULT_IS_OK;
      if ('\0' != cBuffer[_argOffset[2]]) { tone(arg[0], arg[1], arg[2]); break;} 
      tone(arg[0], arg[1]);
      break;
  
    case CMD_ARDUINO_NOTONE:
      /*/
      /*/
      if (! isSafePin(arg[0])) { break; }

      result = RESULT_IS_OK;
      noTone(arg[0]);
      break;
  
#endif

#ifdef FEATURE_RANDOM_ENABLE
    case CMD_ARDUINO_RANDOMSEED:
      /*/
      /=/  randomSeed[value]
      /*/
      randomSeed((0 == arg[0]) ? (int32_t) millis() : arg[0]);
      result = RESULT_IS_OK;
      break;
   
    case CMD_ARDUINO_RANDOM:
      /*/
      /=/  random[min, max]
      /*/
      //  !! random return long
      value = (int64_t) ('\0' == cBuffer[_argOffset[1]]) ? (int32_t) random(arg[0]) : (int32_t) random(arg[0], arg[1]);
      result = RESULT_IN_VARIABLE;

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
      memcpy(netConfig->hostname, &cBuffer[_argOffset[1]], i);
      // Terminate string
      netConfig->hostname[i]='\0';
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
      netConfig->password = arg[1];
      saveConfigToEEPROM(netConfig);
      result = RESULT_IS_OK;
      break;
  
    case CMD_SET_SYSPROTECT:
      /*/
      /=/  set.sysprotect[password, protection]
      /*/
      if (!accessGranted) { break; }
      if ('\0' == cBuffer[_argOffset[1]]) { break; }

      netConfig->useProtection = (1 == arg[1]) ? true : false;
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
      // arg[0] data contain in cBuffer[_argOffset[1]] placed from _argOffset[0]
      netConfig->useDHCP = (uint8_t) arg[1];
      // ip, netmask and gateway have one structure - 4 byte
      // take 6 bytes from second argument of command and use as new MAC-address
      // if convertation is failed (return false) succes variable must be falsed too via logic & operator
      success &= hstoba((uint8_t*) mac, &cBuffer[_argOffset[2]], arraySize(netConfig->macAddress));
      memcpy(netConfig->macAddress, &mac, arraySize(netConfig->macAddress));
      
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
      saveConfigToEEPROM(netConfig);
      result = RESULT_IS_OK;
      break;
#endif // FEATURE_EEPROM_ENABLE

    case CMD_SYS_PORTWRITE:
      /*/
      /=/  portWrite[port, value]
      /*/
      if (PORTS_NUM >= (arg[0] - 96)) { break; }

      writeToPort((byte) arg[0] - 96, arg[1]);
      result = RESULT_IS_OK;

      break;
  
#ifdef FEATURE_SHIFTOUT_ENABLE
    case CMD_SYS_SHIFTOUT:
      /*/
      /=/  shiftOut[dataPin, clockPin, latchPin, bitOrder, value]
      /*/
      // i used as latchPinDefined
      i = ('\0' != arg[2]) && isSafePin(arg[2]);   // << корректный способ проверки или нет?  
      if (isSafePin(arg[0]) &&  isSafePin(arg[1])) {
         if (i) { digitalWrite(arg[2], LOW); }
         shiftOutAdvanced(arg[0], arg[1], arg[3], &cBuffer[_argOffset[4]]);
         if (i) { digitalWrite(arg[2], HIGH);}
         result = RESULT_IS_OK;
      }
      break;
#endif

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
      // Read bytes 0x0E..0x17 from boot signature <= http://www.avrfreaks.net/forum/unique-id-atmega328pb
      getBootSignatureBytes(cBuffer, 0x0E, 10);
      result = RESULT_IN_BUFFER;
      break;

    case CMD_SYS_MCU_SIGN:
      /*/
      /=/  sys.mcu.sign
      /*/
      // Read bytes 0x00..0x03 from boot signature <= http://www.avrfreaks.net/forum/device-signatures
      getBootSignatureBytes(cBuffer, 0x00, 3);
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
      if (arg[0]) { sysMetrics[IDX_METRIC_SYS_CMD_COUNT] = 0; } 
      value = (int64_t) sysMetrics[IDX_METRIC_SYS_CMD_COUNT];
      result = RESULT_IN_VARIABLE;
      break;

    case CMD_SYS_CMD_TIMEMAX:
      /*/
      /=/  sys.cmd.timemax[resetCounter]
      /*/
      if (cBuffer[_argOffset[0]]) { sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = 0; sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N] = 0; } 
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = (int64_t) sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX];
      }
      result = RESULT_IN_VARIABLE;
      break;

    case CMD_SYS_CMD_TIMEMAX_N:
      /*/
      /=/  sys.cmd.timemax.n
      /*/
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = (int64_t) sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N];
      }
      ethClient.println((uint32_t) value, HEX);
      result = RESULT_IS_PRINTED;
      break;
   
    case CMD_SYS_RAM_FREE:
      /*/
      /=/  sys.ram.free
      /*/
      value = (int64_t) sysMetrics[IDX_METRIC_SYS_RAM_FREE];
      result = RESULT_IN_VARIABLE;
      break;

    case CMD_SYS_RAM_FREEMIN:
      /*/
      /=/  sys.ram.freemin
      /*/
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
         value = (int64_t) sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN];
      }
         result = RESULT_IN_VARIABLE;
      break;

#endif


    case CMD_SYS_VCC:
      /*/
      /=/ sys.vcc
      /*/
      // Take VCC
      value = (int64_t) getADCVoltage(ANALOG_CHAN_VBG);
      // VCC may be bigger than max or smaller than min. 
      // To avoid wrong results and graphs in monitoring system - correct min/max metrics
      correctVCCMetrics(value);
      result = RESULT_IN_VARIABLE;
      break;
  
    case CMD_SYS_VCCMIN:
      /*/
      /=/ sys.vccMin
      /*/
      value = (int64_t) sysMetrics[IDX_METRIC_SYS_VCCMIN];
      result = RESULT_IN_VARIABLE;
      break;
  
    case CMD_SYS_VCCMAX:
      /*/
      /=/ sys.vccMax
      /*/
      value = (int64_t) sysMetrics[IDX_METRIC_SYS_VCCMAX];
      result = RESULT_IN_VARIABLE;
      break;
  

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
    case CMD_EXTINT_COUNT:
      /*/
      /=/  extInt.count[intPin, intNumber, mode]
      /*/
      if (! isSafePin(arg[0])) { break; }
      value = (int64_t) manageExtInt(arg[0], arg[2]);
      result = RESULT_IN_VARIABLE;
      break;
#endif // FEATURE_EXTERNAL_INTERRUPT_ENABLE

#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE
    case CMD_ENCODER_COUNT:
      /*/
      /=/  incEnc.count[terminalAPin, terminalBPin, intNumber, initialValue]
      /*/
      if (! isSafePin(arg[0]) || ! isSafePin(arg[1]) { break; }
      // arg[3] (intNumber) currently not used
      value = (int64_t) manageIncEnc(arg[0], arg[1], arg[3]);
      result = RESULT_IN_VARIABLE;
      break;
#endif // FEATURE_INCREMENTAL_ENCODER_ENABLE
     


#ifdef FEATURE_OW_ENABLE
    case CMD_OW_SCAN:
      /*/
      /=/  OW.scan[pin]
      /*/
      if (! isSafePin(arg[0])) { break; }
      result = scanOneWire(arg[0], &ethClient);
      break;
#endif // FEATURE_ONEWIRE_ENABLE


#ifdef FEATURE_I2C_ENABLE
    case CMD_I2C_SCAN:
      /*/
      /=/  I2C.scan[sdaPin, sclPin]
      /*/
      if (! isSafePin(arg[0]) || ! isSafePin(arg[1])) { break;}
      result = scanI2C(&ethClient);
      break;

    case CMD_I2C_WRITE:
      /*/
      /=/ i2c.write(sdaPin, sclPin, i2cAddress, register, data, numBytes)
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      // i2COption used as 'data'
      arg[5] = constrain(arg[5], 1, 4);
      i = arg[5];
      while(i){
         i--;
         i2CValue[i] = arg[4] & 0xFF;
         arg[4] = arg[4] >> 8;
      }
      result = writeBytesToI2C(i2CAddress, i2CRegister, i2CValue, arg[5]);
      result = (0 == result) ? RESULT_IS_OK : RESULT_IS_FAIL;
      break;

    case CMD_I2C_READ:
      /*/
      /=/ i2c.read(sdaPin, sclPin, i2cAddress, register, length, doDoubleReading)
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }

      // i2COption used as 'length' - how much bytes must be read: 0..4 byte
      i2COption = constrain(i2COption, 1, 4);
      // numberOfReadings parameter is not specified. Do once reading only
      // Otherwise make ahead reading for re-run sensor conversion to flush old data
      if ('\0' == cBuffer[_argOffset[5]]) { 
          arg[5] = 1; 
      } else {
          // Just discard result
          readBytesFromi2C(i2CAddress, i2CRegister, i2CValue, i2COption);
      }
      // One reading at least must be done
      if (1 > arg[5]){ arg[5] = 1; }
         
      uint8_t readNumber;
      int32_t accResult, tmpResult;
      
      readNumber = arg[5]; accResult = 0;

      // Will be RESULT_IS_FAIL if readBytesFromi2C() not returns 0
      result = RESULT_IS_OK;
      while (readNumber) {
        readNumber--;
        // ADC stabilization delay
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
         value = (int64_t) (accResult / arg[5]);
         result = RESULT_IN_VARIABLE;
      }
      break;

    case CMD_I2C_BITWRITE:
      /*/
      /=/  i2c.bitWrite(sdaPin, sclPin, i2cAddress, register, bitNumber, value)
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }

      // i2COption used as 'bit number'
      if (0 > i2COption || 7 < i2COption){
            result = RESULT_IS_FAIL;
            break;
         }
      // Use device's register if specified, read 1 byte
      result = readBytesFromi2C(i2CAddress, i2CRegister, i2CValue, 1);
      // "!!" convert value 0100 to 1.
      bitWrite (i2CValue[0], i2COption, (!!arg[5]));
      // Use device's register if specified, write 1 byte, returns Wire lib state
      result = writeByteToI2C(i2CAddress, i2CRegister, i2CValue[0]);
      result = (0 == result) ? RESULT_IS_OK : RESULT_IS_FAIL;
      break;
      
    case CMD_I2C_BITREAD:
      /*/
      /=/  i2c.bitRead(sdaPin, sclPin, i2cAddress, register, bit)
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      // i2COption used as 'bit number'
      if (0 > i2COption || 7 < i2COption) { break; }
      // Use device's register if specified, read 1 byte
      result = readBytesFromi2C(i2CAddress, i2CRegister, i2CValue, 1);
      if (0 == result) { break ;}
      value = (int64_t) bitRead(i2CValue[0], i2COption);
      result = RESULT_IN_VARIABLE;
      break;


#endif // FEATURE_I2C_ENABLE

#ifdef FEATURE_DS18X20_ENABLE
    case CMD_DS18X20_TEMPERATURE:
      /*/
      /=/  DS18x20.temperature[pin, resolution, id]
      /*/
      if (! isSafePin(arg[0])) { break; }
      result = getDS18X20Metric(arg[0], arg[1], &cBuffer[_argOffset[2]], cBuffer);
      break;
#endif // FEATURE_DS18X20_ENABLE

#ifdef FEATURE_DHT_ENABLE
    case CMD_DHT_HUMIDITY:
      /*/
      /=/  DHT.humidity[pin, model]
      /*/
      if (! isSafePin(arg[0])) { break; }
      result = getDHTMetric(arg[0], arg[1], SENS_READ_HUMD, cBuffer);
      break;

    case CMD_DHT_TEMPERATURE:
      /*/
      /=/  DHT.temperature[pin, model]
      /*/
      if (! isSafePin(arg[0])) { break; }
      result = getDHTMetric(arg[0], arg[1], SENS_READ_TEMP, cBuffer);
      break;
   
#endif // FEATURE_DHT_ENABLE

       
#ifdef FEATURE_BMP_ENABLE
    case CMD_BMP_PRESSURE:
      /*/
      /=/  BMP.Pressure[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      // (uint8_t) arg[2] is i2c address, 7 bytes size
      result = getBMPMetric(arg[0], arg[1], i2CAddress, arg[3], arg[4], SENS_READ_PRSS, cBuffer);
      break;

    case CMD_BMP_TEMPERATURE:
      /*/
      /=/ BMP.Temperature[sdaPin, sclPin, i2cAddress, overSampling]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      // (uint8_t) arg[2] is i2c address, 7 bytes size
      result = getBMPMetric(arg[0], arg[1], i2CAddress, arg[3], arg[4], SENS_READ_TEMP, cBuffer);
      break;

      
#ifdef SUPPORT_BME280_INCLUDE
      case CMD_BME_HUMIDITY:
      /*/
      /=/  BME.Humidity[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      // (uint8_t) arg[2] is i2c address, 7 bytes size
      result = getBMPMetric(arg[0], arg[1], i2CAddress, arg[3], arg[4], SENS_READ_HUMD, cBuffer);
      break;      
#endif // SUPPORT_BME280_INCLUDE 

#endif // FEATURE_BMP_ENABLE  


#ifdef FEATURE_BH1750_ENABLE
    case CMD_BH1750_LIGHT:
      /*/
      /=/  BH1750.light[sdaPin, sclPin, i2cAddress, mode]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      // (uint8_t) arg[2] is i2c address, 7 bytes size
      result = getBH1750Metric(arg[0], arg[1], i2CAddress, arg[3], SENS_READ_LUX, cBuffer);
      break;
#endif // FEATURE_BH1750_ENABLE

#ifdef FEATURE_MAX7219_ENABLE
    case CMD_MAX7219_WRITE:
      /*/
      /=/  MAX7219.write[dataPin, clockPin, loadPin, intensity, value]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1]) || !isSafePin(arg[2])) { break; }
      drawOnMAX7219Matrix8x8(arg[0], arg[1], arg[2], arg[3], &cBuffer[_argOffset[4]]);
      result = RESULT_IS_OK;
      break;
#endif // FEATURE_MAX7219_ENABLE

#ifdef FEATURE_PCF8574_LCD_ENABLE
    case CMD_PCF8574_LCDPRINT:
      /*/
      /=/  PCF8574.LCDPrint[sdaPin, sclPin, i2cAddress, lcdBacklight, lcdType, data]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      result = printToPCF8574LCD(arg[0], arg[1], i2CAddress, arg[3], arg[4], &cBuffer[_argOffset[5]]);
      break;

#endif // FEATURE_PCF8574_LCD_ENABLE


#ifdef FEATURE_SHT2X_ENABLE
    case CMD_SHT2X_HUMIDITY:
      /*/
      /=/  SHT2X.Humidity[sdaPin, sclPin, i2cAddress]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      // (uint8_t) arg[2] is i2c address, 7 bytes size
      result = getSHT2XMetric(arg[0], arg[1], i2CAddress, SENS_READ_HUMD, cBuffer);
      break;

    case CMD_SHT2X_TEMPERATURE:
      /*/
      /=/  SHT2X.Temperature[sdaPin, sclPin, i2cAddress]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      // (uint8_t) arg[2] is i2c address, 7 bytes size
      result = getSHT2XMetric(arg[0], arg[1], i2CAddress, SENS_READ_TEMP, cBuffer);
      break;
#endif // FEATURE_SHT2X_ENABLE  

#ifdef FEATURE_ACS7XX_ENABLE
    case CMD_ACS7XX_ZC:
      /*/
      /=/  acs7xx.zc[sensorPin, refVoltage]
      /*/
      if (!isSafePin(arg[0])) { break; }
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
          arg[1] = DEFAULT;
       }
       result = getACS7XXMetric(arg[0], arg[1], SENS_READ_ZC, 0, 0, cBuffer);
      break;

    case CMD_ACS7XX_AC:
      /*/
      /=/  acs7xx.ac[sensorPin, refVoltage, sensitivity, zeroPoint] 
      /*/
      if (!isSafePin(arg[0])) { break; }
      // if refVoltage skipped - use DEFAULT source
      if ('\0' != cBuffer[_argOffset[1]]) {
         arg[1] = DEFAULT;
      }
      result = getACS7XXMetric(arg[0], arg[1], SENS_READ_AC, arg[2], (int32_t) arg[3], cBuffer);
      break;

    case CMD_ACS7XX_DC:
      /*/
      /=/  acs7xx.dc[sensorPin, refVoltage, sensitivity, zeroPoint] 
      /*/
      if (!isSafePin(arg[0])) { break; }
         // if refVoltage skipped - use DEFAULT source
      if ('\0' != cBuffer[_argOffset[1]]) {
         arg[1] = DEFAULT;
      }
      result = getACS7XXMetric(arg[0], arg[1], SENS_READ_DC, arg[2], (int32_t) arg[3], cBuffer);
      break;

#endif // FEATURE_ACS7XX_ENABLE


#ifdef FEATURE_ULTRASONIC_ENABLE
    case CMD_ULTRASONIC_DISTANCE:
      /*/
      /=/  ultrasonic.distance[triggerPin, echoPin]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      value = (int64_t) getUltrasonicMetric(arg[0], arg[1]);
      result = RESULT_IN_VARIABLE;
      break;
#endif // FEATURE_ULTRASONIC_ENABLE


#ifdef FEATURE_IR_ENABLE
    case CMD_IR_SEND:
      /*/
      /=/  ir.send[pwmPin, irPacketType, nBits, data, repeat, address]
      /*/
      // ATmega328: Use D3 only at this time
      // Refer to other Arduino's pinouts to find OC2B pin
//      if (isSafePin(arg[0]) && TIMER2B == digitalPinToTimer(arg[0])) {
         // irPWMPin - global wariable that replace IRremote's TIMER_PWM_PIN
//         irPWMPin = arg[0];
      result = sendCommandByIR(arg[1], arg[2], arg[3], arg[4], arg[5]);
//      }
      break;

    case CMD_IR_SENDRAW:
      /*/
      /=/  ir.sendRaw[pwmPin, irFrequency, nBits, data]
      /=/  >> need to increase ARGS_PART_SIZE, because every data`s Integer number take _four_ HEX-chars => 70 RAW array items take 282 (2+70*4) byte of incoming buffer only
      /*/
      // ATmega328: Use D3 only at this time
      // Refer to other Arduino's pinouts to find OC2B pin
 //     if (isSafePin(arg[0]) && TIMER2B == digitalPinToTimer(arg[0])) {
         // irPWMPin - global wariable that replace IRremote's TIMER_PWM_PIN
//         irPWMPin = arg[0];
      result = sendRawByIR(arg[1], arg[2], &cBuffer[_argOffset[3]]);
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
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      // cBuffer cast to (uint8_t*) to use with subroutine math and SoftwareSerial subs, because used instead sub's internal buffer and save a little RAM size.
      // Its will be casted to char* inside at moment when its need
      result = getPZEM004Metric(arg[0], arg[1], SENS_READ_AC, &cBuffer[_argOffset[2]], (uint8_t*) cBuffer);
      break;
    case CMD_PZEM004_VOLTAGE:
      /*/
      /=/  pzem004.voltage[rxPin, txPin, ip]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      result = getPZEM004Metric(arg[0], arg[1], SENS_READ_VOLTAGE, &cBuffer[_argOffset[2]], (uint8_t*) cBuffer);
      break;
    case CMD_PZEM004_POWER:
      /*/
      /=/  pzem004.power[rxPin, txPin, ip]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      result = getPZEM004Metric(arg[0], arg[1], SENS_READ_POWER, &cBuffer[_argOffset[2]], (uint8_t*) cBuffer);
      break;
    case CMD_PZEM004_ENERGY:
      /*/
      /=/  pzem004.energy[rxPin, txPin, ip]
      /*/
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      result = getPZEM004Metric(arg[0], arg[1], SENS_READ_ENERGY, &cBuffer[_argOffset[2]], (uint8_t*) cBuffer);
      break;
#endif // FEATURE_PZEM004_ENABLE

#ifdef FEATURE_UPS_APCSMART_ENABLE

    case CMD_UPS_APCSMART:
      /*/
      /=/  ups.apcsmart[rxPin, txPin, command]
      /=/    command - HEX or ASCII
      /*/  
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      result = getAPCSmartUPSMetric(arg[0], arg[1], (uint8_t*) &cBuffer[_argOffset[2]], (_argOffset[3] - _argOffset[2]) , (uint8_t*) cBuffer);
      break;

#endif // FEATURE_UPS_APCSMART_ENABLE

#ifdef FEATURE_UPS_MEGATEC_ENABLE
    case CMD_UPS_MEGATEC:
      /*/
      /=/  ups.megatec[rxPin, txPin, command, fieldNumber]
      /=/    command - HEX or ASCII
      /*/  
      if (!isSafePin(arg[0]) || !isSafePin(arg[1])) { break; }
      result = getMegatecUPSMetric(arg[0], arg[1], (uint8_t*) &cBuffer[_argOffset[2]], arg[3], (uint8_t*) cBuffer);
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

   // Result output routine
   // The result is already printed or placed in buffer
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
         case RESULT_IN_VARIABLE:
            //  or result value placed in 'value' variable and must be converted to C-string.
            ltoa(value, cBuffer, 10);
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
      }
      //  Push the buffer
      ethClient.println(cBuffer);
   }
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrint_P(PSTR("Result: ")); Serial.println(cBuffer); Serial.println(); 
#endif
   return cmdIdx;
}

