
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 
                             To avoid compilation errors use proper release Arduino IDE, please. 
 
                                                 v1.6.11 and above is good

*/

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                   PROGRAMM FEATURES SECTION

  Please refer to the "cfg_basic.h" file for enabling or disabling Zabbuino's features and refer to the "src/cfg_tune.h" to deep tuning.
  if connected sensors seems not work - first check setting in port_protect[], port_mode[], port_pullup[] arrays in I/O PORTS
  SETTING SECTION of "src/cfg_tune.h" file
*/
#include "src/dispatcher.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                   GLOBAL VARIABLES SECTION
*/

// some members of struct used in timer's interrupt
volatile sysmetrics_t sysMetrics;

netconfig_t netConfig;

NetworkClass Network;


#ifdef TWI_USE
SoftwareWire SoftTWI(constDefaultSDAPin, constDefaultSCLPin);
#endif

#ifdef INTERRUPT_USE
extern volatile extInterrupt_t extInterrupt[];
#endif

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           STARTUP SECTION
*/
void setup() {
#ifdef SERIAL_USE
  Serial.begin(constSerialMonitorSpeed);
#endif // SERIAL_USE

  DTSL( SerialPrint_P(constZbxAgentVersion); PRINTLN_PSTR(" wakes up"); )
  // memset take less progspace?
  memset((void*) &sysMetrics, 0x00, sizeof(sysmetrics_t));

  sysMetrics.sysVCCMin = sysMetrics.sysVCCMax = getADCVoltage(ANALOG_CHAN_VBG);
  sysMetrics.sysRamFree = sysMetrics.sysRamFreeMin = getRamFree();

  //  uint32_t startSerial = millis();
  /*
    On ATMega32u4 with closed Arduino IDE monitor (may be breadboard and dupont wires is bad)

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
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                       GENERAL SECTION
*/
void loop() {
  uint8_t i, result,
          errorCode = ERROR_NONE;
  char incomingData;
  uint16_t blinkType = constBlinkNope;
  uint32_t nowTime, processStartTime, processEndTime, prevPHYCheckTime, prevNetProblemTime, prevSysMetricGatherTime, clientConnectTime, netDebugPrintTime;
  char cBuffer[constBufferSize + 1]; // +1 for trailing \0
  // Last idx is not the same that array size
  char* optarg[constArgC];

  // 0. Init some libs to make system screen works if it enabled
#ifdef TWI_USE
  SoftTWI.begin();
#endif

#ifdef FEATURE_SYSTEM_RTC_ENABLE
  DTSM( PRINT_PSTR("Init system RTC ");
        if (! initRTC(&SoftTWI)) {Serial.print("un"); }
        PRINTLN_PSTR("succesfull"); 
  )  
#endif

#ifdef FEATURE_USER_FUNCTION_PROCESSING
  uint32_t prevUserFuncCall;
  initStageUserFunction(cBuffer);
#endif

  // System load procedure

  // 1. Factory reset block
  //
#ifdef FEATURE_EEPROM_ENABLE
  // Set mode of constFactoryResetButtonPin and turn on internal pull resistor
  digitalWrite(constStateLedPin, LOW);
  // Check for constFactoryResetButtonPin shorting to ground?
  // (when pulled INPUT pin shorted to GND - digitalRead() return LOW)
  if (LOW == digitalRead(constFactoryResetButtonPin)) {
    DTSL( PRINTLN_PSTR("The factory reset button is pressed"); )
    // Fire up state LED
    digitalWrite(constStateLedPin, HIGH);
    // Wait some msecs
    delay(constHoldTimeToFactoryReset);
    // constFactoryResetButtonPin still shorted?
    if (LOW == digitalRead(constFactoryResetButtonPin)) {
      DTSL( PRINTLN_PSTR("Rewrite EEPROM with defaults..."); )
      setConfigDefaults(&netConfig);
      saveConfigToEEPROM(&netConfig);
      // Blink fast while constFactoryResetButtonPin shorted to GND
      DTSL( PRINTLN_PSTR("Done. Release the factory reset button now"); )
      while (LOW == digitalRead(constFactoryResetButtonPin)) {
        digitalWrite(constStateLedPin, millis() % 100 < 50);
      }
    }
    digitalWrite(constStateLedPin, LOW);
  } // if (LOW == digitalRead(constFactoryResetButtonPin))
#endif // FEATURE_EEPROM_ENABLE

  // 2. Load configuration from EEPROM
  //
#ifdef FEATURE_EEPROM_ENABLE
  if (false == loadConfigFromEEPROM(&netConfig)) {
    DTSM( PRINTLN_PSTR("Config load error"); )
    // bad CRC detected, use default values for this run
    setConfigDefaults(&netConfig);
    if (!saveConfigToEEPROM(&netConfig)) {
      // what to do on saving error?
    }
  }
#else // FEATURE_EEPROM_ENABLE
  DTSM( PRINTLN_PSTR("Use default settings"); )
  // Use hardcoded values if EEPROM feature disabled
  setConfigDefaults(&netConfig);
#endif // FEATURE_EEPROM_ENABLE

  // 3. Forcing the system parameters in accordance with the user's compilation options & hardware set
  //
#ifdef FEATURE_PASSWORD_PROTECTION_FORCE
  netConfig.useProtection = true;
#endif

#if defined(FEATURE_SYSTEM_RTC_ENABLE) && defined(FEATURE_EEPROM_ENABLE)
  set_zone(netConfig.tzOffset);
#endif

  // 4. Network initialization and starting
  //
    Network.init(&netConfig);
    Network.restart();

  DTSL( PRINTLN_PSTR("Serving on:");
        Network.showNetworkState();
        PRINT_PSTR("Password: "); Serial.println(netConfig.password, DEC);
#if defined(FEATURE_SYSTEM_RTC_ENABLE) && defined(FEATURE_EEPROM_ENABLE)
        PRINT_PSTR("Timezone: "); Serial.println(netConfig.tzOffset, DEC);
#endif

      )

/*
#if !defined(NETWORK_RS485)
  // Start listen sockets
  Network.server.begin();
#endif
*/
  // 5. Other system parts initialization
  //
  // I/O ports initialization. Refer to "I/O PORTS SETTING SECTION" in src\cfg_tune.h
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
    extInterrupt[i].value = 0;
  }
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

  // Correcting timestamps
  prevPHYCheckTime = prevNetProblemTime = prevSysMetricGatherTime =  netDebugPrintTime = clientConnectTime = millis();

  // 6. Enter to infinitive loop to serve incoming requests
  //
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
      // When FEATURE_SYSINFO_ENABLE is disabled, compiler can be omit gatherSystemMetrics() sub (due find no operators inside) and trow exception
#ifndef GATHER_METRIC_USING_TIMER_INTERRUPT
      gatherSystemMetrics();
#endif
      //sysMetrics.sysVCC = getADCVoltage(ANALOG_CHAN_VBG);
      // correctVCCMetrics() must be always inline compiled
      //correctVCCMetrics(sysMetrics.sysVCC);
      correctVCCMetrics(getADCVoltage(ANALOG_CHAN_VBG));
      prevSysMetricGatherTime = nowTime;
      // update millis() rollovers to using it in uptime() function
      millisRollover();
    }

#if defined(FEATURE_NET_DHCP_ENABLE) || defined (NETWORK_ETH_ENC28J60)
// || defined (NETWORK_RS485)
    // maintain() is very important for UIPEthernet, because it call internal tick() subroutine
    // but this subroutine adds more fat to WIZnet drivers, because includes DHCP functionality to firmware
    result = Network.maintain();
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
          DTSM( PRINTLN_PSTR("DHCP renew problem occured"); )
      }
    }
#endif // FEATURE_NET_DHCP_ENABLE

    // No DHCP problem found, but no data recieved or network activity for a long time
    if (ERROR_NONE == errorCode && (constNetIdleTimeout <= (uint32_t) (nowTime - prevNetProblemTime))) {
      blinkType = constBlinkNetworkProblem;
      errorCode = ERROR_NET;
    }

#if defined(FEATURE_NETWORK_MONITORING)
    if (constPHYCheckInterval <= (uint32_t) (nowTime - prevPHYCheckTime)) {
      // netModuleCheck() subroutine returns true if detect network module error and reinit it.
      if (5000UL <= (uint32_t) (nowTime - netDebugPrintTime )) {
        DTSL( Network.showPHYState(); )
        netDebugPrintTime = nowTime;
      }
      if (Network.checkPHY()) {
        sysMetrics.netPHYReinits++;
        DTSL( PRINT_PSTR("Network module reinit #"); Serial.println(sysMetrics.netPHYReinits); )
      }
      prevPHYCheckTime = millis();
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
      if (!Network.client) {
        Network.client = Network.server.available();
        if (!Network.client) {
#ifdef FEATURE_USER_FUNCTION_PROCESSING
          // Call "loop stage" user function screen every constRenewSystemDisplayInterval only if no connection exist, because that function can modify cBuffer content
          // and recieved data can be corrupted
          if (constUserFunctionCallInterval <= (uint32_t) (nowTime - prevUserFuncCall)) {
            loopStageUserFunction(cBuffer);
            prevUserFuncCall = nowTime;
          }
#endif // FEATURE_USER_FUNCTION_PROCESSING
          continue;
        }
        // reinit analyzer because previous session can be dropped or losted
        analyzeStream('\0', cBuffer, optarg, REINIT_ANALYZER);
        clientConnectTime = millis();
        DTSH( PRINT_PSTR("New client #"); Serial.println(Network.client); )
      }

      // Client will be dropped if its connection so slow. Then round must be restarted.
      if (constNetSessionTimeout <= (uint32_t) (millis() - clientConnectTime)) {
        DTSH( PRINT_PSTR("Drop client #"); Serial.println(Network.client); )
        Network.client.stop();
        continue;
      }

      // Network data can be splitted to a number frames and ethClient.available() can return 0 on first frame processing. Need to ignore it.
      if (!Network.client.available()) {
        continue;
      }
      incomingData = Network.client.read();
#ifdef FEATURE_SERIAL_LISTEN_TOO
    } else {
      // If in Serial buffer is exist just read it
      incomingData = Serial.read();
    }
#endif

    result = analyzeStream(incomingData, cBuffer, optarg, NO_REINIT_ANALYZER);
    // result is true if analyzeStream() do not finished and need more data
    // result is false if EOL or trailing char detected or there no room in buffer or max number or args parsed...
    if (true == result) {
      continue;
    }
    // ethClient.connected() returns true even client is disconnected, but leave the data in the buffer.
    // Data can be readed, command will executed, but why get answer? If no recipient - why need to load MCU?
    // But checking must be disable for commands which coming in from the Serial. Otherwise commands will be never executed if no active network client exist.
#ifndef FEATURE_SERIAL_LISTEN_TOO
    if (!Network.client.connected()) {
      continue;
    }
#endif
    // Fire up State led, than will be turned off on next loop
    digitalWrite(constStateLedPin, HIGH);
    // may be need test for client.connected()?
    processStartTime = millis();
    DTSM( uint32_t ramBefore = getRamFree(); )
    DTSL( Serial.print(cBuffer); PRINT_PSTR(" => "); )
    //sysMetrics[IDX_METRIC_SYS_CMD_LAST] = executeCommand(cBuffer, optarg, &netConfig);
    sysMetrics.sysCmdLast = executeCommand(cBuffer, optarg, &netConfig);
#ifdef FEATURE_REMOTE_COMMANDS_ENABLE
    // When system.run[] command is recieved, need to run another command, which taken from option #0 by cmdIdx() sub
    if (RESULT_IS_NEW_COMMAND == sysMetrics.sysCmdLast) {
      int16_t k = 0;
      // simulate command recieving to properly string parsing
      while (analyzeStream(cBuffer[k], cBuffer, optarg, false)) {
        k++;
      }
      DTSL( Serial.print(cBuffer); PRINT_PSTR(" => "); )
      sysMetrics.sysCmdLast = executeCommand(cBuffer, optarg, &netConfig);
    }
#endif
    processEndTime = millis();
    // use processEndTime as processDurationTime
    processEndTime = processEndTime - processStartTime ;
    DTSM( PRINT_PSTR("Spended: "); Serial.print(processEndTime);
          PRINT_PSTR(" ms, "); Serial.print((ramBefore - getRamFree()));
          PRINTLN_PSTR(" memory bytes");
        )
    // Change internal runtime metrics if need
    //    if ((uint32_t) sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] < processEndTime) {
    //      sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = processEndTime;
    //      sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N] = sysMetrics[IDX_METRIC_SYS_CMD_LAST];
    //    }
    if (sysMetrics.sysCmdTimeMax < processEndTime) {
      sysMetrics.sysCmdTimeMax = processEndTime;
      sysMetrics.sysCmdTimeMaxN = sysMetrics.sysCmdLast;
    }

    // Wait some time to finishing answer send, close connection, and restart network activity control cycle
    //delay(constNetStabilizationDelay);
    // Actually Ethernet lib's flush() do nothing, but UIPEthernet flush() free ENC28J60 memory blocks where incoming (?) data stored
    Network.client.flush();
    Network.client.stop();
#ifdef FEATURE_SERIAL_LISTEN_TOO
    // Flush the incoming Serial buffer by reading because Serial object have no clear procedure.
    while (0 < Serial.available()) {
      Serial.read();
    }
#endif
    sysMetrics.sysCmdLastExecTime = prevPHYCheckTime = prevNetProblemTime = millis();
    blinkType = constBlinkNope;
    errorCode = ERROR_NONE;
  } // while(true)
}


/* ****************************************************************************************************************************


**************************************************************************************************************************** */
static int16_t executeCommand(char* _dst, char* _optarg[], netconfig_t* _netConfig) {

  int8_t rc;
  uint8_t accessGranted = false;
  uint16_t i;
  int16_t cmdIdx;
  // duration option in the tone[] command is ulong
  //uint32_t argv[constArgC];
  int32_t argv[constArgC];
  // Zabbix use 64-bit numbers, but we can use only -uint32_t...+uint32_t range. Error can be occurs on ltoa() call with value > long_int_max
  int32_t value = 0;
  NetworkAddress tmpAddress;

  //DTSM( Serial.print("[0] "); Serial.println(millis()); )

  rc = RESULT_IS_FAIL;
  cmdIdx = -1;
  //sysMetrics[IDX_METRIC_SYS_CMD_COUNT]++;
  sysMetrics.sysCmdCount++;
  i = arraySize(commands);
  // Search specified command index in the list of implemented functions
  for (; 0 != i;) {
    i--;
    DTSD( Serial.print("# ");  Serial.print(i, HEX); Serial.print(" => "); SerialPrintln_P((char*)pgm_read_word(&(commands[i]))); )
    if (0 == strcmp_P(_dst, (char*)pgm_read_word(&(commands[i])))) {
      cmdIdx = i;
      break;
    }
  }

  // Decremented early increased command counter and got to show result ZBX_NOTSUPPORTED if no suitable command found
  if (0 > cmdIdx) {
    sysMetrics.sysCmdCount--;
    rc = ZBX_NOTSUPPORTED;
    goto finish;
  }

  //DTSM( Serial.print("[1] "); Serial.println(millis()); )
  DTSM( PRINT_PSTR("Execute command #"); Serial.print(cmdIdx, HEX); PRINT_PSTR(" => `"); Serial.print(_dst); Serial.println("`"); )

  // ***************************************************************************************************************
  // If command have no options - it must be run immedately
  switch (cmdIdx) {
    //  case  CMD_ZBX_NOPE:
    //        break;
    case CMD_ZBX_AGENT_PING:
      //
      //   agent.ping
      //
      rc = RESULT_IS_OK;
      break;

    case CMD_ZBX_AGENT_HOSTNAME:
      //
      //   agent.hostname
      //
      strcpy(_dst, _netConfig->hostname);
      rc = RESULT_IN_BUFFER;
      break;

    case CMD_ZBX_AGENT_VERSION:
      //
      //  agent.version
      //
      strcpy_P(_dst, constZbxAgentVersion);
      rc = RESULT_IN_BUFFER;
      break;

    case CMD_SYSTEM_UPTIME:
      //
      //  sys.uptime
      //
      value = uptime();
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

#ifdef FEATURE_SYSINFO_ENABLE
    case CMD_SYSTEM_HW_CHASSIS:
      //
      //  system.hw.chassis
      //
      strcpy_P(_dst, PSTR(BOARD));
      rc = RESULT_IN_BUFFER;
      break;

    case CMD_NET_PHY_NAME:
      //
      //  sys.net.module
      //
      strcpy_P(_dst, PSTR(PHY_MODULE_NAME));
      rc = RESULT_IN_BUFFER;
      break;

    case CMD_NET_PHY_REINITS:
      //
      //  net.phy.reinits
      //
      value = sysMetrics.netPHYReinits;
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

    case CMD_SYS_CMD_TIMEMAX_N:
      //
      //  sys.cmd.timemax.n
      //
      ultoa(sysMetrics.sysCmdTimeMaxN, _dst, 16);
      rc = RESULT_IN_BUFFER;

      break;

    case CMD_SYS_RAM_FREE:
      //
      //  sys.ram.free
      //
      //  That metric must be collected periodically to avoid returns always same data
      value = sysMetrics.sysRamFree;
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

    case CMD_SYS_RAM_FREEMIN:
      //
      //  sys.ram.freemin
      //
      // Without ATOMIC_BLOCK block using sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] variable can be changed in interrupt on reading
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = sysMetrics.sysRamFreeMin;
      }
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;
#endif

    case CMD_SYS_VCC:
      //
      // sys.vcc
      //
      // Take VCC
      value = getADCVoltage(ANALOG_CHAN_VBG);
      // VCC may be bigger than max or smaller than min.
      // To avoid wrong results and graphs in monitoring system - correct min/max metrics
      correctVCCMetrics(value);
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

    case CMD_SYS_VCCMIN:
      //
      // sys.vccMin
      //
      //value = sysMetrics[IDX_METRIC_SYS_VCCMIN];
      value = sysMetrics.sysVCCMin;
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

    case CMD_SYS_VCCMAX:
      //
      // sys.vccMax
      //
      value = sysMetrics.sysVCCMax;
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

#ifdef FEATURE_SYSTEM_RTC_ENABLE

    case CMD_SYSTEM_LOCALTIME:
      //
      //  system.localtime
      //  Zabbix wants UTC as localtime
      //
      if (getUnixTime(&SoftTWI, (uint32_t*) &value)) {
        rc = RESULT_IS_UNSIGNED_VALUE;
      }
      break;
#endif // FEATURE_SYSTEM_RTC_ENABLE
  }

  // ***************************************************************************************************************
  // Command with options take more time
  // batch convert args to number values
  for (i = constArgC; 0 != i;) {
    i--;
    //argv[i] = ('\0' == *_optarg[i]) ? 0 : strtoul((char*) _optarg[i], NULL, 0);
    // strtoul return 0 if _optarg[i] eq '\0'
    //argv[i] = strtoul((char*) _optarg[i], NULL, 0);
    argv[i] = strtol((char*) _optarg[i], NULL, 0);
    DTSH(
      PRINT_PSTR("argv["); Serial.print(i); PRINT_PSTR("] => \"");
    if ('\0' == *_optarg[i]) {
      PRINT_PSTR("<null>");
    } else {
      Serial.print((char*) _optarg[i]);
    }
    PRINT_PSTR("\" => "); Serial.println(argv[i]);
    //PRINT_PSTR(", offset ="); Serial.println(_optarg[i], HEX);
    )
  }

  //DTSM( Serial.print("[2] "); Serial.println(millis()); )

  // Check rights for password protected action
  accessGranted = (!_netConfig->useProtection || (uint32_t) argv[0] == _netConfig->password);

  //DTSM( Serial.print("[3] "); Serial.println(millis()); )

  switch (cmdIdx) {
#ifdef FEATURE_USER_FUNCTION_PROCESSING
    case CMD_USER_RUN:
      //
      //  user.run[option#0, option#1, option#2, option#3, option#4, option#5]
      //
      rc = executeCommandUserFunction(_dst, _optarg, argv);
      break;

#endif // FEATURE_USER_FUNCTION_PROCESSING

#ifdef FEATURE_REMOTE_COMMANDS_ENABLE
    case CMD_SYSTEM_RUN:
      //
      //  system.run["newCommand"]
      //
      if ('\0' == *_optarg[0]) {
        break;
      }
      // take length of 0-th arg + 1 byte for '\0'
      //i = (_argOffset[1] - _argOffset[0]) + 1;
      //i = (_optarg[1] - _optarg[0]) + 1;        << valid ???
      i = strlen((char*) _optarg[0]) + 1;
      // move it to begin of buffer to using as new incoming command
      // Note: ~8bytes can be saved with copying bytes in while() cycle. But source code will not beauty
      memmove(_dst, _optarg[0], i);
      _dst[i] = '\n';
//      _dst[i+1] = '\0';
      // immediately return RESULT_IS_NEW_COMMAND to re-run executeCommand() with new command
      return RESULT_IS_NEW_COMMAND;
      break;
#endif

    case CMD_ARDUINO_ANALOGWRITE:
      //
      //  analogWrite[pin, value]
      //
      if (! isSafePin(argv[0])) {
        break;
      }
      analogWrite(argv[0], argv[1]);
      rc = RESULT_IS_OK;
      break;

    case CMD_ARDUINO_ANALOGREAD:
      //
      //  analogRead[pin, analogReferenceSource, mapToLow, mapToHigh]
      //
#ifdef FEATURE_AREF_ENABLE
      // change source of the reference voltage if its given
      if ('\0' != *_optarg[1]) {
        analogReference(argv[1]);
        delayMicroseconds(2000);
      }
#endif
      if (! isSafePin(argv[0])) {
        break;
      }
      value = analogRead(argv[0]);
      if ('\0' != *_optarg[2] && '\0' != *_optarg[3]) {
        value = map(value, 0, 1023, argv[2], argv[3]);
      }
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

#ifdef FEATURE_AREF_ENABLE
    case CMD_ARDUINO_ANALOGREFERENCE:
      //
      //  analogReference[source]
      //
      analogReference(argv[0]);
      rc = RESULT_IS_OK;
      break;
#endif

    case CMD_ARDUINO_DELAY:
      //
      //  delay[time]
      //
      delay(argv[0]);
      rc = RESULT_IS_OK;
      break;

    case CMD_ARDUINO_DIGITALWRITE:
      //
      //  digitalWrite[pin, value, testPin, testValue]
      //
      // if testPin defined - check both pin to safety
      i = ('\0' == *_optarg[2]) ? isSafePin(argv[0]) : (isSafePin(argv[0]) && isSafePin(argv[2]));
      if (!i) {
        break;
      }
      // turn on or turn off logic on pin
      digitalWrite(argv[0], argv[1]);
      rc = RESULT_IS_OK;
      if ('\0' == *_optarg[2]) {
        break;
      }
      // when testPin defined - switch testPin mode to input, wait a lot, and check testPin state.
      // if readed value not equal testValue - return FAIL
      pinMode(argv[2], INPUT_PULLUP);
      delay(10);
      if ((uint32_t) digitalRead(argv[2]) != (uint32_t) argv[3]) {
        rc = RESULT_IS_FAIL;
      }
      break;

    case CMD_ARDUINO_DIGITALREAD:
      //
      //  digitalRead[pin]                                                      
      //
      value = digitalRead(argv[0]);
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

#ifdef FEATURE_TONE_ENABLE
    case CMD_ARDUINO_TONE:
      //
      //  tone[pin, frequency, duration]
      //
      if (! isSafePin(argv[0])) {
        break;
      }
      rc = RESULT_IS_OK;
      if ('\0' != *_optarg[2]) {
        tone(argv[0], argv[1], argv[2]);
        break;
      }
      tone(argv[0], argv[1]);
      break;

    case CMD_ARDUINO_NOTONE:
      //
      //  noTone[pin]
      //
      if (! isSafePin(argv[0])) {
        break;
      }
      noTone(argv[0]);
      rc = RESULT_IS_OK;
      break;

#endif

#ifdef FEATURE_RANDOM_ENABLE
    case CMD_ARDUINO_RANDOMSEED:
      //
      //  randomSeed[value]
      //
      randomSeed((0 == argv[0]) ? (int32_t) millis() : argv[0]);
      rc = RESULT_IS_OK;
      break;

    case CMD_ARDUINO_RANDOM:
      //
      //  random[min, max]
      //
      //  !! random return long
      value = ('\0' == *_optarg[1]) ? random(argv[0]) : random(argv[0], argv[1]);
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;
#endif // FEATURE_RANDOM_ENABLE

#ifdef FEATURE_EEPROM_ENABLE
    case CMD_SET_HOSTNAME:
      //
      //  set.hostname[password, hostname]
      //
      if (!accessGranted) {
        break;
      }
      // need check for arg existsience?
      // _dst[_argOffset[1]] != \0 if argument #2 given
      if ('0' == *_optarg[1]) {
        break;
      }

      // copy <1-th arg length> bytes from 1-th arg of buffer (0-th arg contain password) to hostname
      //i = (uint8_t) _argOffset[2]-_argOffset[1];
      i = strlen((char*) _optarg[1]);
      if (i > constAgentHostnameMaxLength) {
        i = constAgentHostnameMaxLength;
      }
      // memset(_netConfig->hostname, '\0', i);
      //copy 0 .. (constAgentHostnameMaxLength-1) chars from buffer to hostname
      memcpy(_netConfig->hostname, _optarg[1], i);
      // Terminate string
//      _netConfig->hostname[constAgentHostnameMaxLength] = '\0';
      _netConfig->hostname[i] = '\0';
      saveConfigToEEPROM(_netConfig);
      rc = RESULT_IS_OK;
      break;

    case CMD_SET_PASSWORD:
      //
      //  set.password[oldPassword, newPassword]
      //
      if (!accessGranted) {
        break;
      }
      if ('\0' == *_optarg[1]) {
        break;
      }
      // take new password from argument #2
      _netConfig->password = argv[1];
      saveConfigToEEPROM(_netConfig);
      rc = RESULT_IS_OK;
      break;

    case CMD_SET_SYSPROTECT:
      //
      //  set.sysprotect[password, protection]
      //
      if (!accessGranted) {
        break;
      }
      if ('\0' == *_optarg[1]) {
        break;
      }
      _netConfig->useProtection = (1 == argv[1]) ? true : false;
      saveConfigToEEPROM(_netConfig);
      rc = RESULT_IS_OK;
      break;

    case CMD_SET_NETWORK:
      //
      //  set.network[password, useDHCP, macAddress, ipAddress, ipNetmask, ipGateway]
      //
      if (!accessGranted) {
        break;
      }
      uint8_t success;
      success = true;
      // useDHCP flag coming from argument#1 and must be numeric (boolean) - 1 or 0,
      // argv[0] data contain in _dst[_argOffset[1]] placed from _argOffset[0]
      _netConfig->useDHCP = (uint8_t) argv[1];
      // ip, netmask and gateway have one structure - 4 byte
      // take 6 bytes from second argument of command and use as new MAC-address
      // if convertation is failed (sub return -1) variable must be falsed too via logic & operator
      success &= (6 == hstoba((uint8_t *) &_netConfig->macAddress, _optarg[2]));
      // If string to which point _optarg[3] can be converted to valid NetworkAddress - just do it.
      // Otherwize (string can not be converted) _netConfig->ipAddress will stay untouched;
      success = (success) ? (strToNetworkAddress((char*) _optarg[3], &_netConfig->ipAddress)) : false;
      success = (success) ? (strToNetworkAddress((char*) _optarg[4], &_netConfig->ipNetmask)) : false;
      success = (success) ? (strToNetworkAddress((char*) _optarg[5], &_netConfig->ipGateway)) : false;
      // if any convert operation failed - stop store process
      if (!success) {
        break;
      }
      // Save config to EEPROM on success
      rc = saveConfigToEEPROM(_netConfig) ? RESULT_IS_OK : DEVICE_ERROR_EEPROM_CORRUPTED;
      break;
#endif // FEATURE_EEPROM_ENABLE

    case CMD_SYS_PORTWRITE:
      //
      //  portWrite[port, value]
      //
      if (PORTS_NUM >= (argv[0] - 96)) {
        break;
      }
      writeToPort((byte) argv[0] - 96, argv[1]);
      rc = RESULT_IS_OK;
      break;

#ifdef FEATURE_SHIFTOUT_ENABLE
    case CMD_SYS_SHIFTOUT:
      //
      //  shiftOut[dataPin, clockPin, latchPin, bitOrder, compressionType, data]
      //
      // i variable used as latchPinDefined
      i = ('\0' != argv[2]) && isSafePin(argv[2]);
      if (isSafePin(argv[0]) &&  isSafePin(argv[1])) {
        if (i) {
          digitalWrite(argv[2], LOW);
        }
        rc = shiftOutAdvanced(argv[0], argv[1], argv[3], argv[4], (uint8_t*) _optarg[5]);
        if (i) {
          digitalWrite(argv[2], HIGH);
        }
      }
      break;
#endif

#ifdef FEATURE_WS2812_ENABLE
    case CMD_WS2812_SENDRAW:
      //
      //  WS2812.sendRaw[dataPin, compressionType, data]
      //  !!! need to increase ARGS_PART_SIZE, because every encoded LED color take _six_ HEX-chars => 10 leds stripe take 302 (2+50*6) byte of incoming buffer only
      //
      // Tested on ATmega328@16 and 8 pcs WS2812 5050 RGB LED bar
      if (isSafePin(argv[0])) {
        rc = WS2812Out(argv[0], argv[1], (uint8_t*) _optarg[2]);
      }
      break;
#endif // FEATURE_WS2812_ENABLE

    case CMD_SYS_REBOOT:
      //
      //  reboot[password]
      //
      if (! accessGranted) {
        break;
      }
      if (Network.client.connected()) {
        Network.client.println("1");
      }
      // hang-up if no delay
      delay(constNetStabilizationDelay);
      Network.client.stop();
      // The reason why using the watchdog timer or RST_SWRST_bm is preferable over jumping to the reset vector, is that when the watchdog or RST_SWRST_bm resets the AVR, 
      // the registers will be reset to their known, default settings. Whereas jumping to the reset vector will leave the registers in their previous state, which is 
      // generally not a good idea. http://www.atmel.com/webdoc/avrlibcreferencemanual/FAQ_1faq_softreset.html
#ifdef FEATURE_WATCHDOG_ENABLE
      // Watchdog deactivation
      wdt_disable();
#endif
      asm volatile ("jmp 0");
      break;


#ifdef FEATURE_SYSINFO_ENABLE
    case CMD_SYSTEM_HW_CPU:
      //
      //  system.hw.cpu[metric]
      //
      if (0 == strcmp_P(_optarg[0], PSTR("id"))) {
        // Read 10 bytes with step 1 (0x0E..0x17) of the signature row <= http://www.avrfreaks.net/forum/unique-id-atmega328pb
        getBootSignatureBytes(_dst, 0x0E, 10, 1);
        //if (0 == strcmp_P(_optarg[0], "freq")) {
        // Return back CPU frequency
        // strcpy_P(_dst, PSTR(????));
      } else if (0 == strcmp_P(_optarg[0], PSTR("model"))) {
        // Read 3 bytes with step 2 (0x00, 0x02, 0x04) of the signature row <= http://www.avrfreaks.net/forum/device-signatures
        getBootSignatureBytes(_dst, 0x00, 3, 2);
      } else {
        // Return back CPU name
        strcpy_P(_dst, PSTR(_AVR_CPU_NAME_));
      }
      rc = RESULT_IN_BUFFER;
      break;

    case CMD_SYS_CMD_COUNT:
      //
      //  sys.cmd.count
      //
      if (argv[0]) {
        sysMetrics.sysCmdCount = 0;
      }
      value = sysMetrics.sysCmdCount;
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

    case CMD_SYS_CMD_TIMEMAX:
      //
      //  sys.cmd.timemax[resetCounter]
      //
      if (*_optarg[0]) {
        sysMetrics.sysCmdTimeMax = sysMetrics.sysCmdTimeMaxN = 0;
        //sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = 0;
        //sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX_N] = 0;
      }
      //value = sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX];
      value = sysMetrics.sysCmdTimeMax;
      rc = RESULT_IS_UNSIGNED_VALUE;
      break;

#endif // FEATURE_SYSINFO_ENABLE

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
    case CMD_EXTINT_COUNT:
      //
      //  extInt.count[intPin, mode]
      //
      //  Unfortunately, (rc == RESULT_IS_UNSIGNED_VALUE && value == 0) and (rc == RESULT_IS_FAIL) are looks equal for zabbix -> '0'
      //
      if (! isSafePin(argv[0])) {
        break;
      }
      rc = manageExtInt((uint32_t*) &value, argv[0], argv[1]);
      break;

#endif // FEATURE_EXTERNAL_INTERRUPT_ENABLE

#ifdef FEATURE_OW_ENABLE
    case CMD_OW_SCAN:
      //
      //  OW.scan[pin]
      //
      if (! isSafePin(argv[0])) {
        break;
      }
      rc = scanOneWire(argv[0], &Network);
      break;
#endif // FEATURE_ONEWIRE_ENABLE


#ifdef FEATURE_DS18X20_ENABLE
    case CMD_DS18X20_TEMPERATURE:
      //
      //  DS18x20.temperature[pin, resolution, id]
      //
      if (! isSafePin(argv[0])) {
       //   rc = DEVICE_ERROR_CONNECT;
        break;
      }

      uint8_t dsAddr[8];
      dsAddr[0] = 0;
      // Convert sensor ID (if its given) from HEX string to byte array (DeviceAddress structure) and validate (sub not finished) it.
      // Sensor ID is equal DeviceAddress.
      // if convertation not successfull or ID not valid - return DEVICE_ERROR_WRONG_ID
      if (('\0' != *_optarg[2]) && (8 != hstoba(dsAddr, _optarg[2]))) {
        rc = DEVICE_ERROR_WRONG_ID;
        break;
      }
      rc = getDS18X20Metric(argv[0], argv[1], dsAddr, _dst);
      break;
#endif // FEATURE_DS18X20_ENABLE

#ifdef FEATURE_MHZXX_PWM_ENABLE
    case CMD_MHZXX_PWM_CO2:
      //
      //  MHZxx.PWM.CO2[pin, range]
      //
      if (! isSafePin(argv[0])) {
        // rc = DEVICE_ERROR_CONNECT;
        break;
      }
      rc = getMHZxxMetricPWM(argv[0], argv[1], (uint8_t*) _dst);
      break;
#endif // FEATURE_MHZXX_PWM_ENABLE

#ifdef FEATURE_MHZXX_UART_ENABLE
        case CMD_MHZXX_UART_CO2:
          //
          //  MHZxx.UART.CO2[rxPin, txPin]
          //
          rc = getMHZxxMetricUART(argv[0], argv[1], (uint8_t*) _dst);
          break;
#endif // FEATURE_MHZXX_UART_ENABLE


#ifdef FEATURE_DHT_ENABLE
    case CMD_DHT_HUMIDITY:
      //
      //  DHT.humidity[pin, model]
      //
      if (! isSafePin(argv[0])) {
        rc = DEVICE_ERROR_CONNECT;
        break;
      }
      rc = getDHTMetric(argv[0], argv[1], SENS_READ_HUMD, _dst);
      break;

    case CMD_DHT_TEMPERATURE:
      //
      //  DHT.temperature[pin, model]
      //
      if (! isSafePin(argv[0])) {
        //rc = DEVICE_ERROR_CONNECT;
        break;
      }
      rc = getDHTMetric(argv[0], argv[1], SENS_READ_TEMP, _dst);
      break;

#endif // FEATURE_DHT_ENABLE

#ifdef FEATURE_MAX7219_ENABLE
    case CMD_MAX7219_WRITE:
      //
      //  MAX7219.write[dataPin, clockPin, loadPin, intensity, data]
      //
      if (!isSafePin(argv[0]) || !isSafePin(argv[1]) || !isSafePin(argv[2])) {
        break;
      }
      writeToMAX7219(argv[0], argv[1], argv[2], argv[3], _optarg[4]);
      rc = RESULT_IS_OK;
      break;
#endif // FEATURE_MAX7219_ENABLE


#ifdef FEATURE_ACS7XX_ENABLE
    case CMD_ACS7XX_ZC:
      //
      //  acs7xx.zc[sensorPin, refVoltage]
      //
      if (!isSafePin(argv[0])) {
        break;
      }
      //
      //   for ATmega1280, ATmega2560, ATmega1284, ATmega1284P, ATmega644, ATmega644A, ATmega644P, ATmega644PA
      //      INTERNAL1V1   2  - 1,1V
      //      INTERNAL2V56   3  - 2,56V
      //   ATmega328 and so
      //      INTERNAL   3  - 1,1V
      //   Both
      //      DEFAULT          1  - VCC
      //      EXTERNAL   0  - AREF << mV on AREF, more than '3'

      // if refVoltage skipped - use DEFAULT source
      if ('\0' != *_optarg[1]) {
        argv[1] = DEFAULT;
      }
      rc = getACS7XXMetric(argv[0], argv[1], SENS_READ_ZC, 0, 0, _dst);
      break;

    case CMD_ACS7XX_AC:
      //
      //  acs7xx.ac[sensorPin, refVoltage, sensitivity, zeroPoint]
      //
      if (!isSafePin(argv[0])) {
        break;
      }
      // if refVoltage skipped - use DEFAULT source
      if ('\0' != *_optarg[1]) {
        argv[1] = DEFAULT;
      }
      rc = getACS7XXMetric(argv[0], argv[1], SENS_READ_AC, argv[2], (int32_t) argv[3], _dst);
      break;

    case CMD_ACS7XX_DC:
      //
      //  acs7xx.dc[sensorPin, refVoltage, sensitivity, zeroPoint]
      //
      if (!isSafePin(argv[0])) {
        break;
      }
      // if refVoltage skipped - use DEFAULT source
      if ('\0' != *_optarg[1]) {
        argv[1] = DEFAULT;
      }
      rc = getACS7XXMetric(argv[0], argv[1], SENS_READ_DC, argv[2], (int32_t) argv[3], _dst);
      break;

#endif // FEATURE_ACS7XX_ENABLE

#ifdef FEATURE_IR_ENABLE
    case CMD_IR_SEND:
      //
      //  ir.send[pwmPin, irPacketType, nBits, data, repeat, address]
      //
      // ATmega328: Use D3 only at this time
      // Refer to other Arduino's pinouts to find OC2B pin
      //      if (isSafePin(argv[0]) && TIMER2B == digitalPinToTimer(argv[0])) {
      // irPWMPin - global wariable that replace IRremote's TIMER_PWM_PIN
      //         irPWMPin = argv[0];
      rc = sendCommandByIR(argv[1], argv[2], argv[3], argv[4], argv[5]);
      //      }
      break;

    case CMD_IR_SENDRAW:
      //
      //  ir.sendRaw[pwmPin, irFrequency, nBits, data]
      //  !!! need to increase ARGS_PART_SIZE, because every data`s Integer number take _four_ HEX-chars => 70 RAW array items take 282 (2+70*4) byte of incoming buffer only
      //
      // ATmega328: Use D3 only at this time
      // Refer to other Arduino's pinouts to find OC2B pin
      //     if (isSafePin(argv[0]) && TIMER2B == digitalPinToTimer(argv[0])) {
      // irPWMPin - global wariable that replace IRremote's TIMER_PWM_PIN
      //         irPWMPin = argv[0];
      rc = sendRawByIR(argv[1], argv[2], _optarg[3]);
      //     }
      break;
#endif // FEATURE_IR_ENABLE
    default:

      // ************************************************************************************************************************************
      // Following commands use <argv[0]> or <argv[1]> pins for sensor handling (UART, I2C, etc) and these pins can be disabled in port_protect[] array
      //  Otherwise - processing is failed
      if (! isSafePin(argv[0]) || ! isSafePin(argv[1])) {
        rc = RESULT_IS_FAIL;
        goto finish;
      }

      // ************************************************************************************************************************************
      // Non-I2C related commands block
      //
      switch (cmdIdx) {
#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE
        case CMD_INCENC_VALUE:
          //
          //  incEnc.value[terminalAPin, terminalBPin, initialValue]
          //
          // argv[3] (intNumber) currently not used
          rc = manageIncEnc(&value, argv[0], argv[1], argv[2]);
          break;
#endif // FEATURE_INCREMENTAL_ENCODER_ENABLE

#ifdef FEATURE_ULTRASONIC_ENABLE
        case CMD_ULTRASONIC_DISTANCE:
          //
          //  ultrasonic.distance[triggerPin, echoPin]
          //
          value = getUltrasonicMetric(argv[0], argv[1]);
          rc = RESULT_IS_UNSIGNED_VALUE;
          break;
#endif // FEATURE_ULTRASONIC_ENABLE

#ifdef FEATURE_PZEM004_ENABLE
        //
        //  0xC0A80101 - an default IP address for PZEM (192.168.1.1)
        //
        case CMD_PZEM004_CURRENT:
          //
          //  pzem004.current[rxPin, txPin, addr]
          //
          // _dst cast to (uint8_t*) to use with subroutine math and SoftwareSerial subs, because used instead sub's internal buffer and save a little RAM size.
          // Its will be casted to char* inside at moment when its need
          rc = getPZEM004Metric(argv[0], argv[1], SENS_READ_AC, _optarg[2], (uint8_t*) _dst);
          break;
        case CMD_PZEM004_VOLTAGE:
          //
          //  pzem004.voltage[rxPin, txPin, addr]
          //
          rc = getPZEM004Metric(argv[0], argv[1], SENS_READ_VOLTAGE, _optarg[2], (uint8_t*) _dst);
          break;
        case CMD_PZEM004_POWER:
          //
          //  pzem004.power[rxPin, txPin, addr]
          //
          rc = getPZEM004Metric(argv[0], argv[1], SENS_READ_POWER, _optarg[2], (uint8_t*) _dst);
          break;
        case CMD_PZEM004_ENERGY:
          //
          //  pzem004.energy[rxPin, txPin, addr]
          //
          rc = getPZEM004Metric(argv[0], argv[1], SENS_READ_ENERGY, _optarg[2], (uint8_t*) _dst);
          break;

        case CMD_PZEM004_SETADDR:
          //
          //  pzem004.setAddr[rxPin, txPin, addr]
          //
          rc = getPZEM004Metric(argv[0], argv[1], SENS_CHANGE_ADDRESS, _optarg[2], (uint8_t*) _dst);
          break;         
#endif // FEATURE_PZEM004_ENABLE

#ifdef FEATURE_UPS_APCSMART_ENABLE

        case CMD_UPS_APCSMART:
          //
          //  ups.apcsmart[rxPin, txPin, command]
          //    command - HEX or ASCII
          //
          rc = getAPCSmartUPSMetric(argv[0], argv[1], (uint8_t*) _optarg[2], (uint8_t*) _dst);
          break;
#endif // FEATURE_UPS_APCSMART_ENABLE

#ifdef FEATURE_UPS_MEGATEC_ENABLE
        case CMD_UPS_MEGATEC:
          //
          //  ups.megatec[rxPin, txPin, command, fieldNumber]
          //    command - HEX or ASCII
          //
          rc = getMegatecUPSMetric(argv[0], argv[1], _optarg[2], argv[3], (uint8_t*) _dst);
          break;
#endif // FEATURE_UPS_APCSMART_ENABLE
        default:
          break;
      } // switch (cmdIdx) Non-I2C related commands block

      // ************************************************************************************************************************************
      //  I2C-related commands have additional processing
#ifdef TWI_USE
      // Otherwise - TWI interface can be reconfigured with new pins
      SoftTWI.reconfigure(argv[0], argv[1]);

      uint8_t i2CAddress;
      int16_t i2CRegister;

      i2CAddress = (uint8_t) argv[2];
      i2CRegister = ('\0' != *_optarg[3]) ? (int16_t) argv[3] : I2C_NO_REG_SPECIFIED;

      switch (cmdIdx) {
#ifdef FEATURE_I2C_ENABLE
        case CMD_I2C_SCAN:
          //
          //  I2C.scan[sdaPin, sclPin]
          //
          rc = scanI2C(&SoftTWI, &Network);
          break;

        case CMD_I2C_WRITE:
          //
          // i2c.write(sdaPin, sclPin, i2cAddress, register, length, data)
          //
          rc = writeValueToI2C(&SoftTWI, i2CAddress, i2CRegister, (uint8_t) argv[4], (uint32_t) argv[5]);
          break;

        case CMD_I2C_READ:
          //
          // i2c.read(sdaPin, sclPin, i2cAddress, register, length, numberOfReadings)
          // 
          rc = readValueFromI2C(&SoftTWI, i2CAddress, i2CRegister, (uint32_t*) &value, argv[4], (('\0' != *_optarg[5]) ? argv[5] : 0x00));
          break;

        case CMD_I2C_BITWRITE:
          //
          //  i2c.bitWrite(sdaPin, sclPin, i2cAddress, register, bitNumber, value)
          //
          rc = bitWriteToI2C(&SoftTWI, i2CAddress, i2CRegister, argv[4], argv[5]);
          break;

        case CMD_I2C_BITREAD:
          //
          //  i2c.bitRead(sdaPin, sclPin, i2cAddress, register, bit)
          //
          rc = bitReadFromI2C(&SoftTWI, i2CAddress, i2CRegister, argv[4], (uint8_t*) &value);
          break;
          
#endif // FEATURE_I2C_ENABLE

#ifdef FEATURE_BMP_ENABLE
        case CMD_BMP_PRESSURE:
          //
          //  BMP.Pressure[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getBMPMetric(&SoftTWI, i2CAddress, argv[3], argv[4], SENS_READ_PRSS, _dst);
          break;

        case CMD_BMP_TEMPERATURE:
          //
          // BMP.Temperature[sdaPin, sclPin, i2cAddress, overSampling]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getBMPMetric(&SoftTWI, i2CAddress, argv[3], argv[4], SENS_READ_TEMP, _dst);
          break;

#ifdef SUPPORT_BME280_INCLUDE
        case CMD_BME_HUMIDITY:
          //
          //  BME.Humidity[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getBMPMetric(&SoftTWI, i2CAddress, argv[3], argv[4], SENS_READ_HUMD, _dst);
          break;
#endif // SUPPORT_BME280_INCLUDE 
#endif // FEATURE_BMP_ENABLE  

#ifdef FEATURE_BH1750_ENABLE
        case CMD_BH1750_LIGHT:
          //
          //  BH1750.light[sdaPin, sclPin, i2cAddress, mode]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getBH1750Metric(&SoftTWI, i2CAddress, argv[3], SENS_READ_LUX, _dst);
          break;
#endif // FEATURE_BH1750_ENABLE

#ifdef FEATURE_INA219_ENABLE
        case CMD_INA219_BUSVOLTAGE:
          //
          //  INA219.BusVoltage[sdaPin, sclPin, i2cAddress, maxVoltage, maxCurrent]
          //
          rc = getINA219Metric(&SoftTWI, i2CAddress, argv[3], argv[4], SENS_READ_BUS_VOLTAGE, _dst);
          break;

        case CMD_INA219_CURRENT:
          //
          //  INA219.Current[sdaPin, sclPin, i2cAddress, maxVoltage, maxCurrent]
          //
          rc = getINA219Metric(&SoftTWI, i2CAddress, argv[3], argv[4], SENS_READ_DC, _dst);
          break;

        case CMD_INA219_POWER:
          //
          //  INA219.Power[sdaPin, sclPin, i2cAddress, maxVoltage, maxCurrent]
          //
          rc = getINA219Metric(&SoftTWI, i2CAddress, argv[3], argv[4], SENS_READ_POWER, _dst);
          break;

#endif // FEATURE_INA219_ENABLE

#ifdef FEATURE_PCF8574_LCD_ENABLE
        case CMD_PCF8574_LCDPRINT:
          //
          //  PCF8574.LCDPrint[sdaPin, sclPin, i2cAddress, lcdBacklight, lcdType, data]
          //
          rc = printToPCF8574LCD(&SoftTWI, i2CAddress, argv[3], argv[4], _optarg[5]);
          break;

#endif // FEATURE_PCF8574_LCD_ENABLE

#ifdef FEATURE_SHT2X_ENABLE
        case CMD_SHT2X_HUMIDITY:
          //
          //  SHT2X.Humidity[sdaPin, sclPin, i2cAddress]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getSHT2XMetric(&SoftTWI, i2CAddress, SENS_READ_HUMD, _dst);
          break;

        case CMD_SHT2X_TEMPERATURE:
          //
          //  SHT2X.Temperature[sdaPin, sclPin, i2cAddress]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getSHT2XMetric(&SoftTWI, i2CAddress, SENS_READ_TEMP, _dst);
          break;
#endif // FEATURE_SHT2X_ENABLE  

#ifdef FEATURE_SYSTEM_RTC_ENABLE
        case CMD_SET_LOCALTIME:
          //
          //  set.localtime[password, unixTimestamp, tzOffset]
          //  set.localtime must take unixTimestamp as UTC, because system.localtime command returns UTC too
          //
          if (!accessGranted) { break; }
          // i used as succes bool variable
          i = true;
#ifdef FEATURE_EEPROM_ENABLE
          // tzOffset is defined?
          if ('\0' != *_optarg[2]) {
            _netConfig->tzOffset = (int16_t) argv[2];
            // Save config to EEPROM
            i = saveConfigToEEPROM(_netConfig);
            if (i) {
              set_zone(_netConfig->tzOffset);
              rc = RESULT_IS_OK;
            }
          }
#endif // FEATURE_EEPROM_ENABLE

          // unixTimestamp option is given?
          if ('\0' != *_optarg[1] && i) {
            // tzOffset is defined and stored sucesfully
            if (setUnixTime(&SoftTWI, argv[1])) { rc = RESULT_IS_OK; }
          }
          break;
#endif // FEATURE_SYSTEM_RTC_ENABLE

#ifdef FEATURE_AT24CXX_ENABLE
        case CMD_AT24CXX_WRITE:
          //
          // AT24CXX.write[sdaPin, sclPin, i2cAddress, cellAddress, data]
          // 
          // i is half length of decoded byte array
          //i = (strlen(_optarg[4]) - 2) / 2;
          i = hstoba((uint8_t*) _dst, _optarg[4]);
          if (0 < i) {
            if (AT24CXXWrite(&SoftTWI, i2CAddress, argv[3], i, (uint8_t*) _dst)) { rc =  RESULT_IS_OK; }
          }
          break;

        case CMD_AT24CXX_READ:
          //
          // AT24CXX.read[sdaPin, sclPin, i2cAddress, cellAddress, length]
          // 
          if (AT24CXXRead(&SoftTWI, i2CAddress, argv[3], argv[4], (uint8_t*) _dst)) {
            // need to use _dst as uint8_t, because sometime autocast is fail and user can get wrong data
            uint8_t* _dstptr;
            _dstptr = (uint8_t*) _dst;
            Network.client.print("0x");
            DTSL( Serial.print("0x"); )
            for (i = 0; i < argv[4]; i++) {
              if (0x10 > _dstptr[i]) {
                Network.client.print("0");
                DTSL( Serial.print("0"); )
              }
              Network.client.print(_dstptr[i], HEX);
              DTSL( Serial.print(_dstptr[i], HEX); )
            }
            rc = RESULT_IS_PRINTED;
            _dstptr[0] = 0x00;
          } else {
            rc = RESULT_IS_FAIL;
          }
          break;

#endif // FEATURE_AT24CXX_ENABLE

#ifdef FEATURE_MAX44009_ENABLE
        case CMD_MAX44009_LIGHT:
          //
          //  MAX44009.light[sdaPin, sclPin, i2cAddress, mode, integrationTime]
          // 
          // 0x08 == MAX44009_INTEGRATION_TIME_6MS+1 - put to integrationTime parameter wrong value if no arg#4 given 
          // to kick getMAX44009Metric() to use auto measurement time
          rc = getMAX44009Metric(&SoftTWI, i2CAddress, argv[3], (('\0' == *_optarg[4]) ? 0x08 : argv[4]), SENS_READ_LUX, _dst);
          //rc = getMAX44009Metric(&SoftTWI, i2CAddress, argv[3], 0x08, SENS_READ_LUX, _dst);
          break;
#endif // FEATURE_MAX44009_ENABLE
      
      }  // switch (cmdIdx), I2C block
#endif // TWI_USE

#ifdef FEATURE_MODBUS_RTU_ENABLE
        case CMD_MODBUS_RTU_COILSREAD:
        //  Modbus.RTU.CoilsRead[rxPin, txPin, enablePin, slaveAddr, startAddr, ]
        // CMD_MODBUS_RTU_INPUTSREAD
        // CMD_MODBUS_RTU_COILSWRITE
        //rc = getModbusRTUMetric();
          //rc = getMAX44009Metric(&SoftTWI, i2CAddress, argv[3], 0x08, SENS_READ_LUX, _dst);
          break;
#endif // FEATURE_MODBUS_RTU_ENABLE

  } // switch (cmdIdx) .. default in "commands with options" block

finish:
  // Form the output buffer routine
  // DTS( Serial.print("[4] "); Serial.println(millis()); )
  // The rc is not printed or already placed in the buffer
  //Serial.print("rc: "); Serial.print(rc, HEX); Serial.println();

  if (RESULT_IS_PRINTED != rc) {
    switch (rc) {
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
      case RESULT_IS_SIGNED_VALUE:
        //  or rc value placed in 'value' variable and must be converted to C-string.
        ltoa((int32_t) value, _dst, 10);
        break;
      case RESULT_IS_UNSIGNED_VALUE:
        //  or rc value placed in 'value' variable and must be converted to C-string.
        ultoa((uint32_t) value, _dst, 10);
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
      case ZBX_NOTSUPPORTED:
        strcpy_P(_dst, PSTR((MSG_ZBX_NOTSUPPORTED)));
        break;
      default:
        // otherwise subroutine return unexpected value, need to check its source code
        strcpy_P(_dst, PSTR("Unexpected retcode"));
        break;
    }
    //  Push out the buffer to the client
    if (Network.client.connected()) {
      Network.client.println(_dst);
    }
    DTSL( Serial.println(_dst); )
  //   DTSM( Serial.print("[5] "); Serial.println(millis()); )
  }

  return cmdIdx;
}

