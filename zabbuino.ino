/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                            Use proper release Arduino IDE to avoid compilation errors , please.

                                                v1.6.11 and above is good

*/

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                   PROGRAMM FEATURES SECTION

  Please, refer to the "cfg_basic.h" file for enabling or disabling Zabbuino's features and refer to the "src/cfg_tune.h" to deep tuning.
  if connected sensors seems not work - first check setting in port_protect[], port_mode[], port_pullup[] arrays in I/O PORTS
  SETTING SECTION of "src/cfg_tune.h" file
*/
#include "src/dispatcher.h"
#include "plugin.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                           STARTUP SECTION
*/

// This is need to take system's VCC
#if defined(ARDUINO_ARCH_ESP8266)
ADC_MODE(ADC_VCC);
#endif


void setup() {

#ifdef SERIAL_USE
  DEBUG_PORT.begin(constSerialMonitorSpeed);
#endif // SERIAL_USE
  SPI.begin();

  __DMLL( DEBUG_PORT.print(FSH_P(constZbxAgentVersion)); DEBUG_PORT.println(FSH_P(STRING_wakes_up)); )
  sysMetrics.sysVCCMin = sysMetrics.sysVCCMax = getMcuVoltage();
  sysMetrics.sysRamFree = sysMetrics.sysRamFreeMin = getRamFree();
  pinMode(constStateLedPin, OUTPUT);
  pinMode(constUserFunctionButtonPin, INPUT_PULLUP);

#ifdef ADVANCED_BLINKING
  // blink on start
  //  blinkMore(6, 50, 500);
#endif
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                       GENERAL SECTION
*/
void loop() {
  uint8_t result = 0x00, needNetworkRelaunch = true, errorCode = ERROR_NONE;
  char incomingData;
  uint32_t processStartTime, processEndTime, prevPHYCheckTime, prevNetActivityTime, prevSysMetricGatherTime, clientConnectTime, netDebugPrintTime;

#if defined(FEATURE_USER_FUNCTION_PROCESSING)
  uint8_t  userFunctionButtonStatePrev;
  uint32_t userFunctionButtonChangeStateTime = 0x00, prevUserFuncCallTime = 0x00;
#endif

  request_t request;
  request.type = PACKET_TYPE_NONE;
  request.payloadByte = request.data;

  yield();
  NetworkClient netClient;
  NetworkServer netServer(constZbxAgentTcpPort);


  // 0. Init some libs to make system screen works if it enabled
#ifdef TWI_USE
  SoftTWI.begin(constDefaultSDAPin, constDefaultSCLPin);
#endif
#ifdef FEATURE_SYSTEM_RTC_ENABLE
  __DMLM( DEBUG_PORT.print(FSH_P(STRING_Init_system_RTC)); DEBUG_PORT.print(FSH_P(STRING_3xDot_Space)); )
  if (RESULT_IS_FAIL == initRTC(&SoftTWI)) {
    // sysMetrics.sysStartTimestamp already inited by 0x00 with memset() on start
    __DMLM( DEBUG_PORT.println(FSH_P(STRING_fail)); )
  } else {
    __DMLM( DEBUG_PORT.println(FSH_P(STRING_ok)); )
    // Get current timestamp from system RTC and store it into sysMetrics structure.
    // On RTC failure - do not use it
    if (RESULT_IS_FAIL == getUnixTime(&SoftTWI, (uint32_t*) &sysMetrics.sysStartTimestamp)) {
      sysMetrics.sysStartTimestamp = 0x00;
    }
  }
#endif

  // Run user function
  __USER_FUNCTION( initStageUserFunction(request.payloadByte); )

  // System load procedure
  // 1. Factory reset block
#ifdef FEATURE_EEPROM_ENABLE
  // factoryReset() return false on EEPROM saving fail or not executed
  factoryReset(constUserFunctionButtonPin, constUserFunctionButtonActiveState, sysConfig);
#endif // FEATURE_EEPROM_ENABLE

  // 2. Load configuration from EEPROM
  //

#ifdef FEATURE_EEPROM_ENABLE
  __DMLL( DEBUG_PORT.print(FSH_P(STRING_Config)); DEBUG_PORT.print(FSH_P(STRING_loading)); DEBUG_PORT.print(FSH_P(STRING_3xDot_Space)); )
  if (!loadConfigFromEEPROM(sysConfig)) {
    __DMLL( DEBUG_PORT.println(FSH_P(STRING_fail)); )
    // bad CRC detected, use default values for this run
    setConfigDefaults(sysConfig);
    if (!saveConfigToEEPROM(sysConfig)) {
      __DMLM( DEBUG_PORT.print(FSH_P(STRING_Config)); DEBUG_PORT.println(FSH_P(STRING_saving_error)); )
      __DMLL( DEBUG_PORT.println(FSH_P(STRING_Use_default_settings)); )
      // what to do on saving error?
    }
  } else {
    __DMLL( DEBUG_PORT.println(FSH_P(STRING_ok)); )
  }
#else // FEATURE_EEPROM_ENABLE
  __DMLM( DEBUG_PORT.println(FSH_P(STRING_EEPROM_disabled)); )
  __DMLL( DEBUG_PORT.println(FSH_P(STRING_Use_default_settings)); )
  // Use hardcoded values if EEPROM feature disabled
  setConfigDefaults(sysConfig);
#if defined(ARDUINO_ARCH_ESP8266) && defined(NETWORK_WIRELESS_ESP_NATIVE)
  setWifiDefaults();
#endif //defined(ARDUINO_ARCH_ESP8266)
#endif // FEATURE_EEPROM_ENABLE

  // 3. Forcing the system parameters in accordance with the user's compilation options & hardware set
  //
#ifdef FEATURE_PASSWORD_PROTECTION_FORCE
  sysConfig.useProtection = true;
#endif
#ifdef FEATURE_NET_DHCP_FORCE
  sysConfig.useDHCP = true;
#endif // FEATURE_NET_DHCP_FORCE

#if defined(FEATURE_SYSTEM_RTC_ENABLE)
#if defined(ARDUINO_ARCH_AVR)
  set_zone(sysConfig.tzOffset);
#endif // ARDUINO_ARCH_AVR
#endif

  // 4. Network initialization and starting
  //
#if defined FEATURE_ETHERNET_SHIELD_RESET_BUG_FIX
  // Let's have a some delay from the system start if Ethernet Shield turns on unstable
  processStartTime = millis();
  if (processStartTime < constEthernetShieldInitDelay) {
    delay(constEthernetShieldInitDelay - processStartTime);
  }
#endif
  // Call user function
  __USER_FUNCTION( netPrepareStageUserFunction(request.payloadByte); )
  // init() just make preset of Network object internal data. Real starting maken on relaunch()
  Network::init(sysConfig.macAddress, sysConfig.ipAddress, sysConfig.ipAddress, sysConfig.ipGateway, sysConfig.ipNetmask, sysConfig.useDHCP);
  //netServer.begin();

  __DMLL(
    //DEBUG_PORT.println(F("Wait for network warming up... "));
#if defined(FEATURE_SYSTEM_RTC_ENABLE)
    DEBUG_PORT.print(F("Timezone: ")); DEBUG_PORT.println(sysConfig.tzOffset, DEC);
#endif
    DEBUG_PORT.print(FSH_P(STRING_Password)); DEBUG_PORT.println(sysConfig.password, DEC);
  )

  // 5. Other system parts initialization
  //
  // I/O ports initialization. Refer to "I/O PORTS SETTING SECTION" in src/cfg_tune.h
  if (RESULT_IS_FAIL == initPinMode()) {
    __DMLM( DEBUG_PORT.println(FSH_P(STRING_IO_ports_presets_is_wrong)); )
  }

  // Prepare external interrupts info structure and so
#ifdef INTERRUPT_USE
  initExtInt();
#endif

  // Watchdog activation
#if defined(ARDUINO_ARCH_AVR)
  __WATCHDOG( wdt_enable(constWtdTimeout); )
#elif defined(ARDUINO_ARCH_ESP8266)
  wdt_enable(WDTO_8S);
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
  prevSysMetricGatherTime = prevPHYCheckTime = 0x00;
  prevNetActivityTime = netDebugPrintTime = clientConnectTime = millis();

  // 6. Enter to infinitive loop to serve incoming requests
  //
  // if no exist while() here - netProblemTime must be global or static - its will be 0 every loop() and time-related cases will be processeed abnormally
  // ...and while save some cpu ticks because do not call everytime from "hidden main()" subroutine, and do not init var, and so.
  //*****************************************************************************************************************************************************


  // Call user function
  __USER_FUNCTION( preLoopStageUserFunction(request.payloadByte); )
  __USER_FUNCTION( userFunctionButtonStatePrev = (constUserFunctionButtonActiveState == digitalRead(constUserFunctionButtonPin)); )

  parseRequest(CHAR_NULL, REINIT_ANALYZER, request);

  while (true) {
    // reset watchdog every loop
    __WATCHDOG( wdt_reset(); )
    yield();


#if defined(FEATURE_USER_FUNCTION_PROCESSING)
    // System Button pressing on runttime
    uint8_t userFunctionButtonState = (constUserFunctionButtonActiveState == digitalRead(constUserFunctionButtonPin));
    if (userFunctionButtonState != userFunctionButtonStatePrev) {
      userFunctionButtonStatePrev = userFunctionButtonState;
      // Cancel "wait for debounce"
      userFunctionButtonChangeStateTime = userFunctionButtonChangeStateTime ? 0x00 : millis();
    }

    if (userFunctionButtonChangeStateTime && (millis() - userFunctionButtonChangeStateTime > constUserFunctionButtonDebounceTime)) {
      userFunctionButtonState ? userFunctionButtonActivate(request.payloadByte) : userFunctionButtonDeactivate(request.payloadByte);
      userFunctionButtonChangeStateTime = 0x00;
    }
#endif


    // Gather internal metrics periodically
    if ((millis() - prevSysMetricGatherTime) > constSysMetricGatherPeriod) {
      // When FEATURE_SYSINFO_ENABLE is disabled, compiler can be omit gatherSystemMetrics() sub (due find no operators inside) and trow exception
#ifndef GATHER_METRIC_USING_TIMER_INTERRUPT
      gatherSystemMetrics();
#endif
      getMcuVoltage();
      prevSysMetricGatherTime = millis();
      // update millis() rollovers to measure uptime if no RTC onboard
      millisRollover();
    }


    // Turn off state led if no errors occured in the current loop.
    // Otherwise - make LED blinked or just turn on
    if (ERROR_NONE == errorCode) {
      digitalWrite(constStateLedPin, !constStateLedOn);
      sysMetrics.sysAlarmRisedTime = 0x00;
    } else {
      if (sysMetrics.sysAlarmRisedTime) {
        sysMetrics.sysAlarmRisedTime = millis();
      }

      // Call user function
      __USER_FUNCTION( alarmStageUserFunction(request.payloadByte, errorCode); )

#ifdef ON_ALARM_STATE_BLINK
      digitalWrite(constStateLedPin, (millis() % blinkSettings[errorCode].allTime < blinkSettings[errorCode].onTime) ? constStateLedOn : !constStateLedOn);
#else
      digitalWrite(constStateLedPin, constStateLedOn);
#endif
    } // if (ERROR_NONE == errorCode) ... else

    if (needNetworkRelaunch) {
      needNetworkRelaunch = false;
      // Network module lost address (was resetted) or was not init (first time run)
      // Relaunch will reset hardware and re-init its registers
      sysMetrics.netPHYReinits++;
      __DMLL( DEBUG_PORT.print(FSH_P(STRING_Network_module_reset_No)); DEBUG_PORT.println(sysMetrics.netPHYReinits); )
      // relaunch() returns code that can be used as status led blink type
      errorCode = Network::relaunch();
      if (ERROR_NONE == errorCode) {
#if defined (NETWORK_WIFI)
        delay(constPHYCheckInterval);
#endif
        netServer.begin();
        // relaunch() returns true if DHCP is OK or Static IP used
        __DMLL( Network::printNetworkInfo(); )
      }
    }

    //  return;

    // tick() subroutine is very important for UIPEthernet, and must be called often (every ~250ms). If ENC28J60 driver not used - this subroutine do nothing
    Network::tick();

    if ((millis() - prevPHYCheckTime) > constPHYCheckInterval) {
      //Serial.println("check");
      //phyCheckInterval = constPHYCheckInterval;
      // do not forget SPI.begin() or system is hangs up
      if ((millis() - netDebugPrintTime) > consNetDebugPrintInterval) {
        //__DMLL( Network::printPHYState(); )
        netDebugPrintTime = millis();
      }

      // Network hardware is ok?
      if (Network::isPhyOk()) {
        // Network hardware settings is the same that MCU needs?
        if (!Network::isPhyConfigured()) {
          // Need relaunch on next loop in hardware settings error detected
          needNetworkRelaunch = true;
        } else { // if (!Network::isPhyConfigured())
          // PHY is works properly
          result = Network::maintain();
          // Renew procedure finished with success
          switch (result) {
            case DHCP_CHECK_NONE:
            case DHCP_CHECK_RENEW_OK:
            case DHCP_CHECK_REBIND_OK:
              // No alarm blink need, network activity registred
              errorCode = ERROR_NONE;
              break;
            default:
              // Got some errors - blink with "DHCP problem message"
              //blinkType = constBlinkDhcpProblem;
              errorCode = ERROR_DHCP;
              __DMLM( DEBUG_PORT.println(FSH_P(STRING_DHCP_renew_problem_occured)); )
          } // switch (result)
        } // if (!Network::isPhyConfigured())
      } else { // if (Network::isPhyOk())
        // PHY is disconnected or rise any error flag
        Serial.println("Need to rlnch");
        needNetworkRelaunch = true;
        // Right errorCode will be taken on relaunch()
      } // if (Network::isPhyOk())
      prevPHYCheckTime = millis();
    }

    // No DHCP problem found, but no data recieved or network activity for a long time
    if (ERROR_NONE == errorCode && (constNetIdleTimeout <= (millis() - prevNetActivityTime))) {
      errorCode = ERROR_NO_NET_ACTIVITY;
    }

    //*********************************************

#ifdef FEATURE_SERIAL_LISTEN_TOO
    // !!! Need to drop slow serial connection too
    // Network connections will processed if no data in Serial buffer exist
    if (DEBUG_PORT.available() <= 0) {
#endif
      if (!netClient) {
        // accept() returns client with "Connected but not send data" state and system must have more stability on this
#if defined(NETWORK_ETH_WIZNET)
        netClient = netServer.accept();
#else
        netClient = netServer.available();
#endif
        if (!netClient) {
          // Call "loop stage" user function screen every constRenewSystemDisplayInterval only if no connection exist, because that function can modify cBuffer content
          // and recieved data can be corrupted
#if defined(FEATURE_USER_FUNCTION_PROCESSING)
          if (constUserFunctionCallInterval <= (uint32_t) (millis() - prevUserFuncCallTime)) {
            loopStageUserFunction(request.payloadByte);
            prevUserFuncCallTime = millis();
          }
#endif
          // Jump to new round of main loop
          continue;
        }
        // reinit analyzer because previous session can be dropped or losted
        parseRequest(CHAR_NULL, REINIT_ANALYZER, request);
        clientConnectTime = millis();
        // ToDo: add remoteIp() output
        __DMLH( DEBUG_PORT.print(FSH_P(STRING_New)); DEBUG_PORT.print(FSH_P(STRING_client_No)); DEBUG_PORT.println(netClient); )
      }

      // Client will be dropped if its connection so slow. Then round must be restarted.
      if (constNetSessionTimeout <= (uint32_t) (millis() - clientConnectTime)) {
        __DMLH( DEBUG_PORT.print(FSH_P(STRING_Drop)); DEBUG_PORT.print(FSH_P(STRING_client_No)); DEBUG_PORT.println(netClient); )
        netClient.stop();
        // Jump to new round of main loop
        continue;
      }

      // Network data can be splitted to a number frames and ethClient.available() can return 0 on first frame processing. Need to ignore it.
      if (!netClient.available()) {
        continue;
      }
      incomingData = netClient.read();
#ifdef FEATURE_SERIAL_LISTEN_TOO
    } else {
      // If in Serial buffer is exist just read it
      incomingData = DEBUG_PORT.read();
    }
#endif

    result = parseRequest(incomingData, NO_REINIT_ANALYZER, request);
    // result is true if analyzeStream() do not finished and need more data
    // result is false if EOL or trailing char detected or there no room in buffer or max number or args parsed...
    if (true == result) {
      continue;
    }
    // ethClient.connected() returns true even client is disconnected, but leave the data in the buffer.
    // Data can be readed, command will executed, but why get answer? If no recipient - why need to load MCU?
    // Will be better do nothing if client is disconnected while analyzing is finished
    // But checking must be disable for commands which coming in from the DEBUG_PORT. Otherwise commands will be never executed if no active network client exist.
#ifndef FEATURE_SERIAL_LISTEN_TOO
    if (!netClient.connected()) {
      continue;
    }
#endif
    // Fire up State led, than will be turned off on next loop
    digitalWrite(constStateLedPin, constStateLedOn);
    processStartTime = millis();
    __DMLM( uint32_t ramBefore = getRamFree(); )
    //__DMLL( DEBUG_PORT.print(cBuffer); DEBUG_PORT.print(FSH_P(STRING_right_arrow)); ) // zbxd\1 is broke this output
    sysMetrics.sysCmdLast = executeCommand(netClient, sysConfig, request);

#ifdef FEATURE_REMOTE_COMMANDS_ENABLE
    // When system.run[] command is recieved, need to run another command, which taken from option #0 by cmdIdx() sub
    if (RESULT_IS_NEW_COMMAND == sysMetrics.sysCmdLast) {
      uint16_t k = 0x00;
      parseRequest(CHAR_NULL, REINIT_ANALYZER, request);
      // simulate command recieving to properly string parsing
      // option #0 is moved early to the buffer in the executeCommand()
      while (parseRequest(request.payloadByte[k], NO_REINIT_ANALYZER, request)) {
        k++;
      }
      __DMLL( DEBUG_PORT.print(request.command); DEBUG_PORT.print(FSH_P(STRING_right_arrow)); )
      sysMetrics.sysCmdLast = executeCommand(netClient, sysConfig, request);
    }
#endif
    processEndTime = millis();
    // use processEndTime as processDurationTime
    processEndTime = processEndTime - processStartTime ;
    __DMLM( DEBUG_PORT.print(F("Spended: ")); DEBUG_PORT.print(processEndTime);
            DEBUG_PORT.print(F(" ms, ")); DEBUG_PORT.print((ramBefore - getRamFree()));
            DEBUG_PORT.println(F(" memory bytes"));
          )
    // Correct internal runtime metrics if need
    if (sysMetrics.sysCmdTimeMax < processEndTime) {
      sysMetrics.sysCmdTimeMax = processEndTime;
      sysMetrics.sysCmdTimeMaxN = sysMetrics.sysCmdLast;
    }

    // Wait some time to finishing answer send, close connection, and restart network activity control cycle
    //delay(constNetStabilizationDelay);
    // Actually Ethernet lib's flush() do nothing, but UIPEthernet flush() free ENC28J60 memory blocks where incoming (?) data stored
    netClient.flush();
    netClient.stop();
#ifdef FEATURE_SERIAL_LISTEN_TOO
    // Flush the incoming Serial buffer by reading because Serial object have no clear procedure.
    flushStreamRXBuffer(&Serial, 1000UL, false);
    parseRequest(CHAR_NULL, REINIT_ANALYZER, request);
#endif
    sysMetrics.sysCmdLastExecTime = prevPHYCheckTime = prevNetActivityTime = millis();
    //blinkType = constBlinkNope;
    errorCode = ERROR_NONE;
  } // while(true)
}

/* ****************************************************************************************************************************


**************************************************************************************************************************** */
static int16_t executeCommand(Stream& _netClient, netconfig_t& _sysConfig, request_t& _request) {
  int8_t rc;
  uint8_t accessGranted = false;
  uint16_t i;
  uint8_t cmdIdx;
  // duration option in the tone[] command is ulong
  // Zabbix use 64-bit numbers, but we can use only int32_t range. Error can be occurs on ltoa() call with value > long_int_max
  int32_t value = 0x00;
  uint32_t payloadLength;

  cmdIdx  = arraySize(commands);

  __DMLD( DEBUG_PORT.print(F("Request type: ")); )

  // Is need to returns Error if packet's data size is big or not equal to length taken from header?
  switch (_request.type) {
    case PACKET_TYPE_PLAIN: {
        // Create fake command CMD_ZBX_NOPE to return ZBX_NOTSUPPORTED
        // _request.command point to start of allowed data's space which used as output buffer
        _request.command = (char*) _request.data;
        _request.dataFreeSize = sizeof(_request.data);
        __DMLD( DEBUG_PORT.println(FSH_P(STRING_Plain_text)); )
        break;
      }

    case PACKET_TYPE_ZABBIX: {
        _request.command = (char*) &_request.data[ZBX_HEADER_LENGTH];
        _request.dataFreeSize = sizeof(_request.data) - ZBX_HEADER_LENGTH;
        __DMLD( DEBUG_PORT.println(FSH_P(STRING_Zabbix)); )
        break;
      }

    case PACKET_TYPE_NONE:
    default: {
        *_request.command = CHAR_NULL;
        __DMLD( DEBUG_PORT.println(F("None")); )
        break;
      }
  }

  __DMLL(DEBUG_PORT.print(F("Request cmd: '")); DEBUG_PORT.print(_request.command); DEBUG_PORT.println('\'');  )

  // Search specified command index in the list of implemented functions
  rc = 0x01;
  while (cmdIdx && (0x00 != rc)) {
    yield();
    cmdIdx--;
    PGM_P cmdName = (PGM_P) pgm_read_dword(&(commands[cmdIdx].name));
    rc = strcmp_P(_request.command, cmdName);
    __DMLD( DEBUG_PORT.print(F("#")); DEBUG_PORT.print(FSH_P(STRING_HEX_Prefix));  DEBUG_PORT.print(cmdIdx , HEX); DEBUG_PORT.print(FSH_P(STRING_right_arrow)); DEBUG_PORT.println(FSH_P(cmdName)); )
  }
  cmdIdx = pgm_read_byte(&(commands[cmdIdx].idx));
  // If no suitable command found - do nothing, jump to the finish where show result ZBX_NOTSUPPORTED
  if (0x00 >= cmdIdx) {
    rc = ZBX_NOTSUPPORTED;
    goto finish;
  }

  rc = RESULT_IS_FAIL;
  sysMetrics.sysCmdCount++;

  __DMLM( DEBUG_PORT.print(FSH_P(STRING_Execute_command_No)); DEBUG_PORT.print(cmdIdx, HEX); DEBUG_PORT.print(FSH_P(STRING_right_arrow)); DEBUG_PORT.println(_request.command); )

  // ***************************************************************************************************************
  // If command have no options - it must be run immedately
  switch (cmdIdx) {
    //  case  CMD_ZBX_NOPE:
    //        break;

    case CMD_ZBX_AGENT_PING: {
        //
        //   agent.ping
        //
        rc = RESULT_IS_OK;
        goto finish;
      }

    case CMD_ZBX_AGENT_HOSTNAME: {
        //
        //   agent.hostname
        //
        strcpy(_request.payloadChar, _sysConfig.hostname);
        rc = RESULT_IS_BUFFERED;
        goto finish;
      }

    case CMD_ZBX_AGENT_VERSION: {
        //
        //  agent.version
        //
        strcpy_P(_request.payloadChar, constZbxAgentVersion);
        rc = RESULT_IS_BUFFERED;
        goto finish;
      }

    case CMD_SYSTEM_UPTIME: {
        //
        //  system.uptime
        //
        // Returns uptime in seconds
        value  = ((uint32_t) millisRollover() * UINT32_MAX + millis()) / 1000UL;
#ifdef FEATURE_SYSTEM_RTC_ENABLE
        // Just rewrite millises uptime by another, which taken from system RTC
        if (0x00 != sysMetrics.sysStartTimestamp) {
          SoftTWI.begin(constSystemRtcSDAPin, constSystemRtcSCLPin);
          if (getUnixTime(&SoftTWI, (uint32_t*) &value)) {
            value = value - sysMetrics.sysStartTimestamp;
          }
        }
#endif
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }

#ifdef FEATURE_SYSINFO_ENABLE
    case CMD_SYSTEM_HW_CHASSIS: {
        //
        //  system.hw.chassis
        //
        strcpy_P(_request.payloadChar, PSTR(BOARD));
        rc = RESULT_IS_BUFFERED;
        goto finish;
      }

    case CMD_NET_PHY_NAME: {
        //
        //  net.phy.name
        //
        strcpy_P(_request.payloadChar, PSTR(PHY_MODULE_NAME));
        rc = RESULT_IS_BUFFERED;
        goto finish;
      }

    case CMD_NET_PHY_REINITS: {
        //
        //  net.phy.reinits
        //
        value = sysMetrics.netPHYReinits;
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }

    case CMD_SYS_RAM_FREE: {
        //
        //  sys.ram.free
        //
        //  That metric must be collected periodically to avoid returns always same data
#if defined(ARDUINO_ARCH_AVR)
        // Without ATOMIC_BLOCK block using sysMetrics.sysRamFree variable can be changed in interrupt on reading
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#endif
        value = sysMetrics.sysRamFree;
#if defined(ARDUINO_ARCH_AVR)
        }
#endif
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }

    case CMD_SYS_RAM_FREEMIN: {
        //
        //  sys.ram.freemin
        //
#if defined(ARDUINO_ARCH_AVR)
        // Without ATOMIC_BLOCK block using sysMetrics.sysRamFreeMin variable can be changed in interrupt on reading
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#endif
          value = sysMetrics.sysRamFreeMin;
#if defined(ARDUINO_ARCH_AVR)
        }
#endif
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }
#endif

    case CMD_SYS_VCC: {
        //
        // sys.vcc
        //
        // Take VCC
        value = getMcuVoltage();
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }

    case CMD_SYS_VCCMIN: {
        //
        // sys.vccMin
        //
        //value = sysMetrics[IDX_METRIC_SYS_VCCMIN];
        value = sysMetrics.sysVCCMin;
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }

    case CMD_SYS_VCCMAX: {
        //
        // sys.vccMax
        //
        value = sysMetrics.sysVCCMax;
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }

#ifdef FEATURE_SYSINFO_ALL_ENABLE
    case CMD_SYS_ALL: {
        //
        //  sys.all
        //
        //ultoa(sysMetrics.sysCmdTimeMaxN, , 16);
        rc = getSystemAllInfo((sysmetrics_t&)sysMetrics, _request.payloadChar, _request.dataFreeSize);
        goto finish;
      }
#endif

#ifdef FEATURE_SYSTEM_RTC_ENABLE

    case CMD_SYSTEM_LOCALTIME: {
        //
        //  system.localtime
        //  Zabbix wants UTC as localtime
        //
        //  If "system.localtime" returns fail - try to use "set.localtime" first. May be battery or vcc voltage is low.
        //  System do not kickstart RTC if any problem detected to avoid taking random time and unexpected behaviour
        //
#if defined(ARDUINO_ARCH_AVR)
        if (RESULT_IS_OK == getUnixTime(&SoftTWI, (uint32_t*) &value)) {
          rc = RESULT_IS_UNSIGNED_VALUE;
        }
#elif defined(ARDUINO_ARCH_ESP8266)
        rc = ZBX_NOTSUPPORTED;
#endif //#if defined(ARDUINO_ARCH_AVR)

        goto finish;
      }
#endif // FEATURE_SYSTEM_RTC_ENABLE
  } // switch (cmdIdx) part #1

  // ***************************************************************************************************************
  // Command with options take more time
  // batch convert args to number values
  i = arraySize(_request.argv);
  while (i) {
    i--;
    _request.argv[i] = (NULL == _request.args[i]) ? 0x00 : strtol(_request.args[i], NULL, 0);
    __DMLH(
      DEBUG_PORT.print(F("argv[")); DEBUG_PORT.print(i); DEBUG_PORT.print(F("] => \""));
    if (_request.args[i]) {
    DEBUG_PORT.print(_request.args[i]);
    } else {
      DEBUG_PORT.print(F("<null>"));
    }
    DEBUG_PORT.print(F("\" => ")); DEBUG_PORT.println(_request.argv[i]);
    )
  }

  // Check rights for password protected action
  accessGranted = (!_sysConfig.useProtection || (uint32_t) _request.argv[0x00] == _sysConfig.password);

  switch (cmdIdx) {
#ifdef FEATURE_USER_FUNCTION_PROCESSING
    case CMD_USER_RUN: {
        //
        //  user.run[option#0, option#1, option#2, option#3, option#4, option#5]
        // user.run[0xA0,14]
        //
        rc = executeCommandUserFunction(_request.payloadByte, _request.args, _request.argv, &value);
        goto finish;
      }
#endif // FEATURE_USER_FUNCTION_PROCESSING

#ifdef FEATURE_REMOTE_COMMANDS_ENABLE
    case CMD_SYSTEM_RUN: {
        //
        //  system.run["newCommand"]
        //
        if (!_request.args[0x00]) {
          goto finish;
        }
        char *ptrOption, *ptrPayload;
        ptrOption = _request.args[0x00];
        ptrPayload = _request.payloadChar;
        while (*ptrOption) {
          *ptrPayload = *ptrOption;
          ptrPayload++; ptrOption++;
        }
        *ptrPayload = '\n';
        // immediately return RESULT_IS_NEW_COMMAND to re-run executeCommand() with new command
        return RESULT_IS_NEW_COMMAND;
        goto finish;
      }
#endif // FEATURE_REMOTE_COMMANDS_ENABLE

#ifdef FEATURE_ARDUINO_BASIC_ENABLE
    case CMD_ARDUINO_ANALOGWRITE: {
        //
        //  analogWrite[pin, value]
        //
        if (! isSafePin(_request.argv[0x00])) {
          goto finish;
        }
        analogWrite(_request.argv[0x00], _request.argv[0x01]);
        rc = RESULT_IS_OK;
        goto finish;
      }

    case CMD_ARDUINO_ANALOGREAD: {
        //
        //  analogRead[pin, analogReferenceSource, mapToLow, mapToHigh]
        //
#ifdef FEATURE_AREF_ENABLE
        // change source of the reference voltage if its given
        if (_request.args[0x00]) {
          analogReference(_request.argv[0x00]);
          delayMicroseconds(2000);
        }
#endif
        if (! isSafePin(_request.argv[0x00])) {
          goto finish;
        }
        value = analogRead(_request.argv[0x00]);
        if (_request.args[0x02] && _request.args[0x03]) {
          value = map(value, constAnalogReadMappingLowValue, constAnalogReadMappingHighValue, _request.argv[0x02], _request.argv[0x03]);
        }
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;

      }

    case CMD_ARDUINO_DELAY: {
        //
        //  delay[time]
        //
        if (!_request.args[0x00]) {
          goto finish;
        }
        delay(_request.argv[0x00]);
        rc = RESULT_IS_OK;
        goto finish;
      }

    case CMD_ARDUINO_DIGITALWRITE: {
        //
        //  digitalWrite[pin, value]
        //
        if (! isSafePin(_request.argv[0x00])) {
          goto finish;
        }
        // turn on or turn off logic on pin

        pinMode(_request.argv[0x00], OUTPUT);
        digitalWrite(_request.argv[0x00], !!_request.argv[0x01]);

        rc = RESULT_IS_OK;
        goto finish;
      }

    case CMD_ARDUINO_DIGITALREAD: {
        //
        //  digitalRead[pin, internal_pullup]
        //
        if (! isSafePin(_request.argv[0x00])) {
          goto finish;
        }
        pinMode(_request.argv[0x00], (_request.argv[0x01]) ? INPUT_PULLUP : INPUT);
        value = digitalRead(_request.argv[0x00]);
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }
#endif // FEATURE_ARDUINO_BASIC_ENABLE
#ifdef FEATURE_AREF_ENABLE
    case CMD_ARDUINO_ANALOGREFERENCE: {
        //
        //  analogReference[source]
        //
        if (_request.args[0x00]) {
          analogReference(_request.argv[0x00]);
          delayMicroseconds(2000);
        }
        rc = RESULT_IS_OK;
        goto finish;
      }
#endif
#ifdef FEATURE_TONE_ENABLE
    case CMD_ARDUINO_TONE: {
        //
        //  tone[pin, frequency, duration]
        //
        if (! isSafePin(_request.argv[0x00])) {
          goto finish;
        }

        // duration is given?
        if (_request.args[0x02]) {
          tone(_request.argv[0x00], _request.argv[0x01], _request.argv[0x02]);
        } else {
          tone(_request.argv[0x00], _request.argv[0x01]);
        }
        rc = RESULT_IS_OK;
        goto finish;
      }

    case CMD_ARDUINO_NOTONE: {
        //
        //  noTone[pin]
        //
        if (! isSafePin(_request.argv[0x00])) {
          goto finish;
        }
        noTone(_request.argv[0x00]);
        rc = RESULT_IS_OK;
        goto finish;
      }
#endif

#ifdef FEATURE_RANDOM_ENABLE
    case CMD_ARDUINO_RANDOMSEED: {
        //
        //  randomSeed[value]
        //
        randomSeed(_request.args[0x00] ? ((uint32_t) _request.argv[0x00]) : millis());
        rc = RESULT_IS_OK;
        goto finish;
      }

    case CMD_ARDUINO_RANDOM: {
        //
        //  random[min, max]
        //
        //  !! random return long
        value = (_request.args[0x00] && _request.args[0x01]) ? random((uint32_t) _request.argv[0x00], (uint32_t)_request.argv[0x01]) : random( _request.args[0x00] ? (uint32_t) _request.argv[0x00] : millis());
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }
#endif // FEATURE_RANDOM_ENABLE

#ifdef FEATURE_EEPROM_ENABLE
    case CMD_SET_HOSTNAME: {
        //
        //  set.hostname[password, hostname]
        //
        // _request.args[0x01] is not NULL if argument #2 given
        if (!accessGranted || !_request.args[0x01]) {
          goto finish;
        }

        strncpy(_sysConfig.hostname, _request.args[0x01], constAgentHostnameMaxLength);
        // strncpy() can do not copy trailing \0
        _sysConfig.hostname[constAgentHostnameMaxLength] = CHAR_NULL;
        rc = RESULT_IS_UNSTORED_IN_EEPROM;
        goto finish;
      }

    case CMD_SET_PASSWORD: {
        //
        //  set.password[oldPassword, newPassword]
        //
        if (!accessGranted || !_request.args[0x01]) {
          goto finish;
        }

        // take new password from argument #2
        _sysConfig.password = _request.argv[0x01];
        rc = RESULT_IS_UNSTORED_IN_EEPROM;
        goto finish;
      }

    case CMD_SET_SYSPROTECT: {
        //
        //  set.sysprotect[password, protection]
        //
        if (!accessGranted || !_request.args[0x01]) {
          goto finish;
        }

        _sysConfig.useProtection = (1 == _request.argv[0x01]) ? true : false;
        rc = RESULT_IS_UNSTORED_IN_EEPROM;
        goto finish;
      }

    case CMD_SET_NETWORK: {
        //
        //  set.network[password, useDHCP, macAddress, ipAddress, ipNetmask, ipGateway]
        //
        if (!accessGranted) {
          goto finish;
        }

        uint8_t success = true;
        netconfig_t newConfig;
        memcpy((uint8_t*) &newConfig, (uint8_t*) &_sysConfig, sizeof(newConfig));
        // useDHCP flag coming from argument#1 and must be numeric (boolean) - 1 or 0,
        // argv[0x00] data contain in payload[_argOffset[0x01]] placed from _argOffset[0x00]
        newConfig.useDHCP = !!_request.argv[0x01];
        // ip, netmask and gateway have one size - 4 byte
        // take 6 bytes from second argument of command and use as new MAC-address
        // if convertation is failed (sub return -1) variable must be falsed too via logic & operator
        success = (success) ? (sizeof(newConfig.macAddress) == hstoba((uint8_t *) &newConfig.macAddress, _request.args[0x02])) : false;
        Serial.println("=== D4 ===");
        // If string to which point optarg[0x03] can be converted to valid NetworkAddress - just do it.
        // Otherwize (string can not be converted) _sysConfig.ipAddress will stay untouched;
        success = (success) ? (strToNetworkAddress(_request.args[0x03], newConfig.ipAddress)) : false;
        Serial.println("=== D5 ===");
        success = (success) ? (strToNetworkAddress(_request.args[0x04], newConfig.ipNetmask)) : false;
        Serial.println("=== D6 ===");
        success = (success) ? (strToNetworkAddress(_request.args[0x05], newConfig.ipGateway)) : false;
        Serial.println("=== D7 ===");
        // if any convert operation failed - just do not return "need to eeprom write" return code
        if (success) {
          rc = (saveConfigToEEPROM(newConfig)) ? RESULT_IS_OK : DEVICE_ERROR_EEPROM_CORRUPTED;
        }
        goto finish;
      }

    case CMD_SET_WIFI: {
        //
        //  set.wifi[password, ssid, passphrase]
        //
#if defined(ARDUINO_ARCH_AVR)
        rc = ZBX_NOTSUPPORTED;
#elif defined(ARDUINO_ARCH_ESP8266)

        if (!accessGranted) {
          goto finish;
        }

        uint8_t success = true;
        char* wifiSsid = _request.args[0x01];
        char* wifiPsk  = _request.args[0x02];
        uint8_t len;

        // No SSID specified
        if (!wifiSsid) {
          goto finish;
        }
        len = strlen(wifiSsid);
        success = success ? (0x00 < len && 0x20 >= len) : false;
        // strlen  & etc. is reboot system with nullptr argument
        if (wifiPsk) {
          len = strlen(wifiPsk);
          success = success ? (0x08 <= len && 0x40 >= len) : false;
        }
        if (success) {
          // Just save config
          NetworkTransport.begin(wifiSsid, wifiPsk, 0x00, NULL, false);
          rc = RESULT_IS_OK;
        }
#endif
        goto finish;
      }

#endif // FEATURE_EEPROM_ENABLE

#ifdef FEATURE_SYSTEM_RTC_ENABLE
    case CMD_SET_LOCALTIME: {
        //
        //  set.localtime[password, unixTimestamp, tzOffset]
        //  set.localtime must take unixTimestamp as UTC, because system.localtime command returns UTC too
        //
#if defined(ARDUINO_ARCH_AVR)

        if (!accessGranted) {
          goto finish;
        }
        uint8_t success = true;
#ifdef FEATURE_EEPROM_ENABLE
        // tzOffset is defined?
        if (_request.args[0x02]) {
          _sysConfig.tzOffset = (int16_t) _request.argv[0x02];
          // Save config to EEPROM
          success = saveConfigToEEPROM(_sysConfig);
          if (success) {
            set_zone(_sysConfig.tzOffset);
            rc = RESULT_IS_OK;
          }
        }
#endif // FEATURE_EEPROM_ENABLE

        // unixTimestamp option is given?
        if (_request.args[0x01] && success) {
          // tzOffset is defined and stored sucesfully
          if (setUnixTime(&SoftTWI, _request.argv[0x01])) {
            rc = RESULT_IS_OK;
          }
        }
#elif defined(ARDUINO_ARCH_ESP8266)
        rc = ZBX_NOTSUPPORTED;
#endif //#if defined(ARDUINO_ARCH_AVR)
        goto finish;
      }
#endif // FEATURE_SYSTEM_RTC_ENABLE

    case CMD_SYS_PORTWRITE: {
        //
        //  portWrite[port, value]
        //
        // 'a' used because tolower() used to args saving while parsing
        // PortA have index 1, not 0; PortB is 2, PortC is 3...
#if defined(ARDUINO_ARCH_AVR)
        uint8_t portNo = *_request.args[0x00] - 'a' + 0x01;
        rc = writeToPort(portNo, _request.argv[0x01]);
#elif defined(ARDUINO_ARCH_ESP8266)
        rc = ZBX_NOTSUPPORTED;
#endif // #if defined(ARDUINO_ARCH_AVR)
        goto finish;
      }

#ifdef FEATURE_SHIFTOUT_ENABLE
    case CMD_SYS_SHIFTOUT: {
        //
        //  shiftOut[dataPin, clockPin, latchPin, bitOrder, compressionType, data]
        //

        uint8_t latchUsed = isSafePin(_request.argv[0x02]);
        if (isSafePin(_request.argv[0x00]) &&  isSafePin(_request.argv[0x01])) {
          if (latchUsed) {
            pinMode(_request.argv[0x02], OUTPUT);
            digitalWrite(_request.argv[0x02], LOW);
          }
          rc = shiftOutAdvanced(_request.argv[0x00], _request.argv[0x01], _request.argv[0x03], _request.argv[0x04], (uint8_t*)_request.args[0x05]);
          if (latchUsed) {
            digitalWrite(_request.argv[0x02], HIGH);
          }
        }
        goto finish;
      }
#endif

#ifdef FEATURE_WS2812_ENABLE
    case CMD_WS2812_SENDRAW:
      //
      //  WS2812.sendRaw[dataPin, compressionType, data]
      //  !!! need to increase ARGS_PART_SIZE, because every encoded LED color take _six_ HEX-chars => 10 leds stripe take 302 (2+50*6) byte of incoming buffer only
      //
      // Tested on ATmega328@16 and 8 pcs WS2812 5050 RGB LED bar
      if (isSafePin(_request.argv[0x00])) {
        // 4-th param equal 0x00 mean that buffer not contain raw color bytes and must be prepared (converted from "0xABCDEF.." string)
        // true for 800Khz (2812) and false for 400Khz (2811)
        uint8_t _bitstream800KHz = (_request.args[0x01]) ? ((400 == _request.argv[0x01]) ? false : true) : true;
        rc = WS281xOut(_request.argv[0x00], _bitstream800KHz, _request.argv[0x02], (uint8_t*) _request.args[0x03], 0x00);
      }
      goto finish;
#endif // FEATURE_WS2812_ENABLE

    case CMD_SYS_REBOOT:
      //
      //  reboot[password]
      //
      if (accessGranted) {
        rc = RESULT_IS_SYSTEM_REBOOT_ACTION;
      }
      goto finish;

#ifdef FEATURE_SYSINFO_ENABLE
    case CMD_SYSTEM_HW_CPU: {
        //
        //  system.hw.cpu[metric]
        //
        rc = RESULT_IS_BUFFERED;
        if (0x00 == strcmp_P(_request.args[0x00], PSTR("id"))) {
          getMcuIdAsHexString(_request.payloadChar);
        } else if (0x00 == strcmp_P(_request.args[0x00], PSTR("freq"))) {
          // Return back CPU frequency
          value = getMcuFreq();
          rc = RESULT_IS_UNSIGNED_VALUE;
        } else if (0x00 == strcmp_P(_request.args[0x00], PSTR("model"))) {
#if defined(ARDUINO_ARCH_AVR)
          getMcuModelAsHexString(_request.payloadChar);
#elif defined(ARDUINO_ARCH_ESP8266)
          rc = ZBX_NOTSUPPORTED;
#endif //#if defined(ARDUINO_ARCH_AVR)          
        } else {
          // Return back CPU name
          strcpy_P(_request.payloadChar, PSTR(_CPU_NAME_));
        }
        goto finish;
      }

    case CMD_SYS_CMD_COUNT: {
        //
        //  sys.cmd.count
        //
        if (_request.argv[0x00]) {
          sysMetrics.sysCmdCount = 0x00;
        }
        value = sysMetrics.sysCmdCount;
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }

    case CMD_SYS_CMD_TIMEMAX: {
        //
        //  sys.cmd.timemax[resetCounter]
        //
        if (_request.args[0x00]) {
          sysMetrics.sysCmdTimeMax = sysMetrics.sysCmdTimeMaxN = 0x00;
        }
        value = sysMetrics.sysCmdTimeMax;
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }
#endif // FEATURE_SYSINFO_ENABLE

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
    case CMD_EXTINT_COUNT: {
        //
        //  extInt.count[intPin, mode]
        //
        //  Unfortunately, (rc == RESULT_IS_UNSIGNED_VALUE && value == 0) and (rc == RESULT_IS_FAIL) are looks equal for zabbix -> '0'
        //
#if defined(ARDUINO_ARCH_AVR)
        if (!isSafePin(_request.argv[0x00])) {
          goto finish;
        }
        rc = manageExtInt(_request.argv[0x00], _request.argv[0x01], (uint32_t*) &value);
#elif defined(ARDUINO_ARCH_ESP8266)
        rc = ZBX_NOTSUPPORTED;
#endif //#if defined(ARDUINO_ARCH_AVR)
        goto finish;
      }
#endif // FEATURE_EXTERNAL_INTERRUPT_ENABLE
#ifdef FEATURE_OW_ENABLE
    case CMD_OW_SCAN: {
        //
        //  OW.scan[pin]
        //
        if (!isSafePin(_request.argv[0x00])) {
          goto finish;
        }
        value = scanOneWire(_request.argv[0x00], _request.payloadByte, _request.dataFreeSize);
        if (value) {
          rc = RESULT_IS_BUFFERED;
        }
        goto finish;
      }
#endif // FEATURE_ONEWIRE_ENABLE
#ifdef FEATURE_DS18X20_ENABLE
    case CMD_DS18X20_TEMPERATURE: {
        //
        //  DS18x20.temperature[pin, resolution, id]
        //
        if (!isSafePin(_request.argv[0x00])) {
          goto finish;
        }
        uint8_t owAddress[8] = {0x00};
        // Convert sensor ID (if its given) from HEX string to byte array (DeviceAddress structure) and validate (sub not finished) it.
        // Sensor ID is equal DeviceAddress.
        // if convertation not successfull or ID not valid - return DEVICE_ERROR_WRONG_ID
        if ((_request.args[0x02]) && (ONEWIRE_ID_SIZE != hstoba(owAddress, _request.args[0x02]))) {
          rc = DEVICE_ERROR_WRONG_ID;
        } else {
          rc = getDS18X20Metric(_request.argv[0x00], _request.argv[0x01], owAddress, &value);
        }
        goto finish;
      }
#endif // FEATURE_DS18X20_ENABLE

#ifdef FEATURE_MHZXX_PWM_ENABLE
    case CMD_MHZXX_PWM_CO2:
      //
      //  MHZxx.PWM.CO2[pin, range]
      //
      if (isSafePin(_request.argv[0x00])) {
        rc = getMHZxxMetricPWM(_request.argv[0x00], _request.argv[0x01], &value);
      }
      goto finish;
#endif // FEATURE_MHZXX_PWM_ENABLE

#ifdef FEATURE_DHT_ENABLE
    case CMD_DHT_HUMIDITY:
      //
      //  DHT.humidity[pin, model]
      //
      if (!isSafePin(_request.argv[0x00])) {
        rc = DEVICE_ERROR_CONNECT;
      } else {
        rc = getDHTOneMetric(_request.argv[0x00], _request.argv[0x01], SENS_READ_HUMD, &value);
      }
      goto finish;

    case CMD_DHT_TEMPERATURE: {
        //
        //  DHT.temperature[pin, model]
        //
        if (!isSafePin(_request.argv[0x00])) {
          rc = DEVICE_ERROR_CONNECT;
        } else {
          rc = getDHTOneMetric(_request.argv[0x00], _request.argv[0x01], SENS_READ_TEMP, &value);
        }
        goto finish;
      }

    case CMD_DHT_ALL: {
        //
        //  DHT.temperature[pin, model]
        //
        if (!isSafePin(_request.argv[0x00])) {
          rc = DEVICE_ERROR_CONNECT;
        } else {
          rc = getDHTAllMetric(_request.argv[0x00], _request.argv[0x01], _request.payloadChar, _request.dataFreeSize);
        }
        goto finish;
      }
#endif // FEATURE_DHT_ENABLE

#ifdef FEATURE_MAX7219_ENABLE
    case CMD_MAX7219_WRITE: {
        //
        //  MAX7219.write[dataPin, clockPin, loadPin, intensity, data]
        //
        if (((SCK == _request.argv[0x01] && MOSI == _request.argv[0x00]) || (isSafePin(_request.argv[0x00]) && isSafePin(_request.argv[0x01]))) && isSafePin(_request.argv[0x02])) {
          writeToMAX7219(_request.argv[0x00], _request.argv[0x01], _request.argv[0x02], _request.argv[0x03], (uint8_t*) _request.args[0x04]);
          rc = RESULT_IS_OK;
        }
        goto finish;
      }
#endif // FEATURE_MAX7219_ENABLE

#ifdef FEATURE_MAX6675_ENABLE
    case CMD_MAX6675_TEMPERATURE: {
        //
        //  max6675.temperature[dataPin, clockPin, csPin]
        // max6675.temperature[4,6,5]
        //
        if (isSafePin(_request.argv[0x00]) && isSafePin(_request.argv[0x01]) && isSafePin(_request.argv[0x02])) {
          rc = getMAX6675Metric(_request.argv[0x00], _request.argv[0x01], _request.argv[0x02], SENS_READ_TEMP, &value);
        }
        goto finish;
      }
#endif // FEATURE_MAX6675_ENABLE

#ifdef FEATURE_ACS7XX_ENABLE
#endif // FEATURE_ACS7XX_ENABLE

#ifdef FEATURE_IR_ENABLE
    case CMD_IR_SEND: {
        //
        //  ir.send[pwmPin, irPacketType, nBits, data, repeat, address]
        //
        // ATmega328: Use D3 only at this time
        // Refer to other Arduino's pinouts to find OC2B pin
        //      if (isSafePin(argv[0x00]) && TIMER2B == digitalPinToTimer(argv[0x00])) {
        // irPWMPin - global wariable that replace IRremote's TIMER_PWM_PIN
        //         irPWMPin = argv[0x00];
#if defined(ARDUINO_ARCH_AVR)
        rc = sendCommandByIR(_request.argv[0x01], _request.argv[0x02], _request.argv[0x03], _request.argv[0x04], _request.argv[0x05]);
#elif defined(ARDUINO_ARCH_ESP8266)
        rc = ZBX_NOTSUPPORTED;
#endif //#if defined(ARDUINO_ARCH_AVR)
        //      }
        goto finish;
      }

    case CMD_IR_SENDRAW: {
        //
        //  ir.sendRaw[pwmPin, irFrequency, nBits, data]
        //  !!! need to increase ARGS_PART_SIZE, because every data`s Integer number take _four_ HEX-chars => 70 RAW array items take 282 (2+70*4) byte of incoming buffer only
        //
        // ATmega328: Use D3 only at this time
        // Refer to other Arduino's pinouts to find OC2B pin
        //     if (isSafePin(argv[0x00]) && TIMER2B == digitalPinToTimer(argv[0x00])) {
        // irPWMPin - global wariable that replace IRremote's TIMER_PWM_PIN
        //         irPWMPin = argv[0x00];
#if defined(ARDUINO_ARCH_AVR)
        rc = sendRawByIR(_request.argv[0x01], _request.argv[0x02], _request.args[0x03]);
#elif defined(ARDUINO_ARCH_ESP8266)
        rc = ZBX_NOTSUPPORTED;
#endif //#if defined(ARDUINO_ARCH_AVR)
        //     }
        goto finish;
      }
#endif // FEATURE_IR_ENABLE

#ifdef FEATURE_SERVO_ENABLE
    case CMD_SERVO_TURN: {
        //
        //  Servo.turn[servoPin, targetAnglePulseWidth, turnTime, holdTime, returnAnglePulseWidth]
        //
        //  Need to add updateFrequency as argv[0x05] ?
        //
        //  servo.turn[5, 1500, 500, 2000, 680]
        //
        if (!isSafePin(_request.argv[0x00])) {
          goto finish;
        }

        uint16_t targetAnglePulseWidth, returnAnglePulseWidth;
        uint32_t holdTime, turnTime;
        turnTime = (_request.args[0x02] && _request.argv[0x02] > 0x00) ? _request.argv[0x02] : 0x00;
        holdTime = (_request.args[0x03] && _request.argv[0x03] > 0x00) ? _request.argv[0x03] : 0x00;
        targetAnglePulseWidth = (_request.args[0x01] && _request.argv[0x01] > 0x00) ? _request.argv[0x01] : 0x00;
        returnAnglePulseWidth = (_request.args[0x04] && _request.argv[0x04] > 0x00) ? _request.argv[0x04] : 0x00;
        /*
          this code a little fat when compiled
          turnTime = (_request.argv[0x02] > 0x00) ? _request.argv[0x02] : 0x00;
          holdTime = (_request.argv[0x03] > 0x00) ? _request.argv[0x03] : 0x00;
          targetAnglePulseWidth = (_request.argv[0x01] > 0x00) ? _request.argv[0x01] : 0x00;
          returnAnglePulseWidth = (_request.argv[0x04] > 0x00) ? _request.argv[0x04] : 0x00;
        */
        rc = servoTurn(_request.argv[0x00], targetAnglePulseWidth, turnTime, holdTime, returnAnglePulseWidth);
        goto finish;
      }
#endif // FEATURE_SERVO_ENABLE

#ifdef FEATURE_RELAY_ENABLE
    case CMD_PULSE: {
        //
        //  pulse[targetPin, targetState, holdTime, returnState]
        //
        // targetPin is safe?
        if (!isSafePin(_request.argv[0x00])) {
          goto finish;
        }
        // make targetState boolean !!
        _request.argv[0x01] = !!_request.argv[0x01]; // ? 1 : 0;

        // if no returnState is specified: returnState = !targetState
        if (!_request.args[0x03]) {
          _request.argv[0x03] = ! _request.argv[0x01];
        }

        // holdTime is specified?
        if (!_request.args[0x02]) {
          _request.argv[0x02] = 0x00;
        }
        rc = pulse(_request.argv[0x00], _request.argv[0x01], _request.argv[0x02], _request.argv[0x03]);
        goto finish;
      }

    case CMD_RELAY: {
        //
        //  relay[relayPin, relayState, testPin, testState, testPinMode]
        //  relay[4,1,2,1]
        //  relay[4,1,2,0,1]

        // relayPin is safe?
        if (!isSafePin(_request.argv[0x00])) {
          goto finish;
        }

        // testPin is specified?
        if (!_request.args[0x02]) {
          _request.argv[0x02] = -0x01;
          // testPin is safe?
        } else if (!isSafePin(_request.argv[0x02])) {
          goto finish;
        }
        // testState is specified?
        if (!_request.args[0x03]) {
          _request.argv[0x03] = -0x01;
        }
        // pullup is need?
        rc = relay(_request.argv[0x00], _request.argv[0x01], _request.argv[0x02], _request.argv[0x03], (!!_request.argv[0x04]) ? INPUT_PULLUP : INPUT);
        goto finish;
      }
#endif // FEATURE_RELAY_ENABLE

  } // switch (cmdIdx) in "commands with options" block

  // ************************************************************************************************************************************
  // Following commands use <argv[0x00]> or <argv[0x01]> pins for sensor handling (UART, I2C, etc) and these pins can be disabled in port_protect[] array
  //  Otherwise - processing is failed
  rc = RESULT_IS_FAIL;
  if (!_request.args[0x00] || !_request.args[0x01] || !isSafePin(_request.argv[0x00]) || !isSafePin(_request.argv[0x01])) {
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
      // argv[0x03] (intNumber) currently not used
#if defined(ARDUINO_ARCH_AVR)
      rc = manageIncEnc(&value, _request.argv[0x00], _request.argv[0x01], _request.argv[0x02]);
#elif defined(ARDUINO_ARCH_ESP8266)
      rc = ZBX_NOTSUPPORTED;
#endif //#if defined(ARDUINO_ARCH_AVR)
      goto finish;
#endif // FEATURE_INCREMENTAL_ENCODER_ENABLE

#ifdef FEATURE_ULTRASONIC_ENABLE
    case CMD_ULTRASONIC_DISTANCE:
      //
      //  ultrasonic.distance[triggerPin, echoPin]
      //
      rc = getUltrasonicMetric(_request.argv[0x00], _request.argv[0x01], 0x01, &value);
      goto finish;
#endif // FEATURE_ULTRASONIC_ENABLE

#ifdef FEATURE_PZEM004_ENABLE
    //
    //  0xC0A80101 - an default IP address for PZEM (192.168.1.1)
    //
    case CMD_PZEM004_CURRENT: {
        //
        //  pzem004.current[rxPin, txPin, addr]
        //
        // payload cast to (uint8_t*) to use with subroutine math and SoftwareSerial subs, because used instead sub's internal buffer and save a little RAM size.
        // Its will be casted to char* inside at moment when its need
        rc = getPZEM004Metric(_request.argv[0x00], _request.argv[0x01], _request.args[0x02], SENS_READ_AC, &value);
        goto finish;
      }

    case CMD_PZEM004_VOLTAGE: {
        //
        //  pzem004.voltage[rxPin, txPin, addr]
        //
        rc = getPZEM004Metric(_request.argv[0x00], _request.argv[0x01], _request.args[0x02], SENS_READ_VOLTAGE, &value);
        goto finish;
      }

    case CMD_PZEM004_POWER: {
        //
        //  pzem004.power[rxPin, txPin, addr]
        //
        rc = getPZEM004Metric(_request.argv[0x00], _request.argv[0x01], _request.args[0x02], SENS_READ_POWER, &value);
        goto finish;
      }

    case CMD_PZEM004_ENERGY: {
        //
        //  pzem004.energy[rxPin, txPin, addr]
        //
        rc = getPZEM004Metric(_request.argv[0x00], _request.argv[0x01], _request.args[0x02], SENS_READ_ENERGY, &value);
        goto finish;
      }

    case CMD_PZEM004_SETADDR: {
        //
        //  pzem004.setAddr[rxPin, txPin, addr]
        //
        rc = getPZEM004Metric(_request.argv[0x00], _request.argv[0x01], _request.args[0x02], SENS_CHANGE_ADDRESS, &value);
        goto finish;
      }
#endif // FEATURE_PZEM004_ENABLE

#ifdef FEATURE_UPS_APCSMART_ENABLE
    case CMD_UPS_APCSMART: {
        //
        //  ups.apcsmart[rxPin, txPin, command]
        //    command - HEX or ASCII
        //
        rc = getAPCSmartUPSMetric(_request.argv[0x00], _request.argv[0x01], _request.args[0x02], _request.payloadChar);
        goto finish;
      }
#endif // FEATURE_UPS_APCSMART_ENABLE

#ifdef FEATURE_UPS_MEGATEC_ENABLE
    case CMD_UPS_MEGATEC: {
        //
        //  ups.megatec[rxPin, txPin, command, fieldNumber]
        //    command - HEX or ASCII
        //
        rc = getMegatecUPSMetric(_request.argv[0x00], _request.argv[0x01], _request.args[0x02], _request.argv[0x03], _request.payloadChar);
        goto finish;
      }
#endif // FEATURE_UPS_APCSMART_ENABLE

#ifdef FEATURE_DFPLAYER_ENABLE
    case CMD_DFPLAYER_RUN: {
        //
        //  dfplayer.run[rxPin, txPin, command, option, volume]
        //  dfplayer.run[4, 5, 0x03, 0x02, 30]
        //
        // use -1 if volume must stay. Otherwise - volume will be changed.
        rc = runDFPlayerMini(_request.argv[0x00], _request.argv[0x01], _request.argv[0x02], _request.argv[0x03], (_request.args[0x04] ? -0x01 : _request.argv[0x04]), _request.payloadByte);
        goto finish;
      }
#endif // FEATURE_DFPLAYER_ENABLE

#ifdef FEATURE_PLANTOWER_PMSXX_ENABLE
    case CMD_PLANTOWER_PMSXX_ALL: {
        //
        //  PMS.All[rxPin, txPin]
        //
        rc = getPlantowerPMSAllMetrics(_request.argv[0x00], _request.argv[0x01], _request.payloadChar, _request.dataFreeSize);
        goto finish;
      }

    case CMD_PLANTOWER_PMSXX_FPM: {
        //
        //  PMS.fpm[rxPin, txPin, particleSize]
        //  PM25 read
        //  PMS.fpm[2, 3, 25]
        rc = getPlantowerPMSOneMetric(_request.argv[0x00], _request.argv[0x01], SENS_READ_CONCENTRATION, PLANTOWER_CONCENTRATION_TYPE_FACTORY, _request.argv[0x02], (uint32_t*) &value);
        goto finish;
      }

    case CMD_PLANTOWER_PMSXX_EPM: {
        //
        //  PMS.epm[rxPin, txPin, particleSize]
        //  PM25 read
        //  PMS.epm[2, 3, 25]
        rc = getPlantowerPMSOneMetric(_request.argv[0x00], _request.argv[0x01], SENS_READ_CONCENTRATION, PLANTOWER_CONCENTRATION_TYPE_ENVIRONMENT, _request.argv[0x02], (uint32_t*) &value);
        goto finish;
      }

#endif // FEATURE_PLANTOWER_PMSXX_ALL_ENABLE

#ifdef FEATURE_WUHAN_CUBIC_PM_UART_ENABLE
    case CMD_WCPM_UART_ALL: {
        //
        //  WCPM.UART.All[rxPin, txPin]
        //
        rc = getPlantowerWuhanPMAllMetrics(_request.argv[0x00], _request.argv[0x01], _request.payloadChar, _request.dataFreeSize);
        goto finish;
      }
#endif

#ifdef FEATURE_NOVA_FITNESS_SDS_ENABLE
    case CMD_NOVA_SDS_ALL: {
        //
        //  SDS.All[rxPin, txPin]
        //  SDS.All[4,5]
        rc = getNovaSDSAllMetrics(_request.argv[0x00], _request.argv[0x01], _request.payloadChar, _request.dataFreeSize);
        goto finish;
      }

    case CMD_NOVA_SDS_EPM: {
        //
        //  SDS.epm[rxPin, txPin, particleSize]
        //  PM25 read
        //  SDS.epm[4, 5, 25]
        rc = getNovaSDSOneMetric(_request.argv[0x00], _request.argv[0x01], SENS_READ_CONCENTRATION, _request.argv[0x02], (uint32_t*) &value);
        goto finish;
      }

#endif // FEATURE_NOVA_FITNESS_SDS_ENABLE

#ifdef FEATURE_MHZXX_UART_ENABLE
    case CMD_MHZXX_UART_CO2:
      //
      //  MHZxx.UART.CO2[rxPin, txPin]
      //
      rc = getQModeMhZxMetric(_request.argv[0x00], _request.argv[0x01], SENS_READ_CO2, &value);
      goto finish;
#endif // FEATURE_MHZXX_UART_ENABLE

#ifdef FEATURE_WINSEN_ZE08_CH2O_ENABLE
    case CMD_ZE08_CH2O: {
        //
        //  ze08.ch2o[rxPin, txPin]
        //  ze08.ch2o[16,17]
        rc = getAModeZe08Ch2OMetric(_request.argv[0x00], _request.argv[0x01], SENS_READ_CH2O, &value);
        goto finish;
      }
#endif // FEATURE_WINSEN_ZE08_CH2O_ENABLE

#ifdef FEATURE_WINSEN_ZE14_O3_ENABLE
    case CMD_ZE14_O3: {
        //
        //  ze14.o3[rxPin, txPin]
        rc = getAModeZe14O3Metric(_request.argv[0x00], _request.argv[0x01], SENS_READ_O3, &value);
        goto finish;
      }
#endif // FEATURE_WINSEN_ZE14_O3_ENABLE

#ifdef FEATURE_WINSEN_ZP14_ENABLE
    case CMD_ZP14_NG: {
        //
        //  zp14.ng[rxPin, txPin]
        rc = getAModeZp14Metric(_request.argv[0x00], _request.argv[0x01], SENS_READ_CH4, &value);
        goto finish;
      }
#endif // FEATURE_WINSEN_ZP14_ENABLE

#ifdef FEATURE_MODBUS_RTU_ENABLE
    case CMD_MB_RTU_FC03: {
        //
        //  mb.RTU.FC03[rxPin, txPin, enablePin, uartSpeed, slaveAddr, regAddr]
        //
        // mb.RTU.FC03[4, 5, 0, 9600, 1, 9]
        //uint8_t buffer[registersNum * 0x02 + MODBUS_PACKET_QUERY_FRAME_LENGTH];
        //uint16_t* ptrRegisters = (uint16_t*) &buffer[MODBUS_PACKET_ANSWER_HEADER_LENGTH];


        int8_t txEnablePin = -0x01;

        if (_request.args[0x02]) {
          if (!isSafePin(_request.argv[0x02])) {
            goto finish;
          }
          txEnablePin = _request.argv[0x02];
        }

        uint8_t registersNum = 0x01;
        uint8_t *buffer = _request.payloadByte;
        uint16_t* ptrRegisters = (uint16_t*) &buffer[MODBUS_PACKET_ANSWER_HEADER_LENGTH];

        rc = modbusRtuUart(_request.argv[0x00], _request.argv[0x01], txEnablePin, _request.argv[0x03], MODBUS_FUNCTION_CODE_03, _request.argv[0x04], _request.argv[0x05], buffer, registersNum);
        if (RESULT_IS_OK == rc) {
          value = ptrRegisters[0x00];
          rc = RESULT_IS_UNSIGNED_VALUE;
        }
        goto finish;
      }
#endif // FEATURE_MODBUS_RTU_ENABLE


  } // switch (cmdIdx) Non-I2C related commands block

  // ************************************************************************************************************************************
  //  I2C-related commands have additional processing
#ifdef TWI_USE
  // Otherwise - TWI interface can be reconfigured with new pins
  SoftTWI.begin(_request.argv[0x00], _request.argv[0x01]);
  uint8_t i2CAddress;

  i2CAddress = _request.args[0x02] ? (uint8_t) _request.argv[0x02] : I2C_NO_ADDR_SPECIFIED;

#ifdef FEATURE_I2C_ENABLE
  // this war is used only in FEATURE_I2C_ENABLE blocks
  int16_t i2CRegister;
  i2CRegister = (_request.args[0x03]) ? (uint16_t) _request.argv[0x03] : I2C_NO_REG_SPECIFIED;
#endif // FEATURE_I2C_ENABLE

  switch (cmdIdx) {
#ifdef FEATURE_I2C_ENABLE
    case CMD_I2C_SCAN:
      //
      //  I2C.scan[sdaPin, sclPin]
      // I2C.scan[18,19]
      //
      value = scanI2C(SoftTWI, _request.payloadByte);
      if (value) {
        rc = RESULT_IS_BUFFERED;
      }
      goto finish;

    case CMD_I2C_WRITE: {
        //
        // i2c.write[sdaPin, sclPin, i2cAddress, register, data, numBytes]
        //
        if (I2C_NO_ADDR_SPECIFIED == i2CAddress) {
          goto finish;
        }
        rc = writeValueToI2C(&SoftTWI, i2CAddress, i2CRegister, (uint32_t) _request.argv[0x04], (uint8_t) _request.argv[0x05]);
        goto finish;
      }

    case CMD_I2C_READ: {
        //
        // i2c.read[sdaPin, sclPin, i2cAddress, register, numBytes, numberOfReadings]
        //
        if (I2C_NO_ADDR_SPECIFIED == i2CAddress) {
          goto finish;
        }
        rc = readValueFromI2C(&SoftTWI, i2CAddress, i2CRegister, (uint32_t*) &value, _request.argv[0x04], (_request.args[0x05] ? _request.argv[0x05] : 0x00));
        goto finish;
      }

    case CMD_I2C_BITWRITE: {
        //
        //  i2c.bitWrite[sdaPin, sclPin, i2cAddress, register, bitNo, value]
        //
        if (I2C_NO_ADDR_SPECIFIED == i2CAddress) {
          goto finish;
        }

        rc = bitWriteToI2C(&SoftTWI, i2CAddress, i2CRegister, _request.argv[0x04], _request.argv[0x05]);
        goto finish;
      }

    case CMD_I2C_BITREAD: {
        //
        //  i2c.bitRead[sdaPin, sclPin, i2cAddress, register, bitNo]
        //
        if (I2C_NO_ADDR_SPECIFIED == i2CAddress) {
          goto finish;
        }
        rc = bitReadFromI2C(&SoftTWI, i2CAddress, i2CRegister, _request.argv[0x04], (uint8_t*) &value);
        goto finish;
      }

#endif // FEATURE_I2C_ENABLE

#ifdef FEATURE_BH1750_ENABLE
    case CMD_BH1750_LIGHT: {
        //
        //  BH1750.light[sdaPin, sclPin, i2cAddress]
        //
        rc = getBH1750Metric(&SoftTWI, i2CAddress, SENS_READ_LUX, &value);
        goto finish;
      }
#endif // FEATURE_BH1750_ENABLE

#ifdef FEATURE_VEML6070_ENABLE
    case CMD_VEML6070_UVA: {
        //
        //  VEML6070.uva[sdaPin, sclPin, integrationTime, rSet]
        //  VEML6070.uva[18, 19, 3, 360]
        rc = getVEML6070Metric(&SoftTWI, _request.argv[0x02], _request.argv[0x03], SENS_READ_UVA, &value);
        goto finish;
      }

    case CMD_VEML6070_UVI: {
        //
        //  VEML6070.uvi[sdaPin, sclPin, integrationTime, rSet]
        //  VEML6070.uvi[18, 19, 3, 360]
        rc = getVEML6070Metric(&SoftTWI, _request.argv[0x02], _request.argv[0x03], SENS_READ_UVI, &value);
        goto finish;
      }
#endif // FEATURE_VEML6070_ENABLE

#ifdef FEATURE_SHT2X_ENABLE
    case CMD_SHT2X_HUMIDITY:
      //
      //  SHT2X.Humidity[sdaPin, sclPin, i2cAddress]
      //
      rc = getSHT2XMetric(&SoftTWI, i2CAddress, SENS_READ_HUMD, &value);
      goto finish;

    case CMD_SHT2X_TEMPERATURE:
      //
      //  SHT2X.Temperature[sdaPin, sclPin, i2cAddress]
      //
      // (uint8_t) argv[0x02] is i2c address, 7 bytes size
      rc = getSHT2XMetric(&SoftTWI, i2CAddress, SENS_READ_TEMP, &value);
      goto finish;
#endif // FEATURE_SHT2X_ENABLE


#ifdef FEATURE_SGP30_ENABLE
    case CMD_SGP30_CO2E: {
        //
        //  sgp30.co2e[sdaPin, sclPin, i2cAddress]
        // sgp30.co2e[18, 19, 0x58]
        rc = getSGP30Metric(&SoftTWI, i2CAddress, 0x00, SENS_READ_CO2E, false, &value);
        goto finish;
      }

    case CMD_SGP30_TVOC: {
        //
        //  sgp30.tvoc[sdaPin, sclPin, i2cAddress]
        rc = getSGP30Metric(&SoftTWI, i2CAddress, 0x00, SENS_READ_TVOC, false, &value);
        goto finish;
      }
#endif

#ifdef FEATURE_MAX44009_ENABLE
    case CMD_MAX44009_LIGHT: {
        //
        //  MAX44009.light[sdaPin, sclPin, i2cAddress, mode, integrationTime]
        //

        rc = getMAX44009Metric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], SENS_READ_LUX, &value);
        goto finish;
      }
#endif // FEATURE_MAX44009_ENABLE

#ifdef FEATURE_MLX90614_ENABLE
    case CMD_MLX90614_TEMPERATURE: {
        //
        //  mlx90614.temperature[sdaPin, sclPin, i2cAddress, zone]
        //  mlx90614.temperature[18, 19, 0x5A, 1]
        rc = getMLX90614Metric(&SoftTWI, i2CAddress, _request.argv[0x03], SENS_READ_TEMP, &value);
        goto finish;
      }
#endif

#ifdef FEATURE_PCF8574_LCD_ENABLE
    case CMD_PCF8574_LCDPRINT: {
        //
        //  pcf8574.LCDPrint[sdaPin, sclPin, i2cAddress, lcdBacklight, lcdType, data]
        //
        rc = printToPCF8574LCD(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], _request.args[0x05]);
        // Store current time when on LCD was printed something to calculation in loopStageUserFunction() (for example) 'no refresh timeout' properly
        sysMetrics.sysLCDLastUsedTime = millis();
        goto finish;
      }
#endif // FEATURE_PCF8574_LCD_ENABLE

#ifdef FEATURE_BMP_ENABLE
    case CMD_BMP_PRESSURE: {
        //
        //  BMP.Pressure[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
        //BMP.Pressure[4, 5, 0x76]
        rc = getBMPMetric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], SENS_READ_PRSS, &value);
        goto finish;
      }

    case CMD_BMP_TEMPERATURE: {
        //
        // BMP.Temperature[sdaPin, sclPin, i2cAddress, overSampling]
        //
        rc = getBMPMetric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], SENS_READ_TEMP, &value);
        goto finish;
      }

#ifdef SUPPORT_BME280_INCLUDE
    case CMD_BME_HUMIDITY: {
        //
        //  BME.Humidity[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
        //
        rc = getBMPMetric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], SENS_READ_HUMD, &value);
        goto finish;
      }
#endif // SUPPORT_BME280_INCLUDE
#endif // FEATURE_BMP_ENABLE

#ifdef FEATURE_TSL2561_ENABLE
    case CMD_TSL2561_LIGHT: {
        //
        //  TSL2561.light[sdaPin, sclPin, i2cAddress, integrationTime, gain]
        //  TSL2561.light[18, 19, 0x39, 402, 1]
        rc = getTSL2561Metric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], SENS_READ_LUX, &value);
        goto finish;
      }
#endif // FEATURE_TSL2561_ENABLE

#ifdef FEATURE_ADPS9960_ENABLE
    // !!! IR Led not used for ALS conversion
    // RGBC results can be used as light levels in Lux
    case CMD_ADPS9960_AMBIENT: {
        //
        //  ADPS9960.ambient[sdaPin, sclPin, i2cAddress, integrationTime, gain]
        //  ADPS9960.ambient[18, 19, 0x39, 103, 4]
        rc = getADPS9960Metric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], APDS9960_DEFAULT_LED_DRIVE, SENS_READ_LIGHT_AMBIENT, &value);
        goto finish;
      }

    case CMD_ADPS9960_RED: {
        //
        //  ADPS9960.red[sdaPin, sclPin, i2cAddress, integrationTime, gain]
        //  ADPS9960.red[18, 19, 0x39, 103, 4]
        rc = getADPS9960Metric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], APDS9960_DEFAULT_LED_DRIVE, SENS_READ_LIGHT_RED, &value);
        goto finish;
      }

    case CMD_ADPS9960_GREEN: {
        //
        //  ADPS9960.green[sdaPin, sclPin, i2cAddress, integrationTime, gain]
        //  ADPS9960.green[18, 19, 0x39, 103, 4]
        rc = getADPS9960Metric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], APDS9960_DEFAULT_LED_DRIVE, SENS_READ_LIGHT_GREEN, &value);
        goto finish;
      }

    case CMD_ADPS9960_BLUE: {
        //
        //  ADPS9960.blue[sdaPin, sclPin, i2cAddress, integrationTime, gain]
        //  ADPS9960.blue[18, 19, 0x39, 103, 4]
        rc = getADPS9960Metric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], APDS9960_DEFAULT_LED_DRIVE, SENS_READ_LIGHT_BLUE, &value);
        goto finish;
      }
#endif // FEATURE_ADPS9960_ENABLE

#ifdef FEATURE_PCA9685_ENABLE
    case CMD_PCA9685_WRITE: {
        //
        //  PCA9685.write[sdaPin, sclPin, i2cAddress, outputIdx, onTime, offTime]
        //  PCA9685.write[18, 19, 0x40,, 4096, 0]
        rc = writePCA9685(&SoftTWI, i2CAddress, (_request.args[0x03] ? _request.argv[0x03] : PCA9685_CHANNEL_LEDS_ALL), _request.argv[0x04], _request.argv[0x05], PCA9685_MODE_TOTEM_POLE);
        goto finish;
      }
#endif // FEATURE_PCA9685_ENABLE

#ifdef FEATURE_INA219_ENABLE
    case CMD_INA219_BUSVOLTAGE:
      //
      //  INA219.BusVoltage[sdaPin, sclPin, i2cAddress, voltageRange, maxCurrent]
      //
      //INA219.BusVoltage[18,19,0x40,16,1000]
      rc = getINA219Metric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], SENS_READ_BUS_VOLTAGE, (uint32_t&) value);
      goto finish;

    case CMD_INA219_CURRENT:
      //
      //  INA219.Current[sdaPin, sclPin, i2cAddress, voltageRange, maxCurrent]
      //
      rc = getINA219Metric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], SENS_READ_DC, (uint32_t&) value);
      goto finish;

    case CMD_INA219_POWER:
      //
      //  INA219.Power[sdaPin, sclPin, i2cAddress, voltageRange, maxCurrent]
      //
      rc = getINA219Metric(&SoftTWI, i2CAddress, _request.argv[0x03], _request.argv[0x04], SENS_READ_POWER, (uint32_t&) value);
      goto finish;

#endif // FEATURE_INA219_ENABLE

#ifdef FEATURE_T67XX_ENABLE
    case CMD_T67XX_I2C_CO2: {
        //
        //  t67xx.i2c.co2[sdaPin, sclPin, i2cAddress]
        //  t67xx.i2c.co2[18,19]
        rc = getT67XXMetric(&SoftTWI, i2CAddress, SENS_READ_CO2, &value);
        goto finish;
      }
#endif // FEATURE_T67XX_ENABLE

#ifdef FEATURE_WUHAN_CUBIC_PM_I2C_ENABLE
    case CMD_WCPM_I2C_ALL: {
        //
        //  WCPM.I2C.All[sdaPin, sclPin, i2cAddress]
        //  WCPM.I2C.All[18, 19]
        //  WCPM.UART.All[18, 19]
        rc = getPlantowerWuhanPMAllMetrics(&SoftTWI, i2CAddress, _request.payloadChar, _request.dataFreeSize);
        goto finish;
      }
#endif


  }
#endif // TWI_USE


finish:
  // Form the output buffer routine

  /*
                Process pre-action before sending reply to server
  */

#ifdef FEATURE_EEPROM_ENABLE
  if (RESULT_IS_UNSTORED_IN_EEPROM == rc) {
    rc = (saveConfigToEEPROM(_sysConfig)) ? RESULT_IS_OK : DEVICE_ERROR_EEPROM_CORRUPTED;
  }
#endif

  makeTextPayload(_request.payloadChar, value, rc);
  /*
                        Push Zabbix packet header if incoming packet have Zabbix type
  */
  payloadLength = strlen(_request.payloadChar);
  if (PACKET_TYPE_ZABBIX == _request.type) {
    // Calculate payload length to
    if (RESULT_IS_BUFFERED == rc) {
      switch (cmdIdx) {
#ifdef FEATURE_I2C_ENABLE
        case CMD_I2C_SCAN:
          // 'value' contained addreses number that scanI2C() return, 0x05 -> "0x" + two chars of string representation of HEX number + '\n'
          payloadLength = value * 0x05;
          break;
#endif
#ifdef FEATURE_OW_ENABLE
        case CMD_OW_SCAN:
          // 'value' contained addresses number that scanOneWire() return, 0x13 (19) -> "0x" + eight*two chars of string representation of HEX number + '\n'
          payloadLength = value * (2 + ONEWIRE_ID_SIZE * 2 + 1);
          break;
#endif
        default:
          break;
      }
    }
    // Fill reply packet header
    // Copy prefix to header (uppercase only!)
    memcpy(_request.data, "ZBXD\1", ZBX_HEADER_PREFIX_LENGTH);
    // Copy Length of answer string to header
    memcpy(&_request.data[ZBX_HEADER_PREFIX_LENGTH], &payloadLength, sizeof(payloadLength));
    __DMLD(
      DEBUG_PORT.println(F("--hdr--"));
    for (i = 0; i < ZBX_HEADER_LENGTH; i++) {
    DEBUG_PORT.print(_request.data[i], HEX);
      DEBUG_PORT.print(" '");
      DEBUG_PORT.print((char) _request.data[i]);
      DEBUG_PORT.println("' ");
    }
    DEBUG_PORT.println(F("--pld--"));
    )
    _netClient.write(_request.data, ZBX_HEADER_LENGTH);
  }

  /*
                        Push reply data to the client
  */
  __DMLL( DEBUG_PORT.print(F("Reply: ")); )
  switch (cmdIdx) {
#ifdef FEATURE_OW_ENABLE
    // Special output format for OW.Scan[] command: 0x01020304\n0x05060708\n...
    case CMD_OW_SCAN:
      printArray(_request.payloadByte, ONEWIRE_ID_SIZE * value, _netClient, OW_ADDRESS);
      __DMLL( printArray(_request.payloadByte, ONEWIRE_ID_SIZE * value, Serial, OW_ADDRESS); )
      break;
#endif
#ifdef FEATURE_I2C_ENABLE
    // Special output format for I2C.Scan[] command: 0x01\n0x02\n0x03...
    case CMD_I2C_SCAN: {
        printArray(_request.payloadByte, I2C_ADDRESS_LENGTH * value, _netClient, I2C_ADDRESS);
        __DMLL( printArray(_request.payloadByte, I2C_ADDRESS_LENGTH * value, Serial, I2C_ADDRESS); )
      }
      break;
#endif
    // Plain text output for other
    default:
      //__DMLD( DEBUG_PORT.print("payloadLength:"); DEBUG_PORT.println(payloadLength); )
      _netClient.write(_request.payloadByte, payloadLength);
      if (PACKET_TYPE_ZABBIX != _request.type) {
        _netClient.print('\n');
      }
      __DMLL( DEBUG_PORT.println(_request.payloadChar); )
      break;
  }
  //}


  /*
                Process post-action after sending reply to server
  */
  //
  switch (rc) {
    case RESULT_IS_SYSTEM_REBOOT_ACTION:
      //_netClient.stop();
      delay(10);
      systemReboot();
      break;
  }

  __DMLL( DEBUG_PORT.println(); )
  return cmdIdx;
}
