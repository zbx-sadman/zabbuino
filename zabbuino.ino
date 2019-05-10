/*

  Sketch uses 19,150 bytes (7%) of program storage space. Maximum is 253,952 bytes.
  Global variables use 1,160 bytes (14%) of dynamic memory, leaving 7,032 bytes for local variables. Maximum is 8,192 bytes.

  Sketch uses 21,420 bytes (8%) of program storage space. Maximum is 253,952 bytes.
  Global variables use 1,136 bytes (13%) of dynamic memory, leaving 7,056 bytes for local variables. Maximum is 8,192 bytes.
*/
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
                                                           STARTUP SECTION
*/

void setup() {
#ifdef SERIAL_USE
  Serial.begin(constSerialMonitorSpeed);
#endif // SERIAL_USE
  SPI.begin();

  DTSL( Serial.print(FSH_P(constZbxAgentVersion)); Serial.println(FSH_P(STRING_wakes_up)); )

  sysMetrics.sysVCCMin = sysMetrics.sysVCCMax = getADCVoltage(ANALOG_CHAN_VBG);
  sysMetrics.sysRamFree = sysMetrics.sysRamFreeMin = getRamFree();

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
  uint8_t result, needNetworkRelaunch = true, errorCode = ERROR_NONE;
  char incomingData;
  //uint16_t blinkType = constBlinkNope;
  uint32_t processStartTime, processEndTime, prevPHYCheckTime, prevNetActivityTime, prevSysMetricGatherTime, clientConnectTime, netDebugPrintTime;
  // this value helps make Network init immedately
  //uint32_t phyCheckInterval = 0x00;
  char cBuffer[constBufferSize + 1]; // +1 for trailing \0
  // Last idx is not the same that array size
  char* optarg[constArgC];
  packetInfo_t packetInfo = {PACKET_TYPE_NONE, 0x00, 0x00, optarg};
  NetworkClient netClient;
  NetworkServer netServer(constZbxAgentTcpPort);

  __USER_FUNCTION( uint32_t prevUserFuncCall = 0x00; )

  // 0. Init some libs to make system screen works if it enabled
#ifdef TWI_USE
  SoftTWI.begin();
#endif

#ifdef FEATURE_SYSTEM_RTC_ENABLE
  DTSM( Serial.print(FSH_P(STRING_Init_system_RTC)); Serial.print(FSH_P(STRING_3xDot_Space)); )
  if (! initRTC(&SoftTWI)) {
    // sysMetrics.sysStartTimestamp already inited by 0x00 with memset() on start
    DTSM( Serial.println(FSH_P(STRING_ok)); )
  } else {
    // Get current timestamp from system RTC and store it into sysMetrics structure.
    // On RTC failure - do not use it
    if (!getUnixTime(&SoftTWI, (uint32_t*) &sysMetrics.sysStartTimestamp)) {
      sysMetrics.sysStartTimestamp = 0x00;
    }
  }
  DTSM( Serial.println(FSH_P(STRING_fail)); )
#endif

  // Run user function
  __USER_FUNCTION( initStageUserFunction(cBuffer); )

  // System load procedure
  // 1. Factory reset block
#ifdef FEATURE_EEPROM_ENABLE
  // factoryReset() return false on EEPROM saving fail
  if (!factoryReset(sysConfig)) {
    DTSM( Serial.print(FSH_P(STRING_Config)); Serial.println(FSH_P(STRING_saving_error)); )
  }
#endif // FEATURE_EEPROM_ENABLE

  // 2. Load configuration from EEPROM
  //
#ifdef FEATURE_EEPROM_ENABLE
  DTSL( Serial.print(FSH_P(STRING_Config)); Serial.print(FSH_P(STRING_loading)); Serial.print(FSH_P(STRING_3xDot_Space)); )
  if (!loadConfigFromEEPROM(sysConfig)) {
    DTSL( Serial.println(FSH_P(STRING_fail)); )
    // bad CRC detected, use default values for this run
    setConfigDefaults(sysConfig);
    if (!saveConfigToEEPROM(sysConfig)) {
      DTSM( Serial.print(FSH_P(STRING_Config)); Serial.println(FSH_P(STRING_saving_error)); )
      DTSL( Serial.println(FSH_P(STRING_Use_default_settings)); )
      // what to do on saving error?
    }
  } else {
    DTSL( Serial.println(FSH_P(STRING_ok)); )
  }
#else // FEATURE_EEPROM_ENABLE
  DTSM( Serial.println(FSH_P(STRING_EEPROM_disabled)); )
  DTSL( Serial.println(FSH_P(STRING_Use_default_settings)); )
  // Use hardcoded values if EEPROM feature disabled
  setConfigDefaults(sysConfig);
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
  set_zone(sysConfig.tzOffset);
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
  __USER_FUNCTION( netPrepareStageUserFunction(cBuffer); )


  // init() just make preset of Network object internal data. Real starting maken on relaunch()
  Network.init(sysConfig.macAddress, sysConfig.ipAddress, sysConfig.ipAddress, sysConfig.ipGateway, sysConfig.ipNetmask, sysConfig.useDHCP);
  //netServer.begin();

  DTSL(
    //Serial.println(F("Wait for network warming up... "));
#if defined(FEATURE_SYSTEM_RTC_ENABLE)
    Serial.print(F("Timezone: ")); Serial.println(sysConfig.tzOffset, DEC);
#endif
  )

  // 5. Other system parts initialization
  //
  // I/O ports initialization. Refer to "I/O PORTS SETTING SECTION" in src/cfg_tune.h
  if (RESULT_IS_FAIL == initPortMode()) {
    DTSM( Serial.println(FSH_P(STRING_IO_ports_presets_is_wrong)); )
  }

  // Prepare external interrupts info structure and so
#ifdef INTERRUPT_USE
  initExtInt();
#endif

  // Watchdog activation
  __WATCHDOG( wdt_enable(constWtdTimeout); )

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
  __USER_FUNCTION( preLoopStageUserFunction(cBuffer); )

  while (true) {
    // reset watchdog every loop
    __WATCHDOG( wdt_reset(); )


    // Gather internal metrics periodically
    if ((millis() - prevSysMetricGatherTime) > constSysMetricGatherPeriod) {
      // When FEATURE_SYSINFO_ENABLE is disabled, compiler can be omit gatherSystemMetrics() sub (due find no operators inside) and trow exception
#ifndef GATHER_METRIC_USING_TIMER_INTERRUPT
      gatherSystemMetrics();
#endif
      correctVCCMetrics(getADCVoltage(ANALOG_CHAN_VBG));
      prevSysMetricGatherTime = millis();
      // update millis() rollovers to measure uptime if no RTC onboard
      millisRollover();
    }


    // Turn off state led if no errors occured in the current loop.
    // Otherwise - make LED blinked or just turn on
    if (ERROR_NONE == errorCode) {
      digitalWrite(constStateLedPin, LOW);
      sysMetrics.sysAlarmRisedTime = 0x00;
    } else {
      if (sysMetrics.sysAlarmRisedTime) {
        sysMetrics.sysAlarmRisedTime = millis();
      }

      // Call user function
      __USER_FUNCTION( alarmStageUserFunction(cBuffer, errorCode); )

#ifdef ON_ALARM_STATE_BLINK
      digitalWrite(constStateLedPin, millis() % blinkSettings[errorCode].allTime < blinkSettings[errorCode].onTime);
#else
      digitalWrite(constStateLedPin, HIGH);
#endif
    } // if (ERROR_NONE == errorCode) ... else

    if (needNetworkRelaunch) {
      needNetworkRelaunch = false;
      // Network module lost address (was resetted) or was not init (first time run)
      // Relaunch will reset hardware and re-init its registers
      sysMetrics.netPHYReinits++;
      DTSL( Serial.print(FSH_P(STRING_Network_module_reset_No)); Serial.println(sysMetrics.netPHYReinits); )
      // relaunch() returns code that can be used as status led blink type
      errorCode = Network.relaunch();
      if (ERROR_NONE == errorCode) {
        //Serial.println("relaunch");
        netServer.begin();
        // relaunch() returns true if DHCP is OK or Static IP used
        DTSL( Network.printNetworkInfo(); Serial.print(FSH_P(STRING_Password)); Serial.println(sysConfig.password, DEC); )
      }
    }

    // tick() subroutine is very important for UIPEthernet, and must be called often (every ~250ms). If ENC28J60 driver not used - this subroutine do nothing
    Network.tick();

    if ((millis() - prevPHYCheckTime) > constPHYCheckInterval) {
      //phyCheckInterval = constPHYCheckInterval;

      // do not forget SPI.begin() or system is hangs up
      if ((millis() - netDebugPrintTime) > consNetDebugPrintInterval) {
        DTSL( Network.printPHYState(); )
        netDebugPrintTime = millis();
      }

      // Network hardware is ok?
      if (Network.isPhyOk()) {
        // Network hardware settings is the same that MCU needs?
        if (!Network.isPhyConfigured()) {
          // Need relaunch on next loop in hardware settings error detected
          needNetworkRelaunch = true;
        } else { // if (!Network.isPhyConfigured())
          // PHY is works properly
          result = Network.maintain();
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
              DTSM( Serial.println(FSH_P(STRING_DHCP_renew_problem_occured)); )
          } // switch (result)
        } // if (!Network.isPhyConfigured())
      } else { // if (Network.isPhyOk())
        // PHY is disconnected or rise any error flag
        needNetworkRelaunch = true;
        // Right errorCode will be taken on relaunch()
      } // if (Network.isPhyOk())
      prevPHYCheckTime = millis();
    }

    // No DHCP problem found, but no data recieved or network activity for a long time
    if (ERROR_NONE == errorCode && (constNetIdleTimeout <= (millis() - prevNetActivityTime))) {
      errorCode = ERROR_NO_NET_ACTIVITY;
    }

    //*********************************************

#ifdef FEATURE_SERIAL_LISTEN_TOO
    // Network connections will processed if no data in Serial buffer exist
    if (Serial.available() <= 0) {
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
          __USER_FUNCTION( if (constUserFunctionCallInterval <= (uint32_t) (millis() - prevUserFuncCall)) {
          loopStageUserFunction(cBuffer);
            prevUserFuncCall = millis();
          } )
          // Jump to new round of main loop
          continue;
        }
        // reinit analyzer because previous session can be dropped or losted
        analyzeStream('\0', cBuffer, REINIT_ANALYZER, packetInfo);
        clientConnectTime = millis();
        // ToDo: add remoteIp() output
        DTSH( Serial.print(FSH_P(STRING_New)); Serial.print(FSH_P(STRING_client_No)); Serial.println(netClient); )
      }

      // Client will be dropped if its connection so slow. Then round must be restarted.
      if (constNetSessionTimeout <= (uint32_t) (millis() - clientConnectTime)) {
        DTSH( Serial.print(FSH_P(STRING_Drop)); Serial.print(FSH_P(STRING_client_No)); Serial.println(netClient); )
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
      incomingData = Serial.read();
    }
#endif

    result = analyzeStream(incomingData, cBuffer, NO_REINIT_ANALYZER, packetInfo);
    // result is true if analyzeStream() do not finished and need more data
    // result is false if EOL or trailing char detected or there no room in buffer or max number or args parsed...
    if (true == result) {
      continue;
    }
    // ethClient.connected() returns true even client is disconnected, but leave the data in the buffer.
    // Data can be readed, command will executed, but why get answer? If no recipient - why need to load MCU?
    // Will be better do nothing if client is disconnected while analyzing is finished
    // But checking must be disable for commands which coming in from the Serial. Otherwise commands will be never executed if no active network client exist.
#ifndef FEATURE_SERIAL_LISTEN_TOO
    if (!netClient.connected()) {
      continue;
    }
#endif
    // Fire up State led, than will be turned off on next loop
    digitalWrite(constStateLedPin, HIGH);
    processStartTime = millis();
    DTSM( uint32_t ramBefore = getRamFree(); )
    //DTSL( Serial.print(cBuffer); Serial.print(FSH_P(STRING_right_arrow)); ) // zbxd\1 is broke this output
    sysMetrics.sysCmdLast = executeCommand(netClient, cBuffer, sysConfig, packetInfo);

#ifdef FEATURE_REMOTE_COMMANDS_ENABLE
    // When system.run[] command is recieved, need to run another command, which taken from option #0 by cmdIdx() sub
    if (RESULT_IS_NEW_COMMAND == sysMetrics.sysCmdLast) {
      uint16_t k = 0;
      // simulate command recieving to properly string parsing
      // option #0 is memmoved() early to the buffer in the executeCommand()
      while (analyzeStream(cBuffer[k], cBuffer, NO_REINIT_ANALYZER, packetInfo)) {
        k++;
      }
      DTSL( Serial.print(cBuffer); Serial.print(FSH_P(STRING_right_arrow)); )
      sysMetrics.sysCmdLast = executeCommand(netClient, cBuffer, sysConfig, packetInfo);
    }
#endif
    processEndTime = millis();
    // use processEndTime as processDurationTime
    processEndTime = processEndTime - processStartTime ;
    DTSM( Serial.print(F("Spended: ")); Serial.print(processEndTime);
          Serial.print(F(" ms, ")); Serial.print((ramBefore - getRamFree()));
          Serial.println(F(" memory bytes"));
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
#endif
    sysMetrics.sysCmdLastExecTime = prevPHYCheckTime = prevNetActivityTime = millis();
    //blinkType = constBlinkNope;
    errorCode = ERROR_NONE;
  } // while(true)
}

/* ****************************************************************************************************************************


**************************************************************************************************************************** */
static int16_t executeCommand(NetworkClient & _netClient, char* _dst, netconfig_t& _sysConfig, packetInfo_t& _packetInfo) {

  char* payload;
  int8_t rc;
  uint8_t accessGranted = false;//, zabbixPacketType = false;
  uint16_t i;
  uint8_t cmdIdx;
  // duration option in the tone[] command is ulong
  int32_t argv[constArgC];
  // Zabbix use 64-bit numbers, but we can use only int32_t range. Error can be occurs on ltoa() call with value > long_int_max
  int32_t value = 0x00;
  //NetworkAddress tmpAddress;
  uint32_t payloadLenght;

  char** optarg = _packetInfo.optarg;

  // what about PACKET_TYPE_NONE ?
  payload = (PACKET_TYPE_ZABBIX == _packetInfo.type) ? &_dst[ZBX_HEADER_LENGTH] : &_dst[0];

  cmdIdx  = arraySize(commands);
  DTSD (Serial.print(F("Packet type: "));  Serial.println((PACKET_TYPE_ZABBIX == _packetInfo.type) ? F("Zabbix") : F("Plain_text")); )
  DTSL (Serial.print(F("Packet payload: "));  Serial.println(payload); )

  // Search specified command index in the list of implemented functions
  rc = 0x01;
  while (cmdIdx && (0 != rc)) {
    cmdIdx--;
    PGM_P cmdName = pgm_read_word(&(commands[cmdIdx].name));
    rc = strcmp_P(payload, cmdName);
    DTSD( Serial.print(F("#")); Serial.print(FSH_P(STRING_HEX_Prefix));  Serial.print(cmdIdx , HEX); Serial.print(FSH_P(STRING_right_arrow)); Serial.println(FSH_P(cmdName)); )
  }
  cmdIdx = pgm_read_byte(&(commands[cmdIdx].idx));

  // If no suitable command found - do nothing, jump to the finish where show result ZBX_NOTSUPPORTED
  if (0x00 >= cmdIdx) {
    rc = ZBX_NOTSUPPORTED;
    goto finish;
  }

  rc = RESULT_IS_FAIL;
  sysMetrics.sysCmdCount++;

  DTSM( Serial.print(FSH_P(STRING_Execute_command_No)); Serial.print(cmdIdx, HEX); Serial.print(FSH_P(STRING_right_arrow)); Serial.println(payload); )

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
        strcpy(payload, _sysConfig.hostname);
        rc = RESULT_IS_BUFFERED;
        goto finish;
      }

    case CMD_ZBX_AGENT_VERSION: {
        //
        //  agent.version
        //
        strcpy_P(payload, constZbxAgentVersion);
        rc = RESULT_IS_BUFFERED;
        goto finish;
      }

    case CMD_SYSTEM_UPTIME: {
        //
        //  system.uptime
        //
        // Returns uptime in seconds
        //  value  = ((uint32_t) millisRollover() * (UINT32_MAX / 1000UL) + (millis() / 1000UL));
        value  = ((uint32_t) millisRollover() * UINT32_MAX + millis()) / 1000UL;

#ifdef FEATURE_SYSTEM_RTC_ENABLE
        // Just rewrite millises uptime by another, which taken from system RTC
        if (0x00 != sysMetrics.sysStartTimestamp) {
          SoftTWI.reconfigure(constSystemRtcSDAPin, constSystemRtcSCLPin);
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
        strcpy_P(payload, PSTR(BOARD));
        rc = RESULT_IS_BUFFERED;
        goto finish;
      }

    case CMD_NET_PHY_NAME: {
        //
        //  net.phy.name
        //
        strcpy_P(payload, PSTR(PHY_MODULE_NAME));
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

    case CMD_SYS_CMD_TIMEMAX_N: {
        //
        //  sys.cmd.timemax.n
        //
        ultoa(sysMetrics.sysCmdTimeMaxN, payload, 16);
        rc = RESULT_IS_BUFFERED;
        goto finish;
      }

    case CMD_SYS_RAM_FREE: {
        //
        //  sys.ram.free
        //
        //  That metric must be collected periodically to avoid returns always same data
        value = sysMetrics.sysRamFree;
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }

    case CMD_SYS_RAM_FREEMIN: {
        //
        //  sys.ram.freemin
        //
        // Without ATOMIC_BLOCK block using sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] variable can be changed in interrupt on reading
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          value = sysMetrics.sysRamFreeMin;
        }
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }
#endif

    case CMD_SYS_VCC: {
        //
        // sys.vcc
        //
        // Take VCC
        value = getADCVoltage(ANALOG_CHAN_VBG);
        // VCC may be bigger than max or smaller than min.
        // To avoid wrong results and graphs in monitoring system - correct min/max metrics
        correctVCCMetrics(value);
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

#ifdef FEATURE_SYSTEM_RTC_ENABLE

    case CMD_SYSTEM_LOCALTIME: {
        //
        //  system.localtime
        //  Zabbix wants UTC as localtime
        //
        if (getUnixTime(&SoftTWI, (uint32_t*) &value)) {
          rc = RESULT_IS_UNSIGNED_VALUE;
        }
        goto finish;
      }
#endif // FEATURE_SYSTEM_RTC_ENABLE
  } // switch (cmdIdx) part #1

  // ***************************************************************************************************************
  // Command with options take more time
  // batch convert args to number values
  i = constArgC;
  while (i) {
    i--;
    argv[i] = strtol((char*) optarg[i], NULL, 0);

    DTSH(
      Serial.print(F("argv[")); Serial.print(i); Serial.print(F("] => \""));
    if ('\0' == *optarg[i]) {
    Serial.print(F("<null>"));
    } else {
      Serial.print((char*) optarg[i]);
    }
    Serial.print(F("\" => ")); Serial.println(argv[i]);
    )
  }

  // Check rights for password protected action
  accessGranted = (!_sysConfig.useProtection || (uint32_t) argv[0] == _sysConfig.password);

  switch (cmdIdx) {
#ifdef FEATURE_USER_FUNCTION_PROCESSING
    case CMD_USER_RUN: {
        //
        //  user.run[option#0, option#1, option#2, option#3, option#4, option#5]
        //
        rc = executeCommandUserFunction(payload, optarg, argv, &value);
        goto finish;
      }

#endif // FEATURE_USER_FUNCTION_PROCESSING

#ifdef FEATURE_REMOTE_COMMANDS_ENABLE
    case CMD_SYSTEM_RUN: {
        //
        //  system.run["newCommand"]
        //
        if ('\0' == *optarg[0]) {
          goto finish;
        }
        // take length of 0-th arg + 1 byte for '\0'
        uint8_t k = strlen((char*) optarg[0]) + 1;
        // move it to begin of buffer to using as new incoming command
        // Note: ~8bytes can be saved with copying bytes in while() cycle. But source code will not beauty
        memmove(payload, optarg[0], k);
        payload[k] = '\n';
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
        if (! isSafePin(argv[0])) {
          goto finish;
        }
        analogWrite(argv[0], argv[1]);
        rc = RESULT_IS_OK;
        goto finish;
      }

    case CMD_ARDUINO_ANALOGREAD: {
        //
        //  analogRead[pin, analogReferenceSource, mapToLow, mapToHigh]
        //
#ifdef FEATURE_AREF_ENABLE
        // change source of the reference voltage if its given
        if ('\0' != *optarg[1]) {
          analogReference(argv[1]);
          delayMicroseconds(2000);
        }
#endif
        if (! isSafePin(argv[0])) {
          goto finish;
        }
        value = analogRead(argv[0]);
        if ('\0' != *optarg[2] && '\0' != *optarg[3]) {
          value = map(value, constAnalogReadMappingLowValue, constAnalogReadMappingHighValue, argv[2], argv[3]);
        }
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }

#endif // FEATURE_ARDUINO_BASIC_ENABLE

#ifdef FEATURE_AREF_ENABLE
    case CMD_ARDUINO_ANALOGREFERENCE: {
        //
        //  analogReference[source]
        //
        analogReference(argv[0]);
        rc = RESULT_IS_OK;
        goto finish;
      }
#endif

#ifdef FEATURE_ARDUINO_BASIC_ENABLE
    case CMD_ARDUINO_DELAY: {
        //
        //  delay[time]
        //
        delay(argv[0]);
        rc = RESULT_IS_OK;
        goto finish;
      }

    case CMD_ARDUINO_DIGITALWRITE: {
        //
        //  digitalWrite[pin, value, testPin, testValue]
        //
        if (isSafePin(argv[0])) {
          // turn on or turn off logic on pin
          pinMode(argv[0], OUTPUT);
          digitalWrite(argv[0], argv[1]);
          rc = RESULT_IS_OK;
        }
        goto finish;
      }

    case CMD_ARDUINO_DIGITALREAD: {
        //
        //  digitalRead[pin]
        //
        if (isSafePin(argv[0])) {
          pinMode(argv[0], INPUT);
          value = digitalRead(argv[0]);
          rc = RESULT_IS_UNSIGNED_VALUE;
        }
        goto finish;
      }

#endif // FEATURE_ARDUINO_BASIC_ENABLE

#ifdef FEATURE_TONE_ENABLE
    case CMD_ARDUINO_TONE: {
        //
        //  tone[pin, frequency, duration]
        //
        if (! isSafePin(argv[0])) {
          goto finish;
        }
        rc = RESULT_IS_OK;
        if ('\0' != *optarg[2]) {
          tone(argv[0], argv[1], argv[2]);
          goto finish;
        }
        tone(argv[0], argv[1]);
        goto finish;
      }

    case CMD_ARDUINO_NOTONE: {
        //
        //  noTone[pin]
        //
        if (! isSafePin(argv[0])) {
          goto finish;
        }
        noTone(argv[0]);
        rc = RESULT_IS_OK;
        goto finish;
      }
#endif

#ifdef FEATURE_RANDOM_ENABLE
    case CMD_ARDUINO_RANDOMSEED: {
        //
        //  randomSeed[value]
        //
        randomSeed((0 == argv[0]) ? (int32_t) millis() : argv[0]);
        rc = RESULT_IS_OK;
        goto finish;
      }

    case CMD_ARDUINO_RANDOM: {
        //
        //  random[min, max]
        //
        //  !! random return long
        value = ('\0' == *optarg[1]) ? random(argv[0]) : random(argv[0], argv[1]);
        rc = RESULT_IS_UNSIGNED_VALUE;
        goto finish;
      }
#endif // FEATURE_RANDOM_ENABLE

#ifdef FEATURE_EEPROM_ENABLE
    case CMD_SET_HOSTNAME: {
        //
        //  set.hostname[password, hostname]
        //
        // payload[_argOffset[1]] != \0 if argument #2 given
        if (!accessGranted || '0' != *optarg[1]) {
          strncpy(_sysConfig.hostname, optarg[1], constAgentHostnameMaxLength);
          // strncpy() can do not copy trailing \0
          _sysConfig.hostname[constAgentHostnameMaxLength] = '\0';
          rc = RESULT_IS_UNSTORED_IN_EEPROM;
        }
        goto finish;
      }

    case CMD_SET_PASSWORD: {
        //
        //  set.password[oldPassword, newPassword]
        //
        if (accessGranted && '\0' != *optarg[1]) {
          // take new password from argument #2
          _sysConfig.password = argv[1];
          rc = RESULT_IS_UNSTORED_IN_EEPROM;
        }
        goto finish;
      }

    case CMD_SET_SYSPROTECT: {
        //
        //  set.sysprotect[password, protection]
        //
        if (accessGranted && '\0' != *optarg[1]) {
          _sysConfig.useProtection = (1 == argv[1]) ? true : false;
          rc = RESULT_IS_UNSTORED_IN_EEPROM;
        }
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
        // argv[0] data contain in payload[_argOffset[1]] placed from _argOffset[0]
        newConfig.useDHCP = (0 != (uint8_t) argv[1]);
        // ip, netmask and gateway have one structure - 4 byte
        // take 6 bytes from second argument of command and use as new MAC-address
        // if convertation is failed (sub return -1) variable must be falsed too via logic & operator
        success = (success) ? (6 == hstoba((uint8_t *) &newConfig.macAddress, optarg[2])) : false;
        // If string to which point optarg[3] can be converted to valid NetworkAddress - just do it.
        // Otherwize (string can not be converted) _sysConfig.ipAddress will stay untouched;
        success = (success) ? (strToNetworkAddress((char*) optarg[3], newConfig.ipAddress)) : false;
        success = (success) ? (strToNetworkAddress((char*) optarg[4], newConfig.ipNetmask)) : false;
        success = (success) ? (strToNetworkAddress((char*) optarg[5], newConfig.ipGateway)) : false;
        // if any convert operation failed - just do not return "need to eeprom write" return code
        if (success) {
          rc = (saveConfigToEEPROM(newConfig)) ? RESULT_IS_OK : DEVICE_ERROR_EEPROM_CORRUPTED;
        }
        goto finish;
      }
#endif // FEATURE_EEPROM_ENABLE

#ifdef FEATURE_SYSTEM_RTC_ENABLE
    case CMD_SET_LOCALTIME:
      //
      //  set.localtime[password, unixTimestamp, tzOffset]
      //  set.localtime must take unixTimestamp as UTC, because system.localtime command returns UTC too
      //
      if (!accessGranted) {
        goto finish;
      }
      // i used as succes bool variable
      i = true;
#ifdef FEATURE_EEPROM_ENABLE
      // tzOffset is defined?
      if ('\0' != *optarg[2]) {
        _sysConfig.tzOffset = (int16_t) argv[2];
        // Save config to EEPROM
        i = saveConfigToEEPROM(_sysConfig);
        if (i) {
          set_zone(_sysConfig.tzOffset);
          rc = RESULT_IS_OK;
        }
      }
#endif // FEATURE_EEPROM_ENABLE

      // unixTimestamp option is given?
      if ('\0' != *optarg[1] && i) {
        // tzOffset is defined and stored sucesfully
        if (setUnixTime(&SoftTWI, argv[1])) {
          rc = RESULT_IS_OK;
        }
      }
      goto finish;
#endif // FEATURE_SYSTEM_RTC_ENABLE

    case CMD_SYS_PORTWRITE:
      //
      //  portWrite[port, value]
      //
      i = argv[0] - 96;
      if (PORTS_NUM >= i) {
        goto finish;
      }
      rc = writeToPort((byte) i, argv[1]);
      goto finish;

#ifdef FEATURE_SHIFTOUT_ENABLE
    case CMD_SYS_SHIFTOUT:
      //
      //  shiftOut[dataPin, clockPin, latchPin, bitOrder, compressionType, data]
      //

      // i variable used as latchPinDefined
      i = ('\0' != argv[2]) && isSafePin(argv[2]);
      if (isSafePin(argv[0]) &&  isSafePin(argv[1])) {
        if (i) {
          pinMode(argv[2], OUTPUT);
          digitalWrite(argv[2], LOW);
        }
        rc = shiftOutAdvanced(argv[0], argv[1], argv[3], argv[4], (uint8_t*) optarg[5]);
        if (i) {
          digitalWrite(argv[2], HIGH);
        }
      }
      goto finish;
#endif

#ifdef FEATURE_WS2812_ENABLE
    case CMD_WS2812_SENDRAW:
      //
      //  WS2812.sendRaw[dataPin, compressionType, data]
      //  !!! need to increase ARGS_PART_SIZE, because every encoded LED color take _six_ HEX-chars => 10 leds stripe take 302 (2+50*6) byte of incoming buffer only
      //
      // Tested on ATmega328@16 and 8 pcs WS2812 5050 RGB LED bar
      if (isSafePin(argv[0])) {
        // 4-th param equal 0x00 mean that buffer not contain raw color bytes and must be prepared (converted from "0xABCDEF.." string)
        rc = WS2812Out(argv[0], argv[1], (uint8_t*) optarg[2], 0x00);
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
    case CMD_SYSTEM_HW_CPU:
      //
      //  system.hw.cpu[metric]
      //
      rc = RESULT_IS_BUFFERED;
      if (0 == strcmp_P(optarg[0], PSTR("id"))) {
        // Read 10 bytes with step 1 (0x0E..0x17) of the signature row <= http://www.avrfreaks.net/forum/unique-id-atmega328pb
        getBootSignatureAsHexString(payload, 0x0E, 10, 1);
      } else if (0 == strcmp_P(optarg[0], PSTR("freq"))) {
        // Return back CPU frequency
        value = F_CPU;
        rc = RESULT_IS_UNSIGNED_VALUE;
      } else if (0 == strcmp_P(optarg[0], PSTR("model"))) {
        // Read 3 bytes with step 2 (0x00, 0x02, 0x04) of the signature row <= http://www.avrfreaks.net/forum/device-signatures
        getBootSignatureAsHexString(payload, 0x00, 3, 2);
      } else {
        // Return back CPU name
        strcpy_P(payload, PSTR(_AVR_CPU_NAME_));
      }
      goto finish;

    case CMD_SYS_CMD_COUNT:
      //
      //  sys.cmd.count
      //
      if (argv[0]) {
        sysMetrics.sysCmdCount = 0;
      }
      value = sysMetrics.sysCmdCount;
      rc = RESULT_IS_UNSIGNED_VALUE;
      goto finish;

    case CMD_SYS_CMD_TIMEMAX:
      //
      //  sys.cmd.timemax[resetCounter]
      //
      if (*optarg[0]) {
        sysMetrics.sysCmdTimeMax = sysMetrics.sysCmdTimeMaxN = 0;
      }
      value = sysMetrics.sysCmdTimeMax;
      rc = RESULT_IS_UNSIGNED_VALUE;
      goto finish;

#endif // FEATURE_SYSINFO_ENABLE

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
    case CMD_EXTINT_COUNT:
      //
      //  extInt.count[intPin, mode]
      //  extInt.count[3,1]
      //
      //  Unfortunately, (rc == RESULT_IS_UNSIGNED_VALUE && value == 0) and (rc == RESULT_IS_FAIL) are looks equal for zabbix -> '0'
      //
      if (isSafePin(argv[0])) {
        rc = manageExtInt((uint32_t*) &value, argv[0], argv[1]);
        rc = RESULT_IS_UNSIGNED_VALUE;
      }
      goto finish;

#endif // FEATURE_EXTERNAL_INTERRUPT_ENABLE

#ifdef FEATURE_OW_ENABLE
    case CMD_OW_SCAN:
      //
      //  OW.scan[pin]
      //

      // real buffer size not take in account and no overflow check in scanOneWire()!!
      if (isSafePin(argv[0])) {
        // 
        value = scanOneWire(argv[0], (uint8_t*) payload, constPayloadSize);
        if (value) {
          rc = RESULT_IS_BUFFERED;
        }
      }
      goto finish;
#endif // FEATURE_ONEWIRE_ENABLE


#ifdef FEATURE_DS18X20_ENABLE
    case CMD_DS18X20_TEMPERATURE:
      //
      //  DS18x20.temperature[pin, resolution, id]
      //
      if (!isSafePin(argv[0])) {
        goto finish;
      }

      uint8_t dsAddr[8];
      dsAddr[0] = 0;
      // Convert sensor ID (if its given) from HEX string to byte array (DeviceAddress structure) and validate (sub not finished) it.
      // Sensor ID is equal DeviceAddress.
      // if convertation not successfull or ID not valid - return DEVICE_ERROR_WRONG_ID
      if (('\0' != *optarg[2]) && (8 != hstoba(dsAddr, optarg[2]))) {
        rc = DEVICE_ERROR_WRONG_ID;
      } else {
        rc = getDS18X20Metric(argv[0], argv[1], dsAddr, payload);
      }
      goto finish;
#endif // FEATURE_DS18X20_ENABLE

#ifdef FEATURE_MHZXX_PWM_ENABLE
    case CMD_MHZXX_PWM_CO2:
      //
      //  MHZxx.PWM.CO2[pin, range]
      //
      if (isSafePin(argv[0])) {
        rc = getMHZxxMetricPWM(argv[0], argv[1], (uint8_t*) payload);
      }
      goto finish;
#endif // FEATURE_MHZXX_PWM_ENABLE

#ifdef FEATURE_MHZXX_UART_ENABLE
    case CMD_MHZXX_UART_CO2:
      //
      //  MHZxx.UART.CO2[rxPin, txPin]
      //
      if (isSafePin(argv[0]) && isSafePin(argv[1])) {
        rc = getMHZxxMetricUART(argv[0], argv[1], (uint8_t*) payload);
      }
      goto finish;
#endif // FEATURE_MHZXX_UART_ENABLE


#ifdef FEATURE_DHT_ENABLE
    case CMD_DHT_HUMIDITY:
      //
      //  DHT.humidity[pin, model]
      //
      if (isSafePin(argv[0])) {
        rc = getDHTMetric(argv[0], argv[1], SENS_READ_HUMD, payload);
      } else {
        rc = DEVICE_ERROR_CONNECT;
      }
      goto finish;

    case CMD_DHT_TEMPERATURE:
      //
      //  DHT.temperature[pin, model]
      //
      if (isSafePin(argv[0])) {
        rc = getDHTMetric(argv[0], argv[1], SENS_READ_TEMP, payload);
      } else {
        rc = DEVICE_ERROR_CONNECT;
      }
      goto finish;

#endif // FEATURE_DHT_ENABLE

#ifdef FEATURE_MAX7219_ENABLE
    case CMD_MAX7219_WRITE:
      //
      //  MAX7219.write[dataPin, clockPin, loadPin, intensity, data]
      //
      if (isSafePin(argv[0]) && isSafePin(argv[1]) && isSafePin(argv[2])) {
        writeToMAX7219(argv[0], argv[1], argv[2], argv[3], optarg[4]);
        rc = RESULT_IS_OK;
      }
      goto finish;
#endif // FEATURE_MAX7219_ENABLE

#ifdef FEATURE_MAX6675_ENABLE
    case CMD_MAX6675_TEMPERATURE:
      //
      //  max6675.temperature[dataPin, clockPin, csPin]
      // max6675.temperature[4,6,5]
      //
      if (isSafePin(argv[0]) && isSafePin(argv[1]) && isSafePin(argv[2])) {
        rc = getMAX6675Metric(argv[0], argv[1], argv[2], SENS_READ_TEMP, payload);
      }
      goto finish;

#endif // FEATURE_MAX6675_ENABLE

#ifdef FEATURE_ACS7XX_ENABLE
    case CMD_ACS7XX_ZC:
      //
      //  acs7xx.zc[sensorPin, sampleTime, refVoltage]
      //
      if (!isSafePin(argv[0])) {
        goto finish;
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

      if ('\0' != *optarg[1]) {
        argv[1] = 10000UL;
      }
      // if refVoltage skipped - use DEFAULT source
      if ('\0' != *optarg[2]) {
        argv[2] = DEFAULT;
      }
      rc = getACS7XXMetric(argv[0], argv[1], argv[2], SENS_READ_ZC, 0x00, 0x00, payload);
      goto finish;

    case CMD_ACS7XX_AC:
      //
      //  acs7xx.ac[sensorPin, sampleTime, refVoltage, sensitivity, zeroPoint]
      //
      if (!isSafePin(argv[0])) {
        goto finish;
      }
      if ('\0' != *optarg[1]) {
        argv[1] = 10000UL;
      }
      // if refVoltage skipped - use DEFAULT source
      if ('\0' != *optarg[2]) {
        argv[2] = DEFAULT;
      }
      rc = getACS7XXMetric(argv[0], argv[1], argv[2], SENS_READ_AC, argv[3], argv[4], payload);
      goto finish;

    case CMD_ACS7XX_DC:
      //
      //  acs7xx.dc[sensorPin, sampleTime, refVoltage, sensitivity, zeroPoint]
      //
      if (!isSafePin(argv[0])) {
        goto finish;
      }
      if ('\0' != *optarg[1]) {
        argv[1] = 10000UL;
      }
      // if refVoltage skipped - use DEFAULT source
      if ('\0' != *optarg[2]) {
        argv[2] = DEFAULT;
      }
      rc = getACS7XXMetric(argv[0], argv[1], argv[2], SENS_READ_DC, argv[3], argv[4], payload);
      goto finish;

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
      goto finish;

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
      rc = sendRawByIR(argv[1], argv[2], optarg[3]);
      //     }
      goto finish;
#endif // FEATURE_IR_ENABLE

#ifdef FEATURE_SERVO_ENABLE
    case CMD_SERVO_TURN:
      //
      //  Servo.turn[servoPin, targetAnglePulseWidth, turnTime, holdTime, returnAnglePulseWidth]
      //
      //  Need to add updateFrequency as argv[5] ?
      //
      //  servo.turn[5, 1500, 500, 2000, 680]
      //
      if (isSafePin(argv[0])) {
        uint16_t targetAnglePulseWidth, returnAnglePulseWidth;
        uint32_t holdTime, turnTime;
        turnTime = ('\0' != *optarg[2] && argv[2] > 0) ? argv[2] : 0;
        holdTime = ('\0' != *optarg[3] && argv[3] > 0) ? argv[3] : 0;
        targetAnglePulseWidth = ('\0' != *optarg[1] && argv[1] > 0) ? argv[1] : 0;
        returnAnglePulseWidth = ('\0' != *optarg[4] && argv[4] > 0) ? argv[4] : 0;
        rc = servoTurn(argv[0], targetAnglePulseWidth, turnTime, holdTime, returnAnglePulseWidth);
      }
      goto finish;
#endif // FEATURE_SERVO_ENABLE

#ifdef FEATURE_RELAY_ENABLE
    case CMD_PULSE:
      //
      //  pulse[targetPin, targetState, holdTime, returnState]
      //
      // targetPin is safe?
      if (!isSafePin(argv[0])) {
        goto finish;
      }
      // make targetState boolean
      argv[1] = argv[1] ? 1 : 0;

      // if no returnState is specified: returnState = !targetState
      if ('\0' == *optarg[3]) {
        argv[3] = ! argv[1];
      }

      // holdTime is specified?
      if ('\0' == *optarg[2]) {
        argv[2] = 0;
      }
      rc = pulse(argv[0], argv[1], argv[2], argv[3]);
      goto finish;

    case CMD_RELAY:
      //
      //  relay[relayPin, relayState, testPin, testState, testPinMode]
      //  relay[4,1,2,1]
      //  relay[4,1,2,0,1]

      // relayPin is safe?
      if (!isSafePin(argv[0])) {
        goto finish;
      }

      // testPin is specified?
      if ('\0' == *optarg[2]) {
        argv[2] = -1;
        // testPin is safe?
      } else if (!isSafePin(argv[2])) {
        goto finish;
      }
      // testState is specified?
      if ('\0' == *optarg[3]) {
        argv[3] = -1;
      }
      // pullup is need?
      if ('\0' != *optarg[4] && argv[4] != 0) {
        argv[4] = INPUT_PULLUP;
      } else {
        argv[4] = INPUT;
      }
      rc = relay(argv[0], argv[1], argv[2], argv[3], argv[4]);
      goto finish;
#endif // FEATURE_RELAY_ENABLE


    default:
      // ************************************************************************************************************************************
      // Following commands use <argv[0]> or <argv[1]> pins for sensor handling (UART, I2C, etc) and these pins can be disabled in port_protect[] array
      //  Otherwise - processing is failed
      rc = RESULT_IS_FAIL;
      if ('\0' == *optarg[0] || '\0' == *optarg[1] || !isSafePin(argv[0]) || !isSafePin(argv[1])) {
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
          goto finish;
#endif // FEATURE_INCREMENTAL_ENCODER_ENABLE

#ifdef FEATURE_ULTRASONIC_ENABLE
        case CMD_ULTRASONIC_DISTANCE:
          //
          //  ultrasonic.distance[triggerPin, echoPin]
          //
          value = getUltrasonicMetric(argv[0], argv[1]);
          rc = RESULT_IS_UNSIGNED_VALUE;
          goto finish;
#endif // FEATURE_ULTRASONIC_ENABLE

#ifdef FEATURE_PZEM004_ENABLE
        //
        //  0xC0A80101 - an default IP address for PZEM (192.168.1.1)
        //
        case CMD_PZEM004_CURRENT:
          //
          //  pzem004.current[rxPin, txPin, addr]
          //
          // payload cast to (uint8_t*) to use with subroutine math and SoftwareSerial subs, because used instead sub's internal buffer and save a little RAM size.
          // Its will be casted to char* inside at moment when its need
          rc = getPZEM004Metric(argv[0], argv[1], SENS_READ_AC, optarg[2], (uint8_t*) payload);
          goto finish;

        case CMD_PZEM004_VOLTAGE:
          //
          //  pzem004.voltage[rxPin, txPin, addr]
          //
          rc = getPZEM004Metric(argv[0], argv[1], SENS_READ_VOLTAGE, optarg[2], (uint8_t*) payload);
          goto finish;

        case CMD_PZEM004_POWER:
          //
          //  pzem004.power[rxPin, txPin, addr]
          //
          rc = getPZEM004Metric(argv[0], argv[1], SENS_READ_POWER, optarg[2], (uint8_t*) payload);
          goto finish;

        case CMD_PZEM004_ENERGY:
          //
          //  pzem004.energy[rxPin, txPin, addr]
          //
          rc = getPZEM004Metric(argv[0], argv[1], SENS_READ_ENERGY, optarg[2], (uint8_t*) payload);
          goto finish;

        case CMD_PZEM004_SETADDR:
          //
          //  pzem004.setAddr[rxPin, txPin, addr]
          //
          rc = getPZEM004Metric(argv[0], argv[1], SENS_CHANGE_ADDRESS, optarg[2], (uint8_t*) payload);
          goto finish;
#endif // FEATURE_PZEM004_ENABLE

#ifdef FEATURE_UPS_APCSMART_ENABLE
        case CMD_UPS_APCSMART:
          //
          //  ups.apcsmart[rxPin, txPin, command]
          //    command - HEX or ASCII
          //
          rc = getAPCSmartUPSMetric(argv[0], argv[1], (uint8_t*) optarg[2], (uint8_t*) payload);
          goto finish;
#endif // FEATURE_UPS_APCSMART_ENABLE

#ifdef FEATURE_UPS_MEGATEC_ENABLE
        case CMD_UPS_MEGATEC:
          //
          //  ups.megatec[rxPin, txPin, command, fieldNumber]
          //    command - HEX or ASCII
          //
          rc = getMegatecUPSMetric(argv[0], argv[1], optarg[2], argv[3], (uint8_t*) payload);
          goto finish;
#endif // FEATURE_UPS_APCSMART_ENABLE

#ifdef FEATURE_DFPLAYER_ENABLE
        case CMD_DFPLAYER_RUN:
          //
          //  dfplayer.run[rxPin, txPin, command, option, volume]
          //  dfplayer.run[4, 5, 0x03, 0x02, 30]
          //
          // use -1 if no volume specified. Otherwize - volume will be corrected.
          rc = runDFPlayerMini(argv[0], argv[1], argv[2], argv[3], (('\0' == *optarg[4]) ? -0x01 : argv[4]), (uint8_t*) payload);
          goto finish;
#endif // FEATURE_DFPLAYER_ENABLE

#ifdef FEATURE_PLANTOWER_PMS_ALL_ENABLE
        case CMD_PLANTOWER_PMS_ALL:
          //
          //  PMS.All[rxPin, txPin]
          //
          rc = getPlantowerPM25AllMetrics(argv[0], argv[1], payload);
          goto finish;
#endif // FEATURE_PLANTOWER_PMS_ALL_ENABLE

          // default:
          //  goto finish;
      } // switch (cmdIdx) Non-I2C related commands block

      // ************************************************************************************************************************************
      //  I2C-related commands have additional processing
#ifdef TWI_USE
      // Otherwise - TWI interface can be reconfigured with new pins
      SoftTWI.reconfigure(argv[0], argv[1]);

      int8_t i2CAddress;

      i2CAddress = (('\0' != *optarg[2]) ? abs((int8_t) argv[2]) : I2C_NO_ADDR_SPECIFIED);

#ifdef FEATURE_I2C_ENABLE
      // this war is used only in FEATURE_I2C_ENABLE blocks
      int16_t i2CRegister;
      i2CRegister = ('\0' != *optarg[3]) ? (int16_t) argv[3] : I2C_NO_REG_SPECIFIED;
#endif // FEATURE_I2C_ENABLE


      switch (cmdIdx) {
#ifdef FEATURE_I2C_ENABLE
        case CMD_I2C_SCAN:
          //
          //  I2C.scan[sdaPin, sclPin]
          //
          value = scanI2C(&SoftTWI, (uint8_t*) payload);
          if (value) {
            rc = RESULT_IS_BUFFERED;
          }
          goto finish;

        case CMD_I2C_WRITE:
          //
          // i2c.write(sdaPin, sclPin, i2cAddress, register, data, numBytes)
          //
          if (I2C_NO_ADDR_SPECIFIED == i2CAddress) {
            goto finish;
          }
          rc = writeValueToI2C(&SoftTWI, i2CAddress, i2CRegister, (uint32_t) argv[4], (uint8_t) argv[5]);
          goto finish;

        case CMD_I2C_READ:
          //
          // i2c.read(sdaPin, sclPin, i2cAddress, register, numBytes, numberOfReadings)
          //
          if (I2C_NO_ADDR_SPECIFIED == i2CAddress) {
            goto finish;
          }
          rc = readValueFromI2C(&SoftTWI, i2CAddress, i2CRegister, (uint32_t*) &value, argv[4], (('\0' != *optarg[5]) ? argv[5] : 0x00));
          goto finish;

        case CMD_I2C_BITWRITE:
          //
          //  i2c.bitWrite(sdaPin, sclPin, i2cAddress, register, bitNo, value)
          //
          if (I2C_NO_ADDR_SPECIFIED == i2CAddress) {
            goto finish;
          }
          rc = bitWriteToI2C(&SoftTWI, i2CAddress, i2CRegister, argv[4], argv[5]);
          goto finish;

        case CMD_I2C_BITREAD:
          //
          //  i2c.bitRead(sdaPin, sclPin, i2cAddress, register, bitNo)
          //
          if (I2C_NO_ADDR_SPECIFIED == i2CAddress) {
            goto finish;
          }
          rc = bitReadFromI2C(&SoftTWI, i2CAddress, i2CRegister, argv[4], (uint8_t*) &value);
          goto finish;

#endif // FEATURE_I2C_ENABLE

#ifdef FEATURE_BMP_ENABLE
        case CMD_BMP_PRESSURE:
          //
          //  BMP.Pressure[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getBMPMetric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : BMP180_I2C_ADDRESS), argv[3], argv[4], SENS_READ_PRSS, payload);
          goto finish;

        case CMD_BMP_TEMPERATURE:
          //
          // BMP.Temperature[sdaPin, sclPin, i2cAddress, overSampling]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getBMPMetric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : BMP180_I2C_ADDRESS), argv[3], argv[4], SENS_READ_TEMP, payload);
          goto finish;

#ifdef SUPPORT_BME280_INCLUDE
        case CMD_BME_HUMIDITY:
          //
          //  BME.Humidity[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getBMPMetric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : BME280_I2C_ADDRESS), argv[3], argv[4], SENS_READ_HUMD, payload);
          goto finish;
#endif // SUPPORT_BME280_INCLUDE 
#endif // FEATURE_BMP_ENABLE  

#ifdef FEATURE_BH1750_ENABLE
        case CMD_BH1750_LIGHT:
          //
          //  BH1750.light[sdaPin, sclPin, i2cAddress, mode]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getBH1750Metric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : BH1750_I2C_ADDRESS), argv[3], SENS_READ_LUX, payload);
          goto finish;
#endif // FEATURE_BH1750_ENABLE

#ifdef FEATURE_INA219_ENABLE
        case CMD_INA219_BUSVOLTAGE:
          //
          //  INA219.BusVoltage[sdaPin, sclPin, i2cAddress, voltageRange, maxCurrent]
          //
          rc = getINA219Metric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : INA219_I2C_ADDRESS), argv[3], argv[4], SENS_READ_BUS_VOLTAGE, payload);
          goto finish;

        case CMD_INA219_CURRENT:
          //
          //  INA219.Current[sdaPin, sclPin, i2cAddress, voltageRange, maxCurrent]
          //
          rc = getINA219Metric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : INA219_I2C_ADDRESS), argv[3], argv[4], SENS_READ_DC, payload);
          goto finish;

        case CMD_INA219_POWER:
          //
          //  INA219.Power[sdaPin, sclPin, i2cAddress, voltageRange, maxCurrent]
          //
          rc = getINA219Metric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : INA219_I2C_ADDRESS), argv[3], argv[4], SENS_READ_POWER, payload);
          goto finish;

#endif // FEATURE_INA219_ENABLE

#ifdef FEATURE_PCF8574_LCD_ENABLE
        case CMD_PCF8574_LCDPRINT:
          //
          //  PCF8574.LCDPrint[sdaPin, sclPin, i2cAddress, lcdBacklight, lcdType, data]
          //
          if (I2C_NO_ADDR_SPECIFIED == i2CAddress) {
            goto finish;
          }
          rc = printToPCF8574LCD(&SoftTWI, i2CAddress, argv[3], argv[4], optarg[5]);
          // Store current time when on LCD was printed something to calculation in loopStageUserFunction() (for example) 'no refresh timeout' properly
          sysMetrics.sysLCDLastUsedTime = millis();
          goto finish;

#endif // FEATURE_PCF8574_LCD_ENABLE

#ifdef FEATURE_SHT2X_ENABLE
        case CMD_SHT2X_HUMIDITY:
          //
          //  SHT2X.Humidity[sdaPin, sclPin, i2cAddress]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getSHT2XMetric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : SHT2X_I2C_ADDRESS), SENS_READ_HUMD, payload);
          goto finish;

        case CMD_SHT2X_TEMPERATURE:
          //
          //  SHT2X.Temperature[sdaPin, sclPin, i2cAddress]
          //
          // (uint8_t) argv[2] is i2c address, 7 bytes size
          rc = getSHT2XMetric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : SHT2X_I2C_ADDRESS), SENS_READ_TEMP, payload);
          goto finish;
#endif // FEATURE_SHT2X_ENABLE  

#ifdef FEATURE_AT24CXX_ENABLE
        case CMD_AT24CXX_WRITE:
          //
          // AT24CXX.write[sdaPin, sclPin, i2cAddress, cellAddress, data]
          //
          // i is half length of decoded byte array
          //i = (strlen(optarg[4]) - 2) / 2;
          if (I2C_NO_ADDR_SPECIFIED == i2CAddress) {
            goto finish;
          }
          i = hstoba((uint8_t*) payload, optarg[4]);
          if (0 < i) {
            if (AT24CXXWrite(&SoftTWI, i2CAddress, argv[3], i, (uint8_t*) payload)) {
              rc =  RESULT_IS_OK;
            }
          }
          goto finish;

        case CMD_AT24CXX_READ:
          //
          // AT24CXX.read[sdaPin, sclPin, i2cAddress, cellAddress, length]
          //
          if (AT24CXXRead(&SoftTWI, i2CAddress, argv[3], argv[4], (uint8_t*) payload)) {
            // need to use payload as uint8_t, because sometime autocast is fail and user can get wrong data
            uint8_t* payloadPtr;
            payloadPtr = (uint8_t*) payload;
            _netClient.print("0x");
            DTSL( Serial.print("0x"); )
            for (i = 0; i < argv[4]; i++) {
              if (0x10 > payloadPtr[i]) {
                _netClient.print("0");
                DTSL( Serial.print("0"); )
              }
              _netClient.print(payloadPtr[i], HEX);
              DTSL( Serial.print(payloadPtr[i], HEX); )
            }
            rc = RESULT_IS_PRINTED;
            payloadPtr[0] = 0x00;
          } else {
            rc = RESULT_IS_FAIL;
          }
          goto finish;

#endif // FEATURE_AT24CXX_ENABLE

#ifdef FEATURE_MAX44009_ENABLE
        case CMD_MAX44009_LIGHT:
          //
          //  MAX44009.light[sdaPin, sclPin, i2cAddress, mode, integrationTime]
          //
          // 0x08 == MAX44009_INTEGRATION_TIME_6MS+1 - put it to integrationTime parameter if no arg#4 given
          // to kick getMAX44009Metric() to use auto measurement time
          rc = getMAX44009Metric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : MAX44009_I2C_ADDRESS), argv[3], (('\0' == *optarg[4]) ? 0x08 : argv[4]), SENS_READ_LUX, payload);
          goto finish;
#endif // FEATURE_MAX44009_ENABLE

#ifdef FEATURE_VEML6070_ENABLE
        case CMD_VEML6070_UV:
          //
          //  VEML6070.uv[sdaPin, sclPin, integrationTime]
          //  VEML6070.uv[18, 19, 3]
          // 0x01 == VEML6070 integrationTime default value - put it to integrationTime parameter if no arg#3 given
          // argv[4] & 0x03 operation need to take right integrationTime - 0x00...0x03
          rc = getVEML6070Metric(&SoftTWI, (('\0' == *optarg[2]) ? 0x01 : argv[2] & 0x03), SENS_READ_UV, payload);
          goto finish;
#endif // FEATURE_VEML6070_ENABLE

#ifdef FEATURE_PCA9685_ENABLE
        case CMD_PCA9685_WRITE:
          //
          //  PCA9685.write[sdaPin, sclPin, i2cAddress, outputIdx, onTime, offTime]
          //  PCA9685.write[18, 19, 0x40,, 4096, 0]
          rc = writePCA9685(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : PCA9685_I2C_ADDRESS), (('\0' == *optarg[3]) ? PCA9685_CHANNEL_LEDS_ALL : argv[3]), argv[4], argv[5]);
          goto finish;
#endif // FEATURE_PCA9685_ENABLE

#ifdef FEATURE_TSL2561_ENABLE
        case CMD_TSL2561_LIGHT:
          //
          //  TSL2561.light[sdaPin, sclPin, i2cAddress, integrationTime, gain]
          //  TSL2561.light[18, 19, 0x39, 402, 1]
          rc = getTSL2561Metric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : TSL2561_I2C_ADDRESS), argv[3], argv[4], SENS_READ_LUX, payload);
          goto finish;
#endif // FEATURE_TSL2561_ENABLE


#ifdef FEATURE_ADPS9960_ENABLE
        // !!! IR Led not used for ALS conversion
        // RGBC results can be used as light levels in Lux
        case CMD_ADPS9960_AMBIENT:
          //
          //  ADPS9960.ambient[sdaPin, sclPin, i2cAddress, integrationTime, gain]
          //  ADPS9960.ambient[18, 19, 0x39, 103, 4]
          rc = getADPS9960Metric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : ADPS9960_I2C_ADDRESS), (('\0' == *optarg[3]) ? APDS9960_DEFAULT_ADC_INTEGRATION_TIME : argv[3]), (('\0' == *optarg[4]) ? APDS9960_DEFAULT_ALS_GAIN : argv[4]), APDS9960_DEFAULT_LED_DRIVE, SENS_READ_LIGHT_AMBIENT, payload);
          goto finish;

        case CMD_ADPS9960_RED:
          //
          //  ADPS9960.red[sdaPin, sclPin, i2cAddress, integrationTime, gain]
          //  ADPS9960.red[18, 19, 0x39, 103, 4]
          rc = getADPS9960Metric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : ADPS9960_I2C_ADDRESS), (('\0' == *optarg[3]) ? APDS9960_DEFAULT_ADC_INTEGRATION_TIME : argv[3]), (('\0' == *optarg[4]) ? APDS9960_DEFAULT_ALS_GAIN : argv[4]), APDS9960_DEFAULT_LED_DRIVE, SENS_READ_LIGHT_RED, payload);
          goto finish;

        case CMD_ADPS9960_GREEN:
          //
          //  ADPS9960.green[sdaPin, sclPin, i2cAddress, integrationTime, gain]
          //  ADPS9960.green[18, 19, 0x39, 103, 4]
          rc = getADPS9960Metric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : ADPS9960_I2C_ADDRESS), (('\0' == *optarg[3]) ? APDS9960_DEFAULT_ADC_INTEGRATION_TIME : argv[3]), (('\0' == *optarg[4]) ? APDS9960_DEFAULT_ALS_GAIN : argv[4]), APDS9960_DEFAULT_LED_DRIVE, SENS_READ_LIGHT_GREEN, payload);
          goto finish;

        case CMD_ADPS9960_BLUE:
          //
          //  ADPS9960.blue[sdaPin, sclPin, i2cAddress, integrationTime, gain]
          //  ADPS9960.blue[18, 19, 0x39, 103, 4]
          rc = getADPS9960Metric(&SoftTWI, ((I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : ADPS9960_I2C_ADDRESS), (('\0' == *optarg[3]) ? APDS9960_DEFAULT_ADC_INTEGRATION_TIME : argv[3]), (('\0' == *optarg[4]) ? APDS9960_DEFAULT_ALS_GAIN : argv[4]), APDS9960_DEFAULT_LED_DRIVE, SENS_READ_LIGHT_BLUE, payload);
          goto finish;
#endif // FEATURE_ADPS9960_ENABLE


#ifdef FEATURE_MLX90614_ENABLE
        case CMD_MLX90614_TEMPERATURE:
          //
          //  mlx90614.temperature[sdaPin, sclPin, i2cAddress, zone]
          //  mlx90614.temperature[18, 19, 0x5A, 1]
          rc = getMLX90614Metric(&SoftTWI, (I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : MLX90614_I2C_ADDR, (('\0' == *optarg[3]) ? MLX90614_TEMPERATURE_ZONE_01 : argv[3]), SENS_READ_TEMP, payload);
          //rc = getMLX90614Metric(&SoftTWI, i2CAddress, argv[3], SENS_READ_TEMP, payload);
          goto finish;
#endif

#ifdef FEATURE_SGP30_ENABLE
        case CMD_SGP30_CO2E:
          //
          //  sgp30.co2e[sdaPin, sclPin, i2cAddress]
          // sgp30.co2e[18, 19, 0x58]
          rc = getSGP30Metric(SoftTWI, (I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : SGP30_I2C_ADDRESS, SENS_READ_CO2E, false, payload);
          goto finish;

        case CMD_SGP30_TVOC:
          //
          //  sgp30.tvoc[sdaPin, sclPin, i2cAddress]
          rc = getSGP30Metric(SoftTWI, (I2C_NO_ADDR_SPECIFIED != i2CAddress) ? i2CAddress : SGP30_I2C_ADDRESS, SENS_READ_TVOC, false, payload);
          goto finish;
#endif



      }  // switch (cmdIdx), I2C block
#endif // TWI_USE

#ifdef FEATURE_MODBUS_RTU_ENABLE
    case CMD_MODBUS_RTU_COILSREAD:
      //  Modbus.RTU.CoilsRead[rxPin, txPin, enablePin, slaveAddr, startAddr, ]
      // CMD_MODBUS_RTU_INPUTSREAD
      // CMD_MODBUS_RTU_COILSWRITE
      //rc = getModbusRTUMetric();
      goto finish;
#endif // FEATURE_MODBUS_RTU_ENABLE

  } // switch (cmdIdx) .. default in "commands with options" block

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

  if (RESULT_IS_PRINTED != rc) {
    switch (rc) {
      case RESULT_IS_BUFFERED:
        break;
      case RESULT_IS_OK:
      case RESULT_IS_SYSTEM_REBOOT_ACTION:
        //  '1' must be returned
        payload[0] = '1';
        payload[1] = '\0';
        break;
      case RESULT_IS_FAIL:
        // or '0'
        payload[0] = '0';
        payload[1] = '\0';
        break;
      case RESULT_IS_SIGNED_VALUE:
        //  or rc value placed in 'value' variable and must be converted to C-string.
        ltoa((int32_t) value, payload, 10);
        break;
      case RESULT_IS_UNSIGNED_VALUE:
        //  or rc value placed in 'value' variable and must be converted to C-string.
        ultoa((uint32_t) value, payload, 10);
        break;
      case DEVICE_ERROR_CONNECT:
        strcpy_P(payload, PSTR((MSG_DEVICE_ERROR_CONNECT)));
        break;
      case DEVICE_ERROR_ACK_L:
        strcpy_P(payload, PSTR((MSG_DEVICE_ERROR_ACK_L)));
        break;
      case DEVICE_ERROR_ACK_H:
        strcpy_P(payload, PSTR((MSG_DEVICE_ERROR_ACK_H)));
        break;
      case DEVICE_ERROR_CHECKSUM:
        strcpy_P(payload, PSTR((MSG_DEVICE_ERROR_CHECKSUM)));
        break;
      case DEVICE_ERROR_TIMEOUT:
        strcpy_P(payload, PSTR((MSG_DEVICE_ERROR_TIMEOUT)));
        break;
      case DEVICE_ERROR_WRONG_ID:
        strcpy_P(payload, PSTR((MSG_DEVICE_ERROR_WRONG_ID)));
        break;
      case DEVICE_ERROR_NOT_SUPPORTED:
        strcpy_P(payload, PSTR((MSG_DEVICE_ERROR_NOT_SUPPORTED)));
        break;
      case DEVICE_ERROR_WRONG_ANSWER:
        strcpy_P(payload, PSTR((MSG_DEVICE_ERROR_WRONG_ANSWER)));
        break;
      case DEVICE_ERROR_EEPROM_CORRUPTED:
        strcpy_P(payload, PSTR((MSG_DEVICE_ERROR_EEPROM)));
        break;
      case ZBX_NOTSUPPORTED:
        strcpy_P(payload, PSTR((MSG_ZBX_NOTSUPPORTED)));
        break;
      default:
        // otherwise subroutine return unexpected value, need to check its source code
        strcpy_P(payload, PSTR("Unexpected retcode"));
        break;
    }
  }

  /*
                        Push Zabbix packet header if incoming packet have Zabbix type
  */
  payloadLenght = strlen(payload);
  if (PACKET_TYPE_ZABBIX == _packetInfo.type) {
    // Calculate payload length to
    if (RESULT_IS_BUFFERED == rc) {
      switch (cmdIdx) {
#ifdef FEATURE_I2C_ENABLE
        case CMD_I2C_SCAN:
          // 'value' contained addreses number that scanI2C() return, 0x05 -> "0x" + two chars of string representation of HEX number + '\n'
          payloadLenght = value * 0x05;
          break;
#endif
#ifdef FEATURE_OW_ENABLE
        case CMD_OW_SCAN:
          // 'value' contained addresses number that scanOneWire() return, 0x13 (19) -> "0x" + eight*two chars of string representation of HEX number + '\n'
          payloadLenght = value * 0x13;
          break;
#endif
        default:
          break;
      }
    }
    // Fill reply packet header
    // Copy prefix to header (uppercase only!)
    memcpy(_dst, "ZBXD\1", ZBX_HEADER_PREFIX_LENGTH);
    // Copy lenght of answer string to header
    memcpy(&_dst[ZBX_HEADER_PREFIX_LENGTH], &payloadLenght, sizeof(payloadLenght));
    DTSD (
      Serial.println(F("--hdr--"));
    for (i = 0; i < ZBX_HEADER_LENGTH; i++) {
    Serial.print(_dst[i], HEX);
      Serial.print(" '");
      Serial.print((char) _dst[i]);
      Serial.println("' ");
    }
    Serial.println(F("--pld--"));
    )
    if (_netClient.connected()) {
      _netClient.write(_dst, ZBX_HEADER_LENGTH);
    }
  }

  /*
                        Push reply data to the client
  */
  DTSL( Serial.print(F("Reply: ")); )
  switch (cmdIdx) {
#ifdef FEATURE_OW_ENABLE
    // Special output format for OW.Scan[] command: 0x01020304\n0x05060708\n...
    case CMD_OW_SCAN:
      if (_netClient.connected()) {
        printArray((uint8_t*) payload, OW_ADDRESS_LENGTH * value, _netClient, OW_ADDRESS);
      }
      DTSL ( printArray((uint8_t*) payload, OW_ADDRESS_LENGTH * value, Serial, OW_ADDRESS); )
      break;
#endif
#ifdef FEATURE_I2C_ENABLE
    // Special output format for I2C.Scan[] command: 0x01\n0x02\n0x03...
    case CMD_I2C_SCAN: {
        if (_netClient.connected()) {
          printArray((uint8_t*) payload, I2C_ADDRESS_LENGTH * value, _netClient, I2C_ADDRESS);
        }
        DTSL ( printArray((uint8_t*) payload, I2C_ADDRESS_LENGTH * value, Serial, I2C_ADDRESS); )
      }
      break;
#endif
    // Plain text output for other
    default:
      if (_netClient.connected()) {
        //Network.client.print(payload);
        //DTSL( Serial.print("payloadLenght:"); Serial.println(payloadLenght); )

        _netClient.write(payload, payloadLenght);
        if (PACKET_TYPE_ZABBIX != _packetInfo.type) {
          _netClient.print('\n');
        }
      }
      DTSL( Serial.println(payload); )
      break;
  }
  //}


  /*
                Process post-action after sending reply to server
  */
  //
  switch (rc) {
    case RESULT_IS_SYSTEM_REBOOT_ACTION:
      _netClient.stop();
      delay(100);
      // The reason why using the watchdog timer or RST_SWRST_bm is preferable over jumping to the reset vector, is that when the watchdog or RST_SWRST_bm resets the AVR,
      // the registers will be reset to their known, default settings. Whereas jumping to the reset vector will leave the registers in their previous state, which is
      // generally not a good idea. http://www.atmel.com/webdoc/avrlibcreferencemanual/FAQ_1faq_softreset.html
      //
      //  ...but some Arduino's bootloaders going to "crazy loopboot" when WTD is enable on reset
      //
      // Watchdog deactivation
      __WATCHDOG( wdt_disable(); )
      asm volatile ("jmp 0");
      break;
  }

  DTSL( Serial.println(); )
  return cmdIdx;
}

