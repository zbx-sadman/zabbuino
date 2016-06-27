// My Freeduino is not listed, but is analogue to ARDUINO_AVR_DUEMILANOVE
#define ARDUINO_AVR_DUEMILANOVE
// Just for compilation with various default network configs
//#define USE_NETWORK_192_168_0_1

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                             !!! WizNet W5xxx users !!!

    1. Comment #include <UIPEthernet.h>
    2. Uncomment #include <Ethernet.h> and <SPI.h> headers
*/
#include <Ethernet.h>
#include <SPI.h>

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                !!! ENC28J60 users !!!

    0. Please try to use https://github.com/ntruchsess/arduino_uip/tree/fix_errata12 brahch of UIPEthernet if yours ENC28J60 freeze or loose connection.
    
    1. Comment #include <Ethernet.h> and <SPI.h> headers
    2. Uncomment #include <UIPEthernet.h> 
    
    Tested on UIPEthernet v1.09
    
    When UIPEthernet's fix_errata12 brahch did not help to add stability, you can buy W5100 shield.
    Also u can try uncomment USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE declaration (then ENC will be periodically re-intit if EIR.TXERIF and EIR.RXERIF is 1), 
    but you eed to do one change in UIPEthernet\utility\Enc28J60Network.h :
         private:
            ...    
            static uint8_t readReg(uint8_t address);  // << move its to __public__ section
            ...
             
         public: 
             ...
    
*/
//#include <UIPEthernet.h>
//#define USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE


#include "zabbuino.h"

// Wire lib for I2C sensors
#include <Wire.h>
// OneWire lib for Dallas sensors
#include <OneWire.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
// used by interrupts-related macroses
#include <wiring_private.h>

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 PROGRAMM FEATURES SECTION
                   (Please refer to the zabbuino.h file for more Zabbuino tuning like set State LED pin, network addresses, agent hostname and so)

        if connected sensors seems not work - first check setting in port_protect[], port_mode[], port_pullup[] arrays in I/O PORTS SETTING SECTION

*/

/****       Network      ****/

// Uncomment to use DHCP address obtaining
//#define FEATURE_NET_DHCP_ENABLE

// Uncomment to force using DHCP even netConfig.useDHCP = false
//#define FEATURE_NET_DHCP_FORCE

/****       Arduino      ****/

// Uncomment to enable Arduino's tone[], noTone[] commands
//#define FEATURE_TONE_ENABLE

// Uncomment to enable Arduino's randomSeed, random[] commands
//#define FEATURE_RANDOM_ENABLE

// Uncomment to enable shiftOut[] command
//#define FEATURE_SHIFTOUT_ENABLE

/****      1-Wire bus      ****/

// Uncomment to enable 1-Wire functions
#define FEATURE_OW_ENABLE

// Uncomment to enable Dallas DS18x20 family functions: DS18x20.*[] commands
#define FEATURE_DS18X20_ENABLE

/****        I2C bus       ****/

// Note #1: I2C library (Wire.h) takes at least 32bytes of memory for internal buffers
// Note #2: I2C library (Wire.h) activate internal pullups for SDA & SCL pins when Wire.begin() called

// Uncomment to enable I2C functions
//#define FEATURE_I2C_ENABLE

// Uncomment to enable BMP pressure sensors functions: BMP.*[] commands
//#define FEATURE_BMP085_ENABLE

// Uncomment to enable BH1750 light sensors functions: BH1750.*[] commands
//#define FEATURE_BH1750_ENABLE

/****        MicroWire bus       ****/

// Uncomment to enable MAX7219 8x8 led matrix functions: MAX7219.*[] commands
#define FEATURE_MAX7219_ENABLE

/****    DHT/AM family    ****/

// Uncomment to enable DHT/AM humidity sensors functions: DHT.*[] commands
//#define FEATURE_DHT_ENABLE


/****      System        ****/

// Uncomment to enable AVR watchdog
//                                                                     !!! BEWARE !!!
//                                                     NOT ALL BOOTLOADERS HANDLE WATCHDOG PROPERLY 
//                                                    http://forum.arduino.cc/index.php?topic=157406.0 
// 
// Note: OptiBoot is watchdog compatible and use less flash space that stock bootloader.
// Note: watchdog timeout may be vary for many controllers, see comments to macro WTD_TIMEOUT in zabbuino.h
//#define FEATURE_WATCHDOG_ENABLE

// Uncomment to be able to store runtime settings in EEPROM and use its on start
#define FEATURE_EEPROM_ENABLE

// Uncomment to force protect (enable even netConfig.useProtection is true) your system from illegal access for change runtime settings and reboots 
//#define FEATURE_PASSWORD_PROTECTION_FORCE

// Uncomment to enable system's command which can be used in system debug process: cmdCount, sys.ramFree and so
#define FEATURE_DEBUG_COMMANDS_ENABLE

// Uncomment to view the debug messages on the Serial Monitor
//#define FEATURE_DEBUG_TO_SERIAL

// Uncomment to enable using time+interrupt for internal metric gathering
#define GATHER_METRIC_USING_TIMER_INTERRUPT

// Uncomment to enable external interrupts handling: interrupt.*[] commands
//#define FEATURE_EXTERNAL_INTERRUPT_ENABLE

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 GLOBAL VARIABLES SECTION
*/

netconfig_t netConfig;
#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
// need to #include <wiring_private.h> for compilation
volatile extInterrupt_t extInterrupt[EXTERNAL_NUM_INTERRUPTS];
#endif

EthernetServer ethServer(10050);
EthernetClient ethClient;

char cBuffer[BUFFER_SIZE];
int16_t argOffset[ARGS_MAX];
int32_t sysMetrics[IDX_METRICS_MAX];

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                      STARTUP SECTION
*/

// Enable I2C functions if user forget it
#if defined(FEATURE_BH1750_ENABLE) || defined(FEATURE_BMP085_ENABLE)
#define FEATURE_I2C_ENABLE
#endif

// Enable 1-Wire functions if user forget it
#if defined(FEATURE_DS18X20_ENABLE)
#define FEATURE_OW_ENABLE
#endif


void setup() {
  uint8_t i;

#ifdef ADVANCED_BLINKING
  // blink on start
  blinkMore(6, 50, 500);
#endif

  // Init metrics
  sysMetrics[IDX_METRIC_SYS_VCCMIN] = sysMetrics[IDX_METRIC_SYS_VCCMAX] = MeasureVoltage(ANALOG_CHAN_VBG);
  sysMetrics[IDX_METRIC_SYS_RAM_FREE] = sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] = (int32_t) ramFree();
  sysMetrics[IDX_METRIC_SYS_CMD_COUNT] = sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = 0;
  
#ifdef FEATURE_DEBUG_TO_SERIAL
  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
#endif

#ifdef FEATURE_EEPROM_ENABLE

/* -=-=-=-=-=-=-=-=-=-=-=-
    FACTORY RESET BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */

  // Set mode of PIN_FACTORY_RESET and turn on internal pull resistor
  pinMode(PIN_FACTORY_RESET, INPUT_PULLUP);
//  digitalWrite(PIN_FACTORY_RESET, HIGH);
  // Check for PIN_FACTORY_RESET shoring to ground?
  // (when pulled INPUT pin shorted to GND - digitalRead() return LOW)
  if (LOW == digitalRead(PIN_FACTORY_RESET)){
    // Fire up state LED
    digitalWrite(PIN_STATE_LED, HIGH);
    // Wait some msecs
    delay(HOLD_TIME_TO_FACTORY_RESET);
    // PIN_FACTORY_RESET still shorted?
    if (LOW == digitalRead(PIN_FACTORY_RESET)){
       setDefaults(netConfig);
       saveConfig(&netConfig);
    }
  } // if (LOW == digitalRead(PIN_FACTORY_RESET))

/* -=-=-=-=-=-=-=-=-=-=-=-
    CONFIGURATION LOAD BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */
#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrintln_P(PSTR("Load configuration from EEPROM"));
#endif
  // Try to load configuration from EEPROM
  if (false == loadConfig(&netConfig)) {
     // bad CRC detected, use default values for this run
     setDefaults(netConfig);
     saveConfig(&netConfig);
  }
#else // FEATURE_EEPROM_ENABLE
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrintln_P(PSTR("Use default network settings"));
#endif
     // Use hardcoded values if EEPROM feature disabled
     setDefaults(netConfig);
#endif // FEATURE_EEPROM_ENABLE

#ifdef FEATURE_NET_DHCP_FORCE
     netConfig.useDHCP =true;
#endif


/* -=-=-=-=-=-=-=-=-=-=-=-
    NETWORK START BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */
#ifdef FEATURE_NET_DHCP_ENABLE
  // User want to use DHCP with Zabbuino?
  if (true == netConfig.useDHCP) {
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrintln_P(PSTR("Obtaining address from DHCP..."));
#endif
       // Try to ask DHCP server
      if (0 == Ethernet.begin(netConfig.macAddress)) {
#ifdef FEATURE_DEBUG_TO_SERIAL
         SerialPrintln_P(PSTR("No successfully"));
#endif
         // No offer recieved - switch off DHCP feature for that session
         netConfig.useDHCP = false;
      }
  }
#else // FEATURE_NET_DHCP_ENABLE
  netConfig.useDHCP=false;
#endif // FEATURE_NET_DHCP_ENABLE

  // No DHCP offer recieved or no DHCP need - start with stored/default IP config
  if (false == netConfig.useDHCP) {
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrintln_P(PSTR("Use static IP"));
#endif
     // That overloaded .begin() function return nothing
     Ethernet.begin(netConfig.macAddress, netConfig.ipAddress, netConfig.ipNetmask, netConfig.ipGateway);
  }
  
#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrintln_P(PSTR("Begin internetworking"));
  SerialPrint_P(PSTR("MAC     : ")); printArray(netConfig.macAddress, sizeof(netConfig.macAddress), DBG_PRINT_AS_MAC);
  SerialPrint_P(PSTR("Hostname: ")); Serial.println(netConfig.hostname);
  SerialPrint_P(PSTR("IP      : ")); Serial.println(Ethernet.localIP());
  SerialPrint_P(PSTR("Subnet  : ")); Serial.println(Ethernet.subnetMask());
  SerialPrint_P(PSTR("Gateway : ")); Serial.println(Ethernet.gatewayIP());
  SerialPrint_P(PSTR("Password: ")); Serial.println(netConfig.password);
  // Block is compiled if UIPethernet.h is included
#ifdef UIPETHERNET_H
  SerialPrint_P(PSTR("ENC28J60: rev ")); Serial.println(Enc28J60.getrev());
#endif

  //netConfig.ipGateway.printTo(Serial);
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

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
  // Init external interrupts info structure
  for (i = 0; i < EXTERNAL_NUM_INTERRUPTS; i++) { 
    // -1 - interrupt is detached
    // just use NOT_AN_INTERRUPT = -1 macro from Arduino.h
    extInterrupt[i].mode = NOT_AN_INTERRUPT;
  }
#endif

  // Uncomment to force protect (enable even useProtection is false) your system from illegal access for change runtime settings and reboots 
#ifdef FEATURE_PASSWORD_PROTECTION_FORCE
  netConfig.useProtection = true;
#endif

#ifdef FEATURE_WATCHDOG_ENABLE
  // Watchdog activation
  wdt_enable(WTD_TIMEOUT);
#endif

#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
   // need to analyze return code?
   timerOneInit(SYS_METRIC_RENEW_PERIOD);
#endif

#ifdef FEATURE_I2C_ENABLE
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
  uint32_t nowTime, processStartTime, processEndTime;
  uint32_t prevDHCPRenewTime, prevENCReInitTime, prevNetProblemTime, prevSysMetricGatherTime;
  uint8_t errorCode, blinkType = (uint8_t) BLINK_NOPE;
    
  // Correcting timestamps

  prevDHCPRenewTime = prevENCReInitTime = prevNetProblemTime = prevSysMetricGatherTime = millis();

  // if no exist while() here - netProblemTime must be global or static - its will be 0 every loop() and time-related cases will be processeed abnormally
  // ...and while save some cpu ticks because do not call everytime from "hidden main()" subroutine, and do not init var, and so.
  while (true) { 
    nowTime = millis();

#ifndef GATHER_METRIC_USING_TIMER_INTERRUPT
    // Gather internal metrics periodically
    // may be do it with interrupt?
    if (SYS_METRIC_RENEW_PERIOD <= (uint32_t) (nowTime - prevSysMetricGatherTime)) { gatherMetrics(); prevSysMetricGatherTime = nowTime; }
#endif 

#ifdef FEATURE_NET_DHCP_ENABLE
    // DHCP used in this session and time to renew lease?
    if (true == netConfig.useDHCP && (NET_DHCP_RENEW_PERIOD <= (uint32_t) (nowTime - prevDHCPRenewTime))) {
       // Ethernet library's manual say that Ethernet.maintain() can be called every loop for DHCP renew, but i won't do this so often
       errorCode = Ethernet.maintain();
       // Renew procedure finished with success
       if (DHCP_CHECK_RENEW_OK == errorCode || DHCP_CHECK_REBIND_OK  == errorCode) { 
          // No alarm blink  need, network activity registred, renewal period restarted
          blinkType = (uint8_t) BLINK_NOPE;
          prevDHCPRenewTime = prevNetProblemTime = nowTime;
       } else {
          // Got some errors - blink with "DHCP problem message"
          blinkType = (uint8_t) BLINK_DHCP_PROBLEM;
#ifdef FEATURE_DEBUG_TO_SERIAL
//            SerialPrintln_P(PSTR("DHCP renew problem occured"));
#endif 
       }
    }
#endif // FEATURE_NET_DHCP_ENABLE

    // No DHCP problem found but no data recieved or network activity for a long time
    if (BLINK_NOPE == blinkType && (NET_IDLE_TIMEOUT <= (uint32_t) (nowTime - prevNetProblemTime))) { 
#ifdef FEATURE_DEBUG_TO_SERIAL
//            SerialPrintln_P(PSTR("No data recieved for a long time"));
#endif 
       blinkType =(uint8_t) BLINK_NET_PROBLEM; 
    }

#ifdef USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE
    // Time to reinit ENC28J60?
    if (NET_ENC28J60_REINIT_PERIOD <= (uint32_t) (nowTime - prevENCReInitTime)) {
       // if EIR.TXERIF or EIR.RXERIF is set - ENC28J60 detect error, re-init module 
       if (Enc28J60.readReg(EIR) & (EIR_TXERIF | EIR_RXERIF)) {
#ifdef FEATURE_DEBUG_TO_SERIAL
          SerialPrintln_P(PSTR("ENC28J60 reinit"));
#endif
          Enc28J60.init(netConfig.macAddress); 
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

    // Test state of active session: client still connected or unread data is exist in buffer?
    if (ethClient.connected()) {
       // A lot of chars wait for reading
       if (ethClient.available()) {
          // Do not need next char to analyze - EOL detected or there no room in buffer or max number or args parsed...   
          if (false == analyzeStream(ethClient.read())) {
             /*****  processing command *****/
             // Destroy unused client's data 
             ethClient.flush(); 
             // Fire up State led, than will be turned off on next loop
             digitalWrite(PIN_STATE_LED, HIGH);
             //
             // may be need test for client.connected()? 
             processStartTime = millis();
             executeCommand();
             processEndTime = millis();
             // use processEndTime as processDurationTime
             processEndTime = (processStartTime <= processEndTime) ? (processEndTime - processStartTime) : (4294967295UL - processStartTime + processEndTime);
             sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = max(sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX], processEndTime);

             // Wait some time to finishing answer send
             delay(NET_STABILIZATION_DELAY);
             // close connection           
             ethClient.stop(); 
          }
          // Restart network activity control cycle
          prevENCReInitTime = prevNetProblemTime = millis();
          blinkType = (uint8_t) BLINK_NOPE;
       }
    } else {
       // Active session is not exist. Try to take new for processing.
       ethClient = ethServer.available();
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
*  Detect Zabbix packets, on-fly spit incoming stream to command, arguments
*
**************************************************************************************************************************** */
uint8_t analyzeStream(char charFromClient) {
  uint8_t static needSkipZabbix2Header, argIndex;
  uint16_t static bufferWritePosition;

  // If there is not room in buffer - simulate EOL recieving
  if (BUFFER_SIZE <= bufferWritePosition ) { charFromClient = '\n'; }
  // Put next char to buffer
  cBuffer[bufferWritePosition] = tolower(charFromClient); 
 
  // When ZBX_HEADER_PREFIX_LENGTH chars is saved to buffer - test its for Zabbix2 protocol header prefix ("ZBXD\01") presence
  if (ZBX_HEADER_PREFIX_LENGTH == bufferWritePosition) {
     if (0 == memcmp(&cBuffer, ZBX_HEADER_PREFIX, ZBX_HEADER_PREFIX_LENGTH)) {
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
     switch (charFromClient) {
        case ']':
        case 0x20:
          // Space or final square bracket found. Do nothing and next char will be written to same position. 
          // Return 'Need next char'
          return true;
        case '[':
        case ',':
          // Delimiter or separator found. Push begin of next argument (bufferWritePosition+1) on buffer to arguments offset array. 
          argOffset[argIndex] = bufferWritePosition+1; argIndex++; 
          // Make current buffer segment like C-string
          cBuffer[bufferWritePosition] = '\0'; break;
        case '\n':
          // EOL detected
          // Save last argIndex that pointed to <null> item. All unused argOffset[] items must be pointed to this <null> item too.
          cBuffer[bufferWritePosition] = '\0'; 
          while (ARGS_MAX > argIndex) { argOffset[argIndex++] = bufferWritePosition;}
          // increase argIndex++ to pass (ARGS_MAX < argIndex) condition 
          argIndex++; break;
      }
      // EOL reached or there is not room to store args. Stop stream analyzing and do command executing
      if (ARGS_MAX < argIndex) {
         // clear vars for next round
         bufferWritePosition = argIndex = 0;
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
*  Command execution subroutine
*  
**************************************************************************************************************************** */
void executeCommand()
{
  uint8_t AccessGranted, i;
  int32_t result = RESULT_IS_FAIL;
  // duration  in tone[] - ulong
  uint32_t arg[ARGS_MAX];
  int16_t cmdIdx = -1;
  
  sysMetrics[IDX_METRIC_SYS_CMD_COUNT]++;

  for (i = 0; i < CMD_MAX; i++)
  {
     if (0 == strcmp_P(cBuffer, (char*)pgm_read_word(&(commands[i])))) {cmdIdx = i; break;}
  }

#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrint_P(PSTR("Execute command #")); Serial.print(cmdIdx); SerialPrint_P(PSTR(" =>")); Serial.println(cBuffer);
#endif 
  
  // batch convert args to number values
  for (i = 0; i < ARGS_MAX; i++)
  {
     arg[i] = ('\0' == cBuffer[argOffset[i]]) ? 0 : strtoul(&cBuffer[argOffset[i]], NULL,0);

#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrint_P(PSTR("arg[")); Serial.print(i); SerialPrint_P(PSTR("] => \"")); 
     if ('\0' == cBuffer[argOffset[i]]) {
        SerialPrint_P(PSTR("<null>")); 
     } else {
        Serial.print(&cBuffer[argOffset[i]]); 
     }
     SerialPrint_P(PSTR("\" => ")); Serial.print(arg[i]);
     SerialPrint_P(PSTR(", offset =")); Serial.println(argOffset[i]);
#endif 
  }

  
  // Check rights for password protected commands
  if (!netConfig.useProtection) {
     AccessGranted = true;
  } else if (arg[0] == netConfig.password) {
     AccessGranted = true;
  } else {
     AccessGranted = false;
  }

  switch (cmdIdx) {
//     case -1: 
//        break;
    case CMD_ZBX_AGENT_PING:
      // Команда: agent.ping
      // Параметры: не требуются
      // Результат: возвращается значение '1'
      result = RESULT_IS_OK;
      break;
    case CMD_ZBX_AGENT_HOSTNAME:
      // Команда: agent.hostname
      // Параметры: не требуются
      // Результат: возвращается имя узла
      // strcpy_P(cBuffer, PSTR(ZBX_HOSTNAME));
      strcpy(cBuffer, netConfig.hostname);
      result = RESULT_IN_BUFFER;
      break;
         
    case CMD_ZBX_AGENT_VERSION:
      // Команда: agent.version
      // Параметры: не требуются
      // Результат: возвращается версия агента
      strcpy_P(cBuffer, PSTR(ZBX_AGENT_VERISON));
      result = RESULT_IN_BUFFER;
      break;

#ifdef FEATURE_DEBUG_COMMANDS_ENABLE

    case CMD_SYS_UPTIME:
      // Команда: agent.uptime
      // Параметры: не требуются
      // Результат: возвращается количество секунд, прошедших с момента включения
      result = millis() / 1000;
      break;
   
    case CMD_SYS_CMD_COUNT:
      // Команда: agent.cmdCount
      // Параметры: не требуются
      // Результат: возвращается количество обработанных команд
      if (arg[0]) { sysMetrics[IDX_METRIC_SYS_CMD_COUNT] = 0; } 
      result = sysMetrics[IDX_METRIC_SYS_CMD_COUNT];
      break;

    case CMD_SYS_CMD_TIMEMAX:
      // Команда: 
      // Параметры: не требуются.
      // Результат:
      if (arg[0]) { sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX] = 0; } 
      result = sysMetrics[IDX_METRIC_SYS_CMD_TIMEMAX];
      break;
   
    case CMD_SYS_RAM_FREE:
      // Команда: sys.ramFree
      // Параметры: не требуются.
      // Результат: возвращается объем свободной оперативной памяти контроллера.
      result = sysMetrics[IDX_METRIC_SYS_RAM_FREE];
      break;

    case CMD_SYS_RAM_FREEMIN:
      // Команда: sys.memmin
      // Параметры: не требуются.
      // Результат: возвращается зафиксированный в процессе периодических измерений минимальный объем свободной оперативной памяти контроллера.
      result = sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN];
      break;
   
    case CMD_SYS_MCU_NAME:
      // Команда: sys.cpuName
      // Параметры: не требуются.
      // Результат: возвращается мнемоническое имя микроконтроллера
      strcpy_P(cBuffer, PSTR(_AVR_CPU_NAME_));
      result = RESULT_IN_BUFFER;
      break;
   
    case CMD_SYS_NET_MODULE:
      // Команда: sys.netmodule
      // Параметры: не требуются.
      // Результат: возвращается мнемоническое имя network modile
      strcpy_P(cBuffer, PSTR(NET_MODULE_NAME));
      result = RESULT_IN_BUFFER;
      break;
#endif

#ifdef FEATURE_EEPROM_ENABLE
    case CMD_SET_HOSTNAME:
      // Команда: sethostname[password, hostname]
      // Параметры: password - пароль, используемый для изменения свойств системы,
      //            hostname - новое имя узла.
      // Результат: изменяется имя узла при условии совпадения параметра password с системным пароля и заполненения параметра hostname, происходит возврат значения '1'. 
      //            В противном случае возвращается значение '0';
      // Примечание: Проверка пароля происходит только при установленном в значение 1 конфигурационного параметра netConfig.useProtection (см команду setprotection[]).
      if (AccessGranted) {
         // need check for arg existsience?
         // cBuffer[argOffset[1]] != \0 if argument #2 given
         if (cBuffer[argOffset[1]]) {
            sethostname(netConfig.hostname, &cBuffer[argOffset[1]]);
            // strncpy(netConfig.hostname, &cBuffer[argOffset[1]], ZBX_AGENT_HOSTNAME_MAXLEN-1);
            // netConfig.hostname[ZBX_AGENT_HOSTNAME_MAXLEN]='\0';
            //        Serial.println(netConfig.hostname);
            saveConfig(&netConfig);
            result = RESULT_IS_OK;
         }
      }
      break;
  
    case CMD_SET_PASSWORD:
      // Команда: setpassword[oldPassword, newPassword]
      // Параметры: oldPassword - пароль, используемый для изменения свойств системы,
      //            newPassword - вновьустанавливаемый пароль.
      // Результат: изменяется пароль при условии совпадения параметра oldPassword с системным пароля и заполненения параметра newPassword, производится запись в EEPROM,
      //            происходит возврат значения '1'. В противном случае возвращается значение '0'. 
      // Примечание: Проверка пароля происходит только при установленном в значение 1 конфигурационного параметра netConfig.useProtection (см команду setprotection[]).
      if (AccessGranted) {
         if (cBuffer[argOffset[1]]) {
            // take new password from argument #2
            netConfig.password = arg[1];
            saveConfig(&netConfig);
            result = RESULT_IS_OK;
         }
      }
      break;
  
    case CMD_SET_SYSPROTECT:
      // Команда: setprotection[password, protection]
      // Параметры: password - пароль, используемый для изменения свойств системы,
      //            protection - флаг установки защиты паролем: 1 - защита включена, любое иное - защита отменена, 
      // Результат: изменяется значение конфигурационного параметра netConfig.useProtection, , производится запись в EEPROM, происходит возврат значения '1'. 
      //            В случае неудачи возвращается значение '0'.
      // Примечание: Проверка пароля происходит только при установленном в значение 1 конфигурационного параметра netConfig.useProtection (см команду setprotection[]).
      if (AccessGranted) {
         if (cBuffer[argOffset[1]]) {
            // take new password from argument #2
            netConfig.useProtection = (1 == arg[1]) ? true : false;
            saveConfig(&netConfig);
            result = RESULT_IS_OK;
         }
      }
      break;
   
    case CMD_SET_NETWORK:
      // Команда: setnetwork[password, useDHCP, macAddress, ipAddress, ipNetmask, ipGateway]
      // Параметры: password - пароль, используемый для изменения свойств системы,
      //            useDHCP - 1 enable, 0 - disable
      //            macAddress - новый MAC-адрес, задается в шестнадцатеричной форме: 0xAABBCCDDEEFF
      //            ipAddress  - новый IP-адрес, задается в шестнадцатеричной форме: http://www.miniwebtool.com/ip-address-to-hex-converter/
      //            ipNetmask, - новая сетевая маска, задается в шестнадцатеричной форме: -"-"-
      //            ipGateway  - новый адрес шлюза по умолчанию, задается в шестнадцатеричной форме: -"-"-
      // Результат: изменяются сетевые настройки, производится запись в EEPROM, происходит возврат значения '1'. 
      //            В случае неудачи возвращается значение '0'.
      // Примечание: Проверка пароля происходит только при установленном в значение 1 конфигурационного параметра netConfig.useProtection (см команду setprotection[]).
  
      if (AccessGranted) {
         uint8_t ip[4], mac[6], success = 1;
         // useDHCP flag coming from first argument and must be numeric (boolean) - 1 or 0, 
         // arg[0] data contain in cBuffer[argOffset[1]] placed from argOffset[0]
         netConfig.useDHCP = (uint8_t) arg[1];
         // ip, netmask and gateway have one structure - 4 byte
         // take 6 bytes from second argument of command and use as new MAC-address
         // if convertation is failed (return false) succes variable must be falsed too via logic & operator
         success &= hstoba((uint8_t*) mac, &cBuffer[argOffset[2]], sizeof(netConfig.macAddress));
         memcpy(&netConfig.macAddress, &mac, sizeof(netConfig.macAddress));
      
         // use 4 bytes from third argument of command as new IP-address. sizeof(IPAddress) returns 6 instead 4
         success &= hstoba((uint8_t*) &ip, &cBuffer[argOffset[3]], 4);
         netConfig.ipAddress = IPAddress(ip);
  
         // take 4 bytes from third argument of command an use as new IP Netmask
         success &= hstoba((uint8_t*) &ip, &cBuffer[argOffset[4]], 4);
         netConfig.ipNetmask = IPAddress(ip);
  
         // convert 4 bytes from fourth argument to default gateway
         success &= hstoba((uint8_t*) &ip, &cBuffer[argOffset[5]], 4);
         netConfig.ipGateway = IPAddress(ip);
  
         // netConfig.ipGateway.printTo(Serial);
         // Save config to EEProm if success
         if (success) {
            saveConfig(&netConfig);
            result = RESULT_IS_OK;
         }
       }
       break;
  
#endif // FEATURE_EEPROM_ENABLE

    case CMD_SYS_REBOOT:
      // Команда: reboot[password]
      // Параметры: password - пароль, используемый для изменения свойств системы
      // Результат: система начинает выполнять программу заново, с адреса 0 (мягкая перезагрузка), происходит возврат значения '1'. 
      //            В случае неудачи возвращается значение '0'.
      // Примечание: Проверка пароля происходит только при установленном в значение 1 конфигурационного параметра netConfig.useProtection (см команду setprotection[]).
      if (AccessGranted) {
         ethClient.println("1");
         // hang-up if no delay
         delay(NET_STABILIZATION_DELAY);
         ethClient.stop();
#ifdef FEATURE_WATCHDOG_ENABLE
         // Watchdog deactivation
         wdt_disable();
#endif
         asm volatile ("jmp 0");  
      }
      break;

    case CMD_SYS_VCC:
      // Команда: sys.vcc
      // Параметры: не требуются
      // Результат: производится "замер" напряжения на входе VCC микроконтроллера, его значение в mV возвращается пользователю. 
      // Take VCC
      result = MeasureVoltage(ANALOG_CHAN_VBG);
      // VCC may be bigger than max or smaller than min. 
      // To avoid wrong results and graphs in monitoring system - correct min/max metrics
      correctVCCMetrics(result);
       break;
  
    case CMD_SYS_VCCMIN:
      // Команда: sys.minvcc
      // Параметры: не требуются
      // Результат: пользователю возвращается значение минимального значения напряжения в mV на входе VCC микроконтроллера с момента подачи питания.
      result = sysMetrics[IDX_METRIC_SYS_VCCMIN];
       break;
  
    case CMD_SYS_VCCMAX:
      // Команда: sys.maxvcc
      // Параметры: не требуются
      // Результат: пользователю возвращается значение максимального значения напряжения в mV на входе VCC микроконтроллера с момента подачи питания.
      result = sysMetrics[IDX_METRIC_SYS_VCCMAX];
       break;
  
    case CMD_ARDUINO_DELAY:
      // Команда: delay[value]
      // Параметры: value - время паузы перед выдачей ответа. Задается в миллисекундах.
      // Результат: Возврат значения '1' производится после истечения времени задержки.
      // Примечание: команда является оберткой функции Delay() http://www.arduino.cc/en/Reference/Delay
      delay(arg[0]);
      result = RESULT_IS_OK;
      break;
  
    case CMD_SYS_PORTWRITE:
      // Команда: portWrite[port, value]
      // Параметры: port - символьное обозначение порта (B,C,D..),
      //            value - значение, которое требуется записать в заданный порт ввода/вывода
      // Результат: изменяется состояние порта ввода/вывода (PORTB, PORTC, PORTD...) и происходит возврат значения '1'.
      // Примечание: если ваш экземпляр Arduino имеет более, чем три порта, то на данный момент вам необходимо самостоятельно добавить в скетч информацию о них.
      //
      // Номер порта представляет собой разницу между ASCII-кодом аргумента port и 96. Таким образом b=2, c=3, d=4 и т.д.
      portWrite((byte) arg[0] - 96, arg[1]);
      result = RESULT_IS_OK;
      break;
  
    case CMD_ARDUINO_ANALOGWRITE:
      // Команда: analogWrite[pin, value]
      // Параметры: pin - цифровое обозначение пина, value - значение скважности, которое требуется задать для данного пина.
      // Результат: изменяется скважности PWM для пина. Удачное выполнение команды влечет за собой возврат значения '1', неудачное - значения '0'.
      // Примечание: команда является оберткой функции analogWrite() http://www.arduino.cc/en/Reference/AnalogWrite
      // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит. Если пин не является PWM-совместимым, на нем выставляется значение HIGH.
      // Внимание! Функция analogWrite() самостоятельно устанавливает пин в режим работы OUTPUT
      if (isSafePin(arg[0])) {
        analogWrite(arg[0], arg[1]);
        result = RESULT_IS_OK;
      }
      break;
  
    case CMD_ARDUINO_ANALOGREAD:
      // Команда: analogread[pin]
      // Параметры: pin - цифровое обозначение пина
      // Результат: возврат величины, "считанной" с пина. Диапазон значений 0...1023 (возможные варианты диапазона значений зависят от способа подключения сигнала к пину и внутренних настроек Arduino)
      // Примечание: команда является оберткой функции analogRead() www.arduino.cc/en/Reference/AnalogRead
      // Данная команда имеет смысл только для аналоговых пинов.
      // Состояние INPUT для пина должно быть определено в коде скетча. В противном случае совпадения считываемых данных с ожидаемыми может не произойти.   
      result = (long) analogRead(arg[0]);
      break;
      
  
    case CMD_ARDUINO_ANALOGREFERENCE:
      // Команда: analogReference[source]
      // Параметры: source - источник опорного напряжения (0..N). Значения можно найти в заголовочном файле Arduino.h
      // Результат: устанавливается источник опорного напряжения относительно которого происходят аналоговые измерения и происходит возврат значения '1'
      // Примечание: команда является оберткой функции analogReference() www.arduino.cc/en/Reference/AnalogReference
      analogReference(arg[0]);
      result = RESULT_IS_OK;
      break;
  
    case CMD_ARDUINO_DIGITALWRITE:
      // Команда: digitalWrite[pin, value]
      // Параметры: pin - цифровое обозначение пина, value - значение, которое требуется выставить на заданном пине.
      // Результат: изменяется состояние пина. Удачное выполнение команды влечет за собой возврат значения '1', неудачное - значения '0'.
      // Примечание: команда является оберткой функции digitalWrite() www.arduino.cc/en/Reference/DigitalWrite
      // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит.    
      if (isSafePin(arg[0])) {
        digitalWrite(arg[0], arg[1]);
        result = RESULT_IS_OK;
      }
      break;
      
    case CMD_ARDUINO_DIGITALREAD:
      // Команда: digitalRead[pin]
      // Параметры: pin - цифровое обозначение пина
      // Результат: возвращается значение, "считанное" с пина. Диапазон значений - HIGH/LOW.
      // Примечание: команда является оберткой функции DigitalRead() http://www.arduino.cc/en/Reference/DigitalRead
      // Состояние INPUT для пина должно быть определено в коде скетча. В противном случае совпадения считываемых данных с ожидаемыми может не произойти.
      result = (long) digitalRead(arg[0]);
       break;

#ifdef FEATURE_TONE_ENABLE
    case CMD_ARDUINO_TONE:
      // Команда: tone[pin, frequency, duration]
      // Параметры: pin - цифровое обозначение пина, frequency - частота сигнала, duration - длительность сигнала
      // Результат: начинается генерация на указанном пине сигнала "прямоугольная волна" заданной частоты.
      // Примечание: команда является оберткой функции tone() http://www.arduino.cc/en/Reference/Tone
      // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит.
      if (isSafePin(arg[0])) {
        tone(arg[0], arg[1], arg[2]);
        result = RESULT_IS_OK;
      }
      break;
  
    case CMD_ARDUINO_NOTONE:
      // Команда: noTone[pin]
      // Параметры: pin - цифровое обозначение пина
      // Результат: завершается генерация на указанном пине сигнала "прямоугольная волна"
      // Примечание: команда является оберткой функции noTone() http://www.arduino.cc/en/Reference/NoTone
      // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит.
      if (isSafePin(arg[0])) {
        noTone(arg[0]);
        result = RESULT_IS_OK;
      }
      break;
  
#endif

#ifdef FEATURE_SHIFTOUT_ENABLE
    case CMD_SYS_SHIFTOUT:
      // Команда: shiftOut[dataPin, clockPin, latchPin, bitOrder, value]
      // Параметры: dataPin, clockPin, latchPin - цифровое обозначения пинов вывода данных, синхронизации, защелкивания.
      //            bitOrder - последовательность вывода бит, value значение для вывода.
      // Результат: устанавливается соответствующее параметру value состояние выводов подключенного сдвигового регистра.
      //            Удачное выполнение команды влечет за собой возврат значения `1`, неудачное - значения '0'.
      // Примечание: команда является расширением функции shiftOut().
      //            Параметр value может быть задан как в десятичной и шестнадцатеричной форме (с префиксом 0x).
      //            Длина числа в шестнадцатеричной форме ограничена размером внутреннего буфера.
      // Состояние OUTPUT для пинов должно быть задано в коде скетча. Если пины защищены, вызова соотвествующих функций не происходит.
      // Для защелкивания сдвигового регистра перед использованием команды shiftout значение пина latch должно быть определено.
      // В противном случае защелкивания сдвигового регистра не производится.

      // i used as latchPinDefined
      i = ('\0' != arg[2]) && isSafePin(arg[2]);   // << корректный способ проверки или нет?  
      if (isSafePin(arg[0]) &&  isSafePin(arg[1])) {
         if (i) { digitalWrite(arg[2], LOW); }
         advShiftOut(arg[0], arg[1], arg[3], &cBuffer[argOffset[4]]);
         if (i) { digitalWrite(arg[2], HIGH);}
         result = RESULT_IS_OK;
      }
      break;
#endif

#ifdef FEATURE_RANDOM_ENABLE
    case CMD_ARDUINO_RANDOMSEED:
      // Команда: randomSeed[value]
      // Параметры: value - начальное число ряда псевдослучайных значений
      // Результат: инициализируется генератор псевдослучайных чисел
      // Примечание: команда является оберткой функции randomSeed() http://www.arduino.cc/en/Reference/randomSeed
      randomSeed(arg[0]);
      result = RESULT_IS_OK;
      break;
   
    case CMD_ARDUINO_RANDOM:
      // Команда: random[min, max]
      // Параметры: min, max - нижняя и верхняя границы псевдослучайных значений
      // Результат: возвращается псевдослучайное число
      // Примечание: команда является оберткой функции random() http://www.arduino.cc/en/Reference/random
      //  !! random return long
      result = (long) random(arg[0], arg[1]);
      break;
#endif // FEATURE_RANDOM_ENABLE


#ifdef FEATURE_OW_ENABLE
    case CMD_OW_SCAN:
      // Команда: OW.Scan[pin]
      // Параметры: pin - цифровое обозначение пина, к которому подключена шина 1-Wire. 
      // Результат: производится поиск всех устройств 1-Wire, список их идентификаторов в шестнадцатеричном виде возвращается пользователю. 
      //            При отсутствии результатов поиска - возвращается '0';
      if (isSafePin(arg[0])) {
         result = oneWireScan(arg[0]);
      }
      break;


#ifdef FEATURE_DS18X20_ENABLE
    case CMD_DS18X20_TEMPERATURE:
      // Команда: DS18x20.Temperature[pin, resolution, id]
      // Параметры: pin - цифровое обозначение пина, к которому подключен цифровой термометр DS18x20. resolution - разрешение термометра 9..12бит,
      //            id - идентификатор (адрес) термометра.
      // Результат: с цифрового термометра считывается температура и значение в градусах Цельсия возвращается пользователю.
      // Примечание: Точность показаний (1/2 ... 1/16 C) зависит от параметра resolution, от него, также зависит время выполнения команды.
      //             Максимальный временной промежуток - 825ms (resolution = 12bit).
      //             Идентификатор (адрес) термометра можно получить через Serial Monitor при выполнении скетча DallasTemperature -> Single.
      //             Значение -127 выдается при какой-либо ошибке в функции - невозможности считать данные с термометра вследствии ошибки подсоединения или ошибочно указанном ID.
      //             Так же это значение выдается при попытке обращения к термометру неподдерживаемой модели.
      if (isSafePin(arg[0])) {
         result = DS18X20Read(arg[0], arg[1], &cBuffer[argOffset[2]], cBuffer);
      }
      break;
#endif // FEATURE_DS18X20_ENABLE
#endif // FEATURE_ONEWIRE_ENABLE

#ifdef FEATURE_DHT_ENABLE
    case CMD_DHT_TEMPERATURE:
      // Команда: DHT.Temperature[pin, model]
      // Параметры: pin - цифровое обозначение пина, к которому подключен цифровой датчик DHT/AM/..
      //            model - идентификатор модели датчика - 11 (DHT11), 21 (DHT21, AM2301), 22 (DHT22, AM2302).
      // Результат: с цифрового датчика DHT11/21/22 считывается температура и значение в градусах Цельсия возвращается пользователю.
      // Примечание: Значение -127 выдается при какой-либо ошибке - например несовпадении CRC.
      //             Если модель датчика не указана или указана неверно, то расчет температуры производится по формуле, применяемой для DHT22.
      //             Команда самостоятельно устанавливает состояние INPUT/OUTPUT пина. В целях безопасности стоит инициализировать пин как OUTPUT.
      if (isSafePin(arg[0])) {
        result = DHTRead(arg[0], arg[1], SENS_READ_TEMP, cBuffer);
      }
      break;
   
    case CMD_DHT_HUMIDITY:
      // Команда: DHT.Humidity[pin, model]
      // Параметры: pin - цифровое обозначение пина, к которому подключен цифровой датчик DHT/AM/..
      //            model - идентификатор модели датчика - 11 (DHT11), 21 (DHT21, AM2301), 22 (DHT22, AM2302).
      // Результат: с цифрового датчика считывается величина влажности и значение в процентах возвращается пользователю.
      // Примечание: Значение -127 выдается при какой-либо ошибке - например несовпадении CRC.
      //             Если модель датчика не указана или указана неверно, то расчет величины влажности производится по формуле, применяемой для DHT22.
      //             Команда самостоятельно устанавливает состояние INPUT/OUTPUT пина. В целях безопасности стоит инициализировать пин как OUTPUT.
      if (isSafePin(arg[0])) {
        result = DHTRead(arg[0], arg[1], SENS_READ_HUMD, cBuffer);
      }
      break;
#endif // FEATURE_DHT_ENABLE

#ifdef FEATURE_I2C_ENABLE
    case CMD_I2C_SCAN:
      // Команда: I2C.Scan[sdaPin, sclPin]
      // Параметры: sdaPin, sclPin - цифровые обозначение пинов, к которым подключена шина I2C.
      // Результат: производится поиск всех устройств на шине I2C, список ихадресов в шестнадцатеричном виде возвращается пользователю. 
      //            При отсутствии результатов поиска - возвращается '0';
      // Примечание: sdaPin, sclPin на данный момент не применяются (используются стандартные пины для I2C подключения) и зарезервированы для внедрения SoftTWI интерфейсов.
      if (isSafePin(arg[0]) && isSafePin(arg[1])) {
         // (int8_t) arg[2] is i2c address, 7 bytes size
         result = i2CScan();
      }
      break;
       
#ifdef FEATURE_BMP085_ENABLE
    case CMD_BMP_TEMPERATURE:
      // Команда: BMP.Temperature[sdaPin, sclPin, i2cAddress]
      // Параметры: sdaPin, sclPin - цифровые обозначение пинов, к которым подключена шина I2C.
      //            i2cAddress - адрес датчика на шине I2C (адрес по умолчанию: 0x77) 
      // Результат: с цифрового датчика BMP085/BMP180 считывается температура и значение в градусах цельсия возвращается пользователю.
      // Примечание: Точность датчика - 0,1C.
      //             sdaPin, sclPin на данный момент не применяются (используются стандартные пины для I2C подключения) и зарезервированы для внедрения SoftTWI интерфейсов.
      if (isSafePin(arg[0]) && isSafePin(arg[1])) {
         // (int8_t) arg[2] is i2c address, 7 bytes size
         result = BMP085Read(arg[0], arg[1], arg[2], arg[3], SENS_READ_TEMP, cBuffer);
      }
      break;
  
    case CMD_BMP_PRESSURE:
      // Команда: BMP.Pressure[sdaPin, sclPin, i2cAddress, overSampling]
      // Параметры: sdaPin, sclPin - цифровые обозначение пинов, к которым подключена шина I2C.
      //            i2cAddress - адрес датчика на шине I2C (адрес по умолчанию: 0x77) 
      //            overSampling - значение, определяющее точность и длительность измерения.
      //            0 - ultra low power, RMS noise = 6Pa, conversion time = 4,5ms ... 3 - ultra high resolution, RMS noise = 3Pa, conversion time = 25,5ms
      // Результат: с цифрового датчика считывается величина атмосферного давления и значение в Паскалях возвращается пользователю.
      // Примечание: sdaPin, sclPin на данный момент не применяются (используются стандартные пины для I2C подключения) и зарезервированы для внедрения SoftTWI интерфейсов.
      if (isSafePin(arg[0]) && isSafePin(arg[1])) {
          // (int8_t) arg[2] is i2c address, 7 bytes size
         result = BMP085Read(arg[0], arg[1], arg[2], arg[3], SENS_READ_PRSS, cBuffer);
      }
      break;
  
#endif // FEATURE_BMP085_ENABLE

#ifdef FEATURE_BH1750_ENABLE
    case CMD_BH1750_LIGHT:
      // Команда: BH1750.light[sdaPin, sclPin, i2cAddress, mode]
      // Параметры: sdaPin, sclPin - цифровые обозначение пинов, к которым подключена шина I2C.
      //            i2cAddress - адрес датчика на шине I2C (адрес по умолчанию: 0x23) 
      //            mode:  32 - (0x20) BH1750_ONE_TIME_HIGH_RES_MODE 
      //                   33 - (0x21) BH1750_ONE_TIME_HIGH_RES_MODE_2
      //                   35 - (0x23) BH1750_ONE_TIME_LOW_RES_MODE
      // Рекомендация от производителя: применяйте измерение в режиме высокого разрешения, так как в этом режиме отсекаются помехи (включая шум на частоте 50Hz/60Hz).
      // Режимы высокого разрешения могут быть применены для определения темноты (освещенность менее 10 Lx).
      // Примечание: sdaPin, sclPin на данный момент не применяются (используются стандартные пины для I2C подключения) и зарезервированы для внедрения SoftTWI интерфейсов.
      if (isSafePin(arg[0]) && isSafePin(arg[1])) {
         // (int8_t) arg[2] is i2c address, 7 bytes size
         result = BH1750Read(arg[0], arg[1], (int8_t) arg[2], arg[3], SENS_READ_LUX, cBuffer);
      }
      break;
#endif // FEATURE_BH1750_ENABLE
#endif // FEATURE_I2C_ENABLE


#ifdef FEATURE_MAX7219_ENABLE
    case CMD_MAX7219_WRITE:
      // Команда: MAX7219.write[dataPin, clockPin, loadPin, intensity, value]
      // Параметры: dataPin, clockPin, loadPin - цифровое обозначения пинов вывода данных, синхронизации, загрузки.
      //            intensity - яркость свечения элементов индикатора (0..15), value значение для вывода.
      // Результат: устанавливается соответствующее параметру value состояние элементов индикатора, подключенного к МС MAX7219. 
      //            Удачное выполнение команды влечет за собой возврат значения `1`, неудачное - значения '0'.
      // Примечание: на текущий момент команда оптимизирована для применения со светодиодной матрицей 8x8, подключенной к МС MAX7219.
      //            Команда не производит очистку не затронутых значением value элементов индикатора.
      //            Параметр value должен быть задан в шестнадцатеричной форме (с префиксом 0x), его длина ограничена размером внутреннего буфера.
      // Cтроки матрицы заполняются последовательно, используя значения из value (например: 0x6666001818817E00) следующим образом: 
      //            Строка 1  =>  0x66  =>  B01100110  =>  - + + - - + + -
      //            Строка 2  =>  0x66  =>  B01100110  =>  - + + - - + + - 
      //            Строка 3  =>  0x00  =>  B00000000  =>  - - - - - - - -
      //            Строка 4  =>  0x18  =>  B00011000  =>  - - - + + - - -
      //            Строка 5  =>  0x18  =>  B00011000  =>  - - - + + - - -
      //            Строка 6  =>  0x81  =>  B10000001  =>  + - - - - - - +
      //            Строка 7  =>  0x7E  =>  B01111110  =>  - + + + + + + -
      //            Строка 8  =>  0x00  =>  B00000000  =>  - - - - - - - -
      // Состояние OUTPUT для пинов должно быть задано в коде скетча. Если пины защищены, вызова соотвествующих функций не происходит.
      if (isSafePin(arg[0]) && isSafePin(arg[1])  && isSafePin(arg[2])) {
         max7219DrawOn8x8(arg[0], arg[1], arg[2], arg[3], &cBuffer[argOffset[4]]);
         result = RESULT_IS_OK;
      }
      break;
#endif // FEATURE_MAX7219_ENABLE

      // Команда: shiftOut[dataPin, clockPin, latchPin, bitOrder, value]
      // Параметры: dataPin, clockPin, latchPin - цифровое обозначения пинов вывода данных, синхронизации, защелкивания.
      //            bitOrder - последовательность вывода бит, value значение для вывода.
      // Примечание: команда является расширением функции shiftOut().
      //            Параметр value может быть задан как в десятичной и шестнадцатеричной форме (с префиксом 0x).
      //            Длина числа в шестнадцатеричной форме ограничена размером внутреннего буфера.
      // Состояние OUTPUT для пинов должно быть задано в коде скетча. Если пины защищены, вызова соотвествующих функций не происходит.
      // Для защелкивания сдвигового регистра перед использованием команды shiftout значение пина latch должно быть определено.
      // В противном случае защелкивания сдвигового регистра не производится.

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
    case CMD_EXTINT_COUNT:
      // Команда: interrupt.count[intPin, intNumber, mode]
      // Параметры: intPin - цифровые обозначение пинов, на которое назначено (или будет назначено) внешнее прерывание.
      //            intNumber - номер прерывания. Номера прерываний зависят от используемой платформы, обратитесь к ее описанию для уточнения.
      //            mode - режим прерывания: 1 - CHANGE, 2 - FALLING, 3 - RISING
      // Примечание: intNumber на данный момент не применяется (используются стандартные прерывания, привязанные к intPin) и зарезервирован для будущих разработок.

      // TODO: maybe need to rework code block
      if (isSafePin(arg[0])) {
         int8_t interruptNumber=digitalPinToInterrupt(arg[0]);
         voidFuncPtr interruptHandler;
         // Interrupt number and mode is correct?
         if ((EXTERNAL_NUM_INTERRUPTS > interruptNumber) && (RISING >= arg[3])) {
            // Interrupt mode is changed
            // Serial.println("[1] Interrupt number and mode is correct"); 
            // just use NOT_AN_INTERRUPT = -1 macro from Arduino.h
            if (extInterrupt[interruptNumber].mode != arg[3] && NOT_AN_INTERRUPT != extInterrupt[interruptNumber].mode) {
               // Serial.println("[2] Interrupt mode is changed, detach"); 
               detachInterrupt(arg[1]);
               extInterrupt[interruptNumber].mode = -1;
            } 

           // Interrupt not attached?
           if (NOT_AN_INTERRUPT == extInterrupt[arg[1]].mode) {
              extInterrupt[interruptNumber].mode = arg[3];
              switch (interruptNumber) {
// Basic configuration => EXTERNAL_NUM_INTERRUPTS == 3
                case INT0:
                  interruptHandler = handleINT0;
                  break;
                case INT1:
                  interruptHandler = handleINT1;
                  break;
// AVR_ATmega1284, AVR_ATmega1284P, AVR_ATmega644, AVR_ATmega644A, AVR_ATmega644P, AVR_ATmega644PA => EXTERNAL_NUM_INTERRUPTS == 3
#if (EXTERNAL_NUM_INTERRUPTS > 2)
                case INT2:
                  interruptHandler = handleINT2;
                  break;
#endif // EXTERNAL_NUM_INTERRUPTS > 2
// AVR_ATmega32U4 => EXTERNAL_NUM_INTERRUPTS == 5
#if (EXTERNAL_NUM_INTERRUPTS > 3)
                case INT3:
                  interruptHandler = handleINT3;
                  break;
                case INT4:
                  interruptHandler = handleINT4;
                  break;
#endif // EXTERNAL_NUM_INTERRUPTS > 3
// AVR_ATmega1280, AVR_ATmega2560, AVR_ATmega128RFA1, AVR_ATmega256RFR2 => EXTERNAL_NUM_INTERRUPTS == 8
#if (EXTERNAL_NUM_INTERRUPTS > 5)
                case INT5:
                  interruptHandler = handleINT5;
                  break;
                case INT6:
                  interruptHandler = handleINT6;
                  break;
                case INT7:
                  interruptHandler = handleINT7;
                  break;
#endif // EXTERNAL_NUM_INTERRUPTS > 5
                default:
                // still not attached
                extInterrupt[interruptNumber].mode = NOT_AN_INTERRUPT;
              }  // switch (interruptNumber)
              // check again to take in account 'No interrupt choosed' case
              if (NOT_AN_INTERRUPT != extInterrupt[interruptNumber].mode) {
                 // if pin still not INPUT_PULLUP - system will hang up
                 pinMode(arg[0], INPUT_PULLUP);
                 attachInterrupt(interruptNumber, interruptHandler, arg[3]);
                 // reinit counter
                 extInterrupt[interruptNumber].count = 0;
              }
            
           } // if (NOT_AN_INTERRUPT == extInterrupt[arg[1]].mode)
           result = extInterrupt[interruptNumber].count;
         } // if ((interruptNumber < EXTERNAL_NUM_INTERRUPTS) && (CHANGE >= arg[3]))
       } // if (isSafePin(arg[0]))
       break;
#endif // FEATURE_EXTERNAL_INTERRUPT_ENABLE
     

    default:
      // В любом ином случае команда считается неопределенной.
      strcpy(cBuffer, ZBX_NOTSUPPORTED_MSG);
      // Прирощенный ранее счетчик сматывается
      sysMetrics[IDX_METRIC_SYS_CMD_COUNT]--;
      result = RESULT_IN_BUFFER;
   }


   // Результат уже выведен исполняемой командой?
   if (RESULT_IS_PRINTED != result) {
      // Результат помещен в буфер заранее?
      if (RESULT_IN_BUFFER != result) {
         //  Необходимо возвратить '1'
         if (RESULT_IS_OK == result) {
            result = 1L;
         // или '0'
         } else if (RESULT_IS_FAIL == result) {
            result = 0L;
         }
         //  Если результатом работы команды является число, оно преобразуется в C-string.
         ltoa (result, cBuffer, 10);
      }
      //  Буфер отдается клиенту
      ethClient.println(cBuffer);
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrint_P(PSTR("Result: ")); Serial.println(cBuffer); Serial.println(); 
#endif
   }
}



