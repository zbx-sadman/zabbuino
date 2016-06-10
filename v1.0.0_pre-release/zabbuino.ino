#include "zabbuino.h"
#include "avr_cpunames.h"
// "platforms.h" must be included after "zabbuino.h"
#include "platforms.h"

// Wire lib for I2C sensors
#include <Wire.h>
// OneWire lib for Dallas sensors
#include <OneWire.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                             !!! WizNet W5xxx users !!!

    1. Comment #include <UIPEthernet.h>
    2. Comment #define __ETH_ENC28J60__
    3. Uncomment #include <Ethernet.h> and <SPI.h> headers
*/
#include <Ethernet.h>
#include <SPI.h>

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                !!! ENC28J60 users !!!

    1. Comment #include <Ethernet.h> and <SPI.h> headers
    2. Uncomment #include <UIPEthernet.h> 
    3. Uncomment #define __ETH_ENC28J60__ to use specific ENC28J60 functions 
    
    Tested on UIPEthernet v1.09
*/
//#include <UIPEthernet.h>
//#define __ETH_ENC28J60__


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 PROGRAMM FEATURES SECTION
                                                  (Please refer to the zabbuino.h file for more Zabbuino tuning)
*/

// Uncomment to enable AVR watchdog
// Note: not all bootloader works correctly with watchdog functions. OptiBoot is watchdog compatible.
// Note: watchdog timeout may be vary for many controllers, see comments to macro WTD_TIMEOUT in zabbuino.h
//#define FEATURE_WATCHDOG_ENABLE

// Uncomment to use DHCP address obtaining
// #define FEATURE_NET_DHCP_ENABLE
// Uncomment to force using DHCP even netConfig.useDHCP = false
// #define FEATURE_NET_DHCP_FORCE

// Uncomment to be able to store runtime settings in EEPROM and use its on start
#define FEATURE_EEPROM_ENABLE

// Uncomment to force protect (enable even netConfig.useProtection is true) your system from illegal access for change runtime settings and reboots 
//#define FEATURE_PASSWORD_PROTECTION_FORCE

// Uncomment to enable Arduino's tone[], noTone[] commands
#define FEATURE_TONE_ENABLE

// Uncomment to enable Arduino's randomSeed, random[] commands
#define FEATURE_RANDOM_ENABLE

// Uncomment to enable system's command which can be used in system debug process: cmdCount, sys.freeRAM and so
#define DEBUG_COMMANDS_ENABLE

// Uncomment to enable shiftOut[] command
#define FEATURE_SHIFTOUT_ENABLE

// Uncomment to enable Dallas DS18x20 family functions: DS18x20.*[] commands
#define FEATURE_DS18X20_ENABLE

// Uncomment to enable DHT/AM humidity sensors functions: DHT.*[] commands
#define FEATURE_DHT_ENABLE

// Uncomment to enable BMP pressure sensors functions: BMP.*[] commands
#define FEATURE_BMP085_ENABLE

// Uncomment to view the debug messages on the Serial Monitor
//#define FEATURE_DEBUG_TO_SERIAL

// Uncomment for get debug messages in TCP session - implementation not finished
//#define DEBUG_MSG_TO_ETHCLIENT


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 GLOBAL VARIABLES SECTION
*/

netconfig_t netConfig;
EthernetServer ethServer(10050);
EthernetClient ethClient;

char cBuffer[BUFFER_SIZE];
int argOffset[ARGS_NUM+1];
long sysMetrics[SYS_METRICS_NUMBER];

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                      STARTUP SECTION
*/

void setup() {
#ifdef FEATURE_DEBUG_TO_SERIAL
  Serial.begin(9600);
#endif

#ifdef FEATURE_EEPROM_ENABLE

/* -=-=-=-=-=-=-=-=-=-=-=-
    FACTORY RESET BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */

  // Set mode of PIN_FACTORY_RESET and turn on internal pull resistor
  pinMode(PIN_FACTORY_RESET, INPUT);
  digitalWrite(PIN_FACTORY_RESET, HIGH);
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
  // printArray(netConfig.macAddress,6,1);
  Serial.print("Hostname: "); Serial.println(netConfig.hostname);
  Serial.print("IP      : "); Serial.println(Ethernet.localIP());
  SerialPrint_P(PSTR("Password: ")); Serial.println(netConfig.password);
  //netConfig.ipGateway.printTo(Serial);
#endif

  // Start listen sockets
  ethServer.begin();

/* -=-=-=-=-=-=-=-=-=-=-=-
    OTHER STUFF INIT BLOCK
   -=-=-=-=-=-=-=-=-=-=-=- */

  // I/O ports initialization. Refer to "I/O PORTS SETTING SECTION" in zabbuino.h
  for (uint8_t i = 0; i < PORTS_NUM; i++) setPortMode(i, port_mode[i], port_pullup[i]);

  // Uncomment to force protect (enable even useProtection is false) your system from illegal access for change runtime settings and reboots 
#ifdef FEATURE_PASSWORD_PROTECTION_FORCE
  netConfig.useProtection = true;
#endif

#ifdef FEATURE_WATCHDOG_ENABLE
  // Watchdog activation
  wdt_enable(WTD_TIMEOUT);
#endif
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                      RUN SECTION
*/
void loop() {
  uint32_t nowTime, dhcpRenewTime, encReInitTime, netProblemTime, sysMetricGatherTime;
  uint8_t errorCode, blinkType = BLINK_NOPE;
  
  // Init metrics
  sysMetrics[SYS_METRIC_IDX_VCCMIN] = sysMetrics[SYS_METRIC_IDX_VCCMAX] = MeasureVoltage(ANALOG_CHAN_VBG);
  sysMetrics[SYS_METRC_IDX_CMDCOUNT] = 0;
  
  // Correcting timestamps
  dhcpRenewTime = encReInitTime = netProblemTime = sysMetricGatherTime = millis();
  dhcpRenewTime += NET_DHCP_RENEW_PERIOD;
  netProblemTime += NET_IDLE_TIMEOUT;
  sysMetricGatherTime += SYS_METRC_RENEW_PERIOD;

#if defined(__ETH_ENC28J60__)
  encReInitTime += NET_ENC28J60_REINIT_PERIOD;
#endif

  //!!!!!!!!!!!!!  need to check code for "mills() after 50 days" problem avoid
  // if no exist while() here - netProblemTime must be global or static - its will be 0 every loop() and time-related cases will be processeed abnormally
  // ...and while save some cpu ticks because do not call everytime from "hidden main()" subroutine, and do not init var, and so.
  while (true) { 
    nowTime = millis();

    // Gather internal metrics periodically
    // may be do it with interrupt?
    if (nowTime > sysMetricGatherTime) { gatherMetrics(); sysMetricGatherTime = nowTime + SYS_METRC_RENEW_PERIOD; }

#ifdef FEATURE_NET_DHCP_ENABLE
    // DHCP used in this session and time to renew lease?
    if (true == netConfig.useDHCP && nowTime > dhcpRenewTime) {
       // Ethernet library's manual say that Ethernet.maintain() can be called every loop for DHCP renew, but i won't do this so often
       errorCode = Ethernet.maintain();
       // Renew procedure finished with success
       if (DHCP_CHECK_RENEW_OK == errorCode || DHCP_CHECK_REBIND_OK  == errorCode) { 
          // No alarm blink  need, network activity registred, renewal period restarted
          blinkType = BLINK_NOPE;
          netProblemTime = nowTime + NET_IDLE_TIMEOUT; 
          dhcpRenewTime = nowTime + NET_DHCP_RENEW_PERIOD;
       } else {
          // Got some errors - blink with "DHCP problem message"
          blinkType = BLINK_DHCP_PROBLEM;
#ifdef FEATURE_DEBUG_TO_SERIAL
//            SerialPrintln_P(PSTR("DHCP renew problem occured"));
#endif 
       }
    }
#endif // FEATURE_NET_DHCP_ENABLE

    // No DHCP problem found but no data recieved or network activity for a long time
    if (BLINK_NOPE == blinkType && (nowTime > netProblemTime)) { 
#ifdef FEATURE_DEBUG_TO_SERIAL
//            SerialPrintln_P(PSTR("No data recieved for a long time"));
#endif 
       blinkType = BLINK_NET_PROBLEM; 
    }

#if defined(__ETH_ENC28J60__)
    // Time to reinit ENC28J60?
    if (nowTime > encReInitTime) {
       Enc28J60.init(netConfig.macAddress); 
       // Wait some time again before new reInit
       encReInitTime = nowTime + NET_ENC28J60_REINIT_PERIOD; 
    }
#endif

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
          analyzeStream(ethClient.read());
          // ethClient.flush(); ethClient.println(nowTime); ethClient.println(" - bye");  delay(100); ethClient.stop(); 
          // Restart network activity control cycle
          encReInitTime = netProblemTime = millis();
          encReInitTime +=  NET_ENC28J60_REINIT_PERIOD; 
          netProblemTime += NET_IDLE_TIMEOUT; 
          blinkType = BLINK_NOPE;
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
*  Функция анализа потока данных.
*  Помещает байты считанные из буфера Ethernet, в рабочий буфер, принимает решение о типе пакета,
*  инициирует выполнение принятой команды
*
**************************************************************************************************************************** */
void analyzeStream(char charFromClient) {
   uint8_t static needSkipZabbix2Header, argIndex;
   uint16_t static bufferWritePosition;
   uint8_t lastOffset;
   
   // Put next char to buffer
  cBuffer[bufferWritePosition] = tolower(charFromClient);

  // When ZBX_HEADER_PREFIX_LENGTH chars is saved to buffer - test its for Zabbix2 protocol header prefix ("ZBXD\01") presence
  if (ZBX_HEADER_PREFIX_LENGTH == bufferWritePosition) {
     if (memcmp(&cBuffer, "zbxd\01", ZBX_HEADER_PREFIX_LENGTH) == 0) {
        // If packet have prefix - set 'skip whole header' flag
        needSkipZabbix2Header = true;
     }
  }

  // When ZBX_HEADER_LENGTH chars is saved to buffer - ckeck 'skip whole header' flag
  if (ZBX_HEADER_LENGTH == bufferWritePosition && needSkipZabbix2Header) {
     // If is setted - just begin write new data from begin of buffer. It's operation 'drop' Zabbix2 header
     bufferWritePosition = 0;
     needSkipZabbix2Header = false;
     // 'return' here save a lot cpu time
     return;
  }

  // Process all chars if its not from header data
  if (!needSkipZabbix2Header) {
     switch (charFromClient) {
        case '[':
        case ',':
          // Delimiter or separator found. Push begin of next argument (bufferWritePosition+1) on buffer to arguments offset array. 
          argOffset[argIndex] = bufferWritePosition+1; argIndex++; 
          // Make current buffer segment like C-string
          cBuffer[bufferWritePosition] = '\0'; break;
        case '\n':
          // Don't increase argIndex. All unused argOffset[] items must be pointed to this <null> item.
          argOffset[argIndex] = bufferWritePosition; 
          cBuffer[bufferWritePosition] = '\0'; break;
        case ']':
        case 0x20:
          // Space or final square bracket found. Rewind buffer write position to one position
          bufferWritePosition--; break;
      }
      // EOL reached or there is not room to store args. Stop stream analyzing and do command executing
      if ( '\n' == charFromClient || ARGS_NUM < argIndex ) {
        // Where are last item (<nulled> \n )- ?
        lastOffset = argOffset[argIndex];
        // Point unused argOffsets[] to last item.
         while (ARGS_NUM > argIndex) { argOffset[argIndex] = lastOffset; argIndex++; }  
         // Destroy unused client's data 
         ethClient.flush(); 
         // Test for processing data availability
         if (0 < bufferWritePosition) {
            /*****  processing  *****/
            digitalWrite(PIN_STATE_LED, HIGH);
            executeCommand();
            // delay(100);
            // close connection           
            ethClient.stop(); 
            // clear vars for next round
            bufferWritePosition = argIndex = 0;
            needSkipZabbix2Header = false;
            return;
         }
       }             
  }
  // Until the buffer is filled completely - move write position to the tail
  if (BUFFER_SIZE > bufferWritePosition ) { bufferWritePosition++; } 

}

void executeCommand()
{
  uint8_t enableAction;
  uint32_t result = RESULT_IN_BUFFER;
  uint32_t arg[ARGS_NUM];
  
  sysMetrics[SYS_METRC_IDX_CMDCOUNT]++;

#ifdef FEATURE_DEBUG_TO_SERIAL
  SerialPrint_P(PSTR("Execute command: ")); Serial.println(cBuffer);
#endif 

  // batch convert args to number values
  for (byte i = 0; i < ARGS_NUM; i++)
  {
     arg[i] = ('\0' == cBuffer[argOffset[i]]) ? 0 : atol(&cBuffer[argOffset[i]]);
#ifdef FEATURE_DEBUG_TO_SERIAL
     SerialPrint_P(PSTR("arg[")); Serial.print(i); SerialPrint_P(PSTR("] => \"")); 
     if ('\0' == cBuffer[argOffset[i]]) {
        Serial.print("<null>"); 
     } else {
        Serial.print(&cBuffer[argOffset[i]]); 
     }
     SerialPrint_P(PSTR("\" => ")); Serial.print(arg[i]);
     SerialPrint_P(PSTR(", offset =")); Serial.println(argOffset[i]);
#endif 
  }
  // Check rights for password protected commands
  if (!netConfig.useProtection) {
     enableAction = true;
  } else if (arg[0] == netConfig.password) {
    enableAction = true;
  } else {
    enableAction = false;
  }

// !!!!!!!!!!!!!!! Проверить: portWrite, shiftout

  if (0 == strcmp_P(cBuffer, PSTR(CMD_ZBX_AGENT_PING))) {
    // Команда: agent.ping
    // Параметры: не требуются
    // Результат: возвращается значение '1'
    result = RESULT_IS_OK;

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ZBX_AGENT_HOSTNAME))) {
    // Команда: agent.hostname
    // Параметры: не требуются
    // Результат: возвращается имя узла
    // strcpy_P(cBuffer, PSTR(ZBX_HOSTNAME));
     strcpy(cBuffer, netConfig.hostname);

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ZBX_AGENT_VERSION))) {
    // Команда: agent.version
    // Параметры: не требуются
    // Результат: возвращается версия агента
    strcpy_P(cBuffer, PSTR(ZBX_AGENT_VERISON));

#ifdef DEBUG_COMMANDS_ENABLE
  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_CMDCOUNT))) {
    // Команда: agent.cmdCount
    // Параметры: не требуются
    // Результат: возвращается количество обработанных команд
    result = sysMetrics[SYS_METRC_IDX_CMDCOUNT];

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_FREERAM))) {
    // Команда: sys.freeRAM
    // Параметры: не требуются.
    // Результат: возвращается объем свободной оперативной памяти контроллера.
    result = (long) freeRam();

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_CPUNAME))) {
    // Команда: sys.cpuName
    // Параметры: не требуются.
    // Результат: возвращается мнемоническое имя микроконтроллера
    strcpy_P(cBuffer, PSTR(_AVR_CPU_NAME_));
#endif

#ifdef FEATURE_EEPROM_ENABLE
  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_SET_HOSTNAME))) {
    // Команда: sethostname[password, hostname]
    // Параметры: password - пароль, используемый для изменения свойств системы,
    //            hostname - новое имя узла.
    // Результат: изменяется имя узла при условии совпадения параметра password с системным пароля и заполненения параметра hostname, происходит возврат значения '1'. 
    //            В противном случае возвращается значение '0';
    // Примечание: Проверка пароля происходит только при установленном в значение 1 конфигурационного параметра netConfig.useProtection (см команду setprotection[]).
    result = RESULT_IS_FAIL;
    if (enableAction) {
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

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_SET_PASSWORD))) {
    // Команда: setpassword[oldPassword, newPassword]
    // Параметры: oldPassword - пароль, используемый для изменения свойств системы,
    //            newPassword - вновьустанавливаемый пароль.
    // Результат: изменяется пароль при условии совпадения параметра oldPassword с системным пароля и заполненения параметра newPassword, производится запись в EEPROM,
    //            происходит возврат значения '1'. В противном случае возвращается значение '0'. 
    // Примечание: Проверка пароля происходит только при установленном в значение 1 конфигурационного параметра netConfig.useProtection (см команду setprotection[]).
    result = RESULT_IS_FAIL;
    if (enableAction) {
       if (cBuffer[argOffset[1]]) {
          // take new password from argument #2
          netConfig.password = arg[1];
          saveConfig(&netConfig);
          result = RESULT_IS_OK;
       }
    }

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_SET_PROTECTION))) {
   // Команда: setprotection[password, protection]
   // Параметры: password - пароль, используемый для изменения свойств системы,
   //            protection - флаг установки защиты паролем: 1 - защита включена, любое иное - защита отменена, 
   // Результат: изменяется значение конфигурационного параметра netConfig.useProtection, , производится запись в EEPROM, происходит возврат значения '1'. 
   //            В случае неудачи возвращается значение '0'.
   // Примечание: Проверка пароля происходит только при установленном в значение 1 конфигурационного параметра netConfig.useProtection (см команду setprotection[]).
     result = RESULT_IS_FAIL;
    if (enableAction) {
       if (cBuffer[argOffset[1]]) {
          // take new password from argument #2
          netConfig.useProtection = (1 == arg[1]) ? true : false;
          saveConfig(&netConfig);
          result = RESULT_IS_OK;
       }
    }
 
  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_SET_NETWORK))) {
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
     result = RESULT_IS_FAIL;

     if (enableAction) {
       uint8_t ip[4], mac[6], success = 1;
        // useDHCP flag coming from first argument and must be numeric (boolean) - 1 or 0, 
        // arg[0] data contain in cBuffer[argOffset[1]] placed from argOffset[0]
        netConfig.useDHCP = (uint8_t) arg[1];
        // ip, netmask and gateway have one structure - 4 byte
        // take 6 bytes from second argument of command and use as new MAC-address
        // if convertation is failed (return false) succes variable must be falsed too via logic & operator
        success &= hstoba((uint8_t*) mac, &cBuffer[argOffset[2]], 6);
        memcpy(&netConfig.macAddress, &mac, 6);
    
        // use 4 bytes from third argument of command as new IP-address
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
#endif // FEATURE_EEPROM_ENABLE

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_REBOOT))) {
     // Команда: reboot[password]
     // Параметры: password - пароль, используемый для изменения свойств системы
     // Результат: система начинает выполнять программу заново, с адреса 0 (мягкая перезагрузка), происходит возврат значения '1'. 
     //            В случае неудачи возвращается значение '0'.
     // Примечание: Проверка пароля происходит только при установленном в значение 1 конфигурационного параметра netConfig.useProtection (см команду setprotection[]).
     result = RESULT_IS_FAIL;
     if (enableAction) {
        ethClient.println("1");
        // hang-up if no delay
        delay(100);
        ethClient.stop();
#ifdef FEATURE_WATCHDOG_ENABLE
        // Watchdog deactivation
        wdt_disable();
#endif
        asm volatile ("jmp 0");  
     }

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_GET_VCC))) {
    // Команда: sys.vcc
    // Параметры: не требуются
    // Результат: производится "замер" напряжения на входе VCC микроконтроллера, его значение в mV возвращается пользователю. 
    // Take VCC
    result = MeasureVoltage(ANALOG_CHAN_VBG);
    // VCC may be bigger than max or smaller than min. 
    // To avoid wrong results and graphs in monitoring system - correct min/max metrics
    correctVCCMetrics(result);

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_GET_MINVCC))) {
    // Команда: sys.minvcc
    // Параметры: не требуются
    // Результат: пользователю возвращается значение минимального значения напряжения в mV на входе VCC микроконтроллера с момента подачи питания.
    result = sysMetrics[SYS_METRIC_IDX_VCCMIN];

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_GET_MAXVCC))) {
    // Команда: sys.maxvcc
    // Параметры: не требуются
    // Результат: пользователю возвращается значение максимального значения напряжения в mV на входе VCC микроконтроллера с момента подачи питания.
    result = sysMetrics[SYS_METRIC_IDX_VCCMAX];

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ARDUINO_DELAY))) {
    // Команда: delay[value]
    // Параметры: value - время паузы перед выдачей ответа. Задается в миллисекундах.
    // Результат: Возврат значения '1' производится после истечения времени задержки.
    // Примечание: команда является оберткой функции Delay() http://www.arduino.cc/en/Reference/Delay
    delay(arg[0]);
    result = RESULT_IS_OK;

  } else if (strcmp_P(cBuffer, PSTR(CMD_SYS_PORTWRITE)) == 0) {
    // Команда: portWrite[port, value]
    // Параметры: port - символьное обозначение порта (B,C,D..),
    //            value - значение, которое требуется записать в заданный порт ввода/вывода
    // Результат: изменяется состояние порта ввода/вывода (PORTB, PORTC, PORTD...) и происходит возврат значения '1'.
    // Примечание: если ваш экземпляр Arduino имеет более, чем три порта, то на данный момент вам необходимо самостоятельно добавить в скетч информацию о них.
    //
    // Номер порта представляет собой разницу между ASCII-кодом аргумента port и 96. Таким образом b=2, c=3, d=4 и т.д.
    portWrite((byte) arg[0] - 96, arg[1]);
    result = RESULT_IS_OK;

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ARDUINO_ANALOGWRITE))) {
    // Команда: analogWrite[pin, value]
    // Параметры: pin - цифровое обозначение пина, value - значение скважности, которое требуется задать для данного пина.
    // Результат: изменяется скважности PWM для пина. Удачное выполнение команды влечет за собой возврат значения '1', неудачное - значения '0'.
    // Примечание: команда является оберткой функции analogWrite() http://www.arduino.cc/en/Reference/AnalogWrite
    // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит. Если пин не является PWM-совместимым, на нем выставляется значение HIGH.
    // Внимание! Функция analogWrite() самостоятельно устанавливает пин в режим работы OUTPUT
    result = RESULT_IS_FAIL;
    if (isSafePin(arg[0])) {
      analogWrite(arg[0], arg[1]);
      result = RESULT_IS_OK;
    }

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ARDUINO_ANALOGREAD))) {
    // Команда: analogread[pin]
    // Параметры: pin - цифровое обозначение пина
    // Результат: возврат величины, "считанной" с пина. Диапазон значений 0...1023 (возможные варианты диапазона значений зависят от способа подключения сигнала к пину и внутренних настроек Arduino)
    // Примечание: команда является оберткой функции analogRead() www.arduino.cc/en/Reference/AnalogRead
    // Данная команда имеет смысл только для аналоговых пинов.
    // Состояние INPUT для пина должно быть определено в коде скетча. В противном случае совпадения считываемых данных с ожидаемыми может не произойти.   
    result = (long) analogRead(arg[0]);

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ARDUINO_ANALOGREFERENCE))) {
    // Команда: analogReference[source]
    // Параметры: source - источник опорного напряжения (0..N). Значения можно найти в заголовочном файле Arduino.h
    // Результат: устанавливается источник опорного напряжения относительно которого происходят аналоговые измерения и происходит возврат значения '1'
    // Примечание: команда является оберткой функции analogReference() www.arduino.cc/en/Reference/AnalogReference
    analogReference(arg[0]);
    result = RESULT_IS_OK;

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ARDUINO_DIGITALWRITE))) {
    // Команда: digitalWrite[pin, value]
    // Параметры: pin - цифровое обозначение пина, value - значение, которое требуется выставить на заданном пине.
    // Результат: изменяется состояние пина. Удачное выполнение команды влечет за собой возврат значения '1', неудачное - значения '0'.
    // Примечание: команда является оберткой функции digitalWrite() www.arduino.cc/en/Reference/DigitalWrite
    // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит.    
    result = RESULT_IS_FAIL;
    if (isSafePin(arg[0])) {
      digitalWrite(arg[0], arg[1]);
      result = RESULT_IS_OK;
    }
    
  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ARDUINO_DIGITALREAD))) {
    // Команда: digitalRead[pin]
    // Параметры: pin - цифровое обозначение пина
    // Результат: возвращается значение, "считанное" с пина. Диапазон значений - HIGH/LOW.
    // Примечание: команда является оберткой функции DigitalRead() http://www.arduino.cc/en/Reference/DigitalRead
    // Состояние INPUT для пина должно быть определено в коде скетча. В противном случае совпадения считываемых данных с ожидаемыми может не произойти.
    result = (long) digitalRead(arg[0]);


#ifdef FEATURE_TONE_ENABLE
  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ARDUINO_TONE))) {
    // Команда: tone[pin, frequency, duration]
    // Параметры: pin - цифровое обозначение пина, frequency - частота сигнала, duration - длительность сигнала
    // Результат: начинается генерация на указанном пине сигнала "прямоугольная волна" заданной частоты.
    // Примечание: команда является оберткой функции tone() http://www.arduino.cc/en/Reference/Tone
    // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит.
    result = RESULT_IS_FAIL;
    if (isSafePin(arg[0])) {
      tone(arg[0], arg[1], arg[2]);
      result = RESULT_IS_OK;
    }

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ARDUINO_NOTONE))) {
    // Команда: noTone[pin]
    // Параметры: pin - цифровое обозначение пина
    // Результат: завершается генерация на указанном пине сигнала "прямоугольная волна"
    // Примечание: команда является оберткой функции noTone() http://www.arduino.cc/en/Reference/NoTone
    // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит.
    result = RESULT_IS_FAIL;
    if (isSafePin(arg[0])) {
      noTone(arg[0]);
      result = RESULT_IS_OK;
    }
#endif

#ifdef FEATURE_SHIFTOUT_ENABLE
  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_SYS_SHIFTOUT))) {
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
    // enableAction used asal latchPinDefined
    enableAction = ('\0' != arg[3]) && isSafePin(arg[2]);   // << корректный способ проверки или нет?  
    result = RESULT_IS_FAIL;
    if (isSafePin(arg[0]) &&  isSafePin(arg[1])) {
       if (enableAction) { digitalWrite(arg[2], LOW); }
       advShiftOut(arg[0], arg[1], arg[3], cBuffer);
       if (enableAction) { digitalWrite(arg[2], HIGH);}
       result = RESULT_IS_OK;
    }
#endif

#ifdef FEATURE_RANDOM_ENABLE
  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_ARDUINO_RANDOMSEED))) {
    // Команда: randomSeed[value]
    // Параметры: value - начальное число ряда псевдослучайных значений
    // Результат: инициализируется генератор псевдослучайных чисел
    // Примечание: команда является оберткой функции randomSeed() http://www.arduino.cc/en/Reference/randomSeed
    randomSeed(arg[0]);
    result = RESULT_IS_OK;

  } else if (strcmp_P(cBuffer, PSTR(CMD_ARDUINO_RANDOM)) == 0) {
    // Команда: random[min, max]
    // Параметры: min, max - нижняя и верхняя границы псевдослучайных значений
    // Результат: возвращается псевдослучайное число
    // Примечание: команда является оберткой функции random() http://www.arduino.cc/en/Reference/random
    //  !! random return long
    result = (long) random(arg[0], arg[1]);
#endif

#ifdef FEATURE_DS18X20_ENABLE
  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_DS18X20_SEARCH))) {
    // Команда: DS18x20.Search[pin]
    // Параметры: pin - цифровое обозначение пина, к которому подключен цифровой термометр DS18x20. 
    // Результат: производится поиск первого цифрового термометра функцией OneWire.Search, его идентификатор в шестнадцатеричном виде возвращается пользователю. 
    //            При отсутствии результатов поиска - возвращается '0';
    result = RESULT_IS_FAIL;
    if (isSafePin(arg[0])) {
        uint8_t dsAddr[8];
        OneWire ow(arg[0]);
        // Any DS-s found?
        if (DS18X20GetFirstID(ow, dsAddr)){ 
            // Move ID to buffer;
            ptonhs(cBuffer, (uint8_t*) dsAddr, sizeof(dsAddr));
            result = RESULT_IN_BUFFER;
        }
    }

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_DS18X20_TEMPERATURE))) {
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
#endif

#ifdef FEATURE_DHT_ENABLE
  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_DHT_TEMPERATURE))) {
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

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_DHT_HUMIDITY))) {
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
#endif

#ifdef FEATURE_BMP085_ENABLE
  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_BMP_TEMPERATURE))) {
    // Команда: BMP.Temperature[sdaPin, sclPin]
    // Параметры: sdaPin, sclPin - цифровые обозначение пинов, к которым подключена шина I2C.
    // Результат: с цифрового датчика BMP085/BMP180 считывается температура и значение в градусах цельсия возвращается пользователю.
    // Примечание: Точность датчика - 0,1C.
    //             sdaPin, sclPin на данный момент не применяются (используются стандартные пины для I2C подключения) и зарезервированы для внедрения SoftTWI интерфейсов.
    //if (isSafePin(arg[0]) && isSafePin(arg[1])) {
      result = BMP085Read(arg[0], arg[1], arg[2], SENS_READ_TEMP, cBuffer);
    //}

  } else if (0 == strcmp_P(cBuffer, PSTR(CMD_BMP_PRESSURE))) {
    // Команда: BMP.Pressure[sdaPin, sclPin, overSampling]
    // Параметры: sdaPin, sclPin - цифровые обозначение пинов, к которым подключена шина I2C.
    //            overSampling - значение, определяющее точность и длительность измерения.
    //            0 - ultra low power, RMS noise = 6Pa, conversion time = 4,5ms ... 3 - ultra high resolution, RMS noise = 3Pa, conversion time = 25,5ms
    // Результат: с цифрового датчика считывается величина атмосферного давления и значение в Паскалях возвращается пользователю.
    // Примечание: sdaPin, sclPin на данный момент не применяются (используются стандартные пины для I2C подключения) и зарезервированы для внедрения SoftTWI интерфейсов.
    //  if (isSafePin(arg[0]) && isSafePin(arg[1])) {
      result = BMP085Read(arg[0], arg[1], arg[2], SENS_READ_PRSS, cBuffer);
    //  }
#endif

  } else {
    // В любом ином случае команда считается неопределенной.
    strcpy(cBuffer, ZBX_NOTSUPPORTED_MSG);
    // Прирощенный ранее счетчик сматывается
    sysMetrics[SYS_METRC_IDX_CMDCOUNT]--;
  }

  // Результат уже выведен исполняемой командой?
  if  (RESULT_IS_PRINTED != result)
  {
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
  }

}



