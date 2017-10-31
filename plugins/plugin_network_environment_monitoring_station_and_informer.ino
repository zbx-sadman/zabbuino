/*/
/=/                                               Zabbuino v1.2.2 and above required
/=/ 
/=/  This plugin written for "Network environment monitoring station & informer" (https://github.com/zbx-sadman/zabbuino/wiki/Zabbuino-User-Cases-in-Russian)
/=/  
/=/  Equipment that need for project: 
/=/    - AM2301 sensor - 2pcs
/=/    - BMP280 sensor - 1pcs
/=/    - MH-Z19 sensor - 1pcs
/=/    - 2004 LCD display + I2C interface module - 1pcs
/=/    - WS2812/WS2811 pixel led module - 1pcs
/=/    - Arduino Mini Pro / Nano / Uno / Any ATMega328 based with Optiboot firmware
/*/
#ifdef FEATURE_USER_FUNCTION_PROCESSING

// Enable or disable some functional blocks:
// Output to LCD
#define FEATURE_USER_DISPLAY_ENABLE
// Using WS2812 Pixel LED to indicate CO2 level by color
#define FEATURE_USER_WS2812_LED_ENABLE

const int32_t  constSensorErrorCode                           = 999;     // This number will be shown if sensor error detected

// System display settings

const uint8_t  constUserDisplaySDAPin                         = 4;       // SDA <-> D4
const uint8_t  constUserDisplaySCLPin                         = 5;       // SCL <-> D5
const uint8_t  constUserDisplayI2CAddress                     = 0x27;    // I2C LCD interface board address
const uint8_t  constUserDisplayBackLight                      = 0x01;    // backlight off
const uint16_t constUserDisplayType                           = 2004;    // 16x2 screen, refer to source code of printToPCF8574LCD() subroutine
const uint32_t constUserDisplayRenewInterval                  = 5000UL;  // Info on user screen can be renew no more that once in 5 sec
const uint32_t constUserDisplayNoRefreshTimeout               = 10000UL; // Display renew must be delayed for 10 sec if network process was used LCD (incoming informational message from Zabbix server exist, for example)

#if defined(FEATURE_USER_WS2812_LED_ENABLE)
// Other indicators settings
const uint8_t  constUserWS2812LedPin                          = A3;       // WS2812 DIN <-> A3
typedef struct {                                                          // 11 bytes:
  int32_t lowerBound;                                                     // 4 bytes
  int32_t upperBound;                                                     // 4 bytes
  uint8_t color[3];                                                       // 3 byte - G & R & B
} co2LevelColors_t ;

// https://tion.ru/blog/normy-co2/
//const co2LevelColors_t constCO2LevelColors[]                 = { {0, 0, {0x40, 0x00, 0x00}}, {800, 900, {0x80, 0x40, 0x00}}, {1000, 1400, {0x40, 0x80, 0x00}}, {2400, 2500, {0x00, 0xFF, 0x00}} };
//const co2LevelColors_t constCO2LevelColors[]                 = { {600, 800, {0x40, 0x00, 0x00}}, {800, 1000, {0x80, 0x40, 0x00}}, {1000, 1900, {0x40, 0x80, 0x00}}, {1900, 2500, {0x00, 0xFF, 0x00}} };
const co2LevelColors_t constCO2LevelColors[]                  = { {0, 0, {0x40, 0x00, 0x00}}, {0, 800, {0x40, 0x40, 0x00}}, {0, 1400, {0x00, 0x40, 0x00}} };
//const uint8_t co2LevelIntervals                              = sizeof(constCO2LevelColors) / sizeof(constCO2LevelColors)[0];
const uint8_t co2LevelIntervals                               = 3;

#endif // FEATURE_USER_WS2812_LED_ENABLE

// Sensors settings

// BMP280, placed inside box
const uint8_t  constBMPI2CAddress                             = 0x76;                    // BMP280 board address
const uint8_t  constBMPSDAPin                                 = constUserDisplaySDAPin;  // Use LCD's TWI bus
const uint8_t  constBMPSCLPin                                 = constUserDisplaySCLPin;  //
const uint8_t  constBMPPressureOversampling                   = 4;                       // 4 is need for Standard resolution  18 bit / 0.66 Pa
const uint8_t  constBMPTemperatureOversampling                = 1;                       // 1 is need for Standard resolution, 16 bit / 0.0050 °C
const uint8_t  constBMPFilterCoef                             = 0;                       // 0 meant "filter off"

// MH-Z19, placed inside box
const uint8_t  constMHZXXPWMPin                               = 2;                       // Sensor's PWM pin <-> D2
const uint16_t constMHZXXRange                                = 5000;                    // MH-Z19 range is 5000PPM

// Indoor DHT sensor
const uint8_t  constIndoorDHTPin                              = 6;                      // Sensor's DATA pin <-> D6
const uint8_t  constIndoorDHTType                             = 21;                      // DHT 21, AM2301

// Outdoor DHT sensor
const uint8_t  constOutdoorDHTPin                             = 7;                       // Sensor's DATA pin <-> D7
const uint8_t  constOutdoorDHTType                            = 21;                      // DHT 21, AM2301


/*****************************************************************************************************************************

   Subroutine calls on start of Zabbuino

*****************************************************************************************************************************/
void initStageUserFunction(char* _buffer) {
  uint8_t arr[]={0x00, 0x00, 0xF0};
  // Note that not all system struct is initialized at this stage and you can't get localIP() or localtime() info
  _buffer[0] = 0x06;
  _buffer[1] = 0x01;
  strcpy_P(&_buffer[2], PSTR("\t  Starting\n   "));
  strcpy_P(&_buffer[strlen(_buffer)], constZbxAgentVersion);
  // push data to LCD via I2C
  SoftTWI.reconfigure(constUserDisplaySDAPin, constUserDisplaySCLPin);
  printToPCF8574LCD(&SoftTWI, constUserDisplayI2CAddress, constUserDisplayBackLight, constUserDisplayType, _buffer);

  WS2812Out(constUserWS2812LedPin, 1, (uint8_t*) &arr, 3);
  /*
    Serial.print("1: ");Serial.println(constCO2LevelColors[0].color[0]);
    Serial.print("2: ");Serial.println(constCO2LevelColors[0].color[1]);
    Serial.print("3: ");Serial.println(constCO2LevelColors[0].color[2]);
  */
}

/*****************************************************************************************************************************

   Subroutine calls when config is already loaded, but network not started yet

*****************************************************************************************************************************/
void netPrepareStageUserFunction(char* _buffer) {
  // 'Network' is global object
  if (Network.isDHCPUsed()) {
    // This message printed only if DHCP used
    // Jump to top-left corner by '\x02' and skip 3 lines by '\n'
    strcpy_P(_buffer, PSTR("\2\n\n\nIP :\tDHCP"));
    //strcpy_P(_buffer, PSTR("\x02\n\n\nIP :\tDHCP"));
  } else {
    strcpy_P(_buffer, PSTR("\2\n\n\nIP :\tfixed"));
  }
  SoftTWI.reconfigure(constUserDisplaySDAPin, constUserDisplaySCLPin);
  // push data to LCD via I2C WITHOUT re-init display (_forceInit_ param is false)
  printToPCF8574LCD(&SoftTWI, constUserDisplayI2CAddress, constUserDisplayBackLight, constUserDisplayType, _buffer, false);
}
/*****************************************************************************************************************************

   Subroutine calls on user.run[] command processing
    _buffer - internal buffer, tune.h > constBufferSize bytes size. Note: If you modify it - you are change options (option#0..5) content
    _optarg - array of pointer to options represented as C-string. _optarg[n] point to part of _buffer that is begin of n-th option
    _argv   - array of options represented as signed numbers.

     This subroutine must return result code to parent function that return it to user.
*****************************************************************************************************************************/
int8_t executeCommandUserFunction(char* _buffer, char* _optarg[], int32_t* argv) {
  //DTSM ( SerialPrint_P(PSTR("Command: ")); Serial.println(_dst);
  // option#0 is provided
  if ('\0' != *_optarg[0]) {
    //SerialPrint_P(PSTR("Option #1 as string: ")); Serial.print(*_optarg[0]);
    //SerialPrint_P(PSTR("& as number: ")); Serial.println(argv[0]);
    return RESULT_IS_OK;
  }
}

/*****************************************************************************************************************************

   Subroutine calls on every loop if no active network session exist

*****************************************************************************************************************************/
void loopStageUserFunction(char* _buffer) {
  int8_t rc;
  static int32_t co2Level                = 0,
                 co2PrevLevel            = 0,
                 insidePressureLevel     = 0,
                 insideTemperatureLevel  = 0,
                 indoorTemperatureLevel  = 0,
                 indoorHumidityLevel     = 0,
                 outdoorHumidityLevel    = 0,
                 outdoorTemperatureLevel = 0;
  static uint8_t sensorReadStep = 0x01;
  //  static uint32_t prevSensorsReadTime = 0;
  static IPAddress deviceIP;

  // DHT sensor should be polled not more often than once every one or two seconds (depended from sensor model).
  // getDHTMetric() have delay inside, but will be better to do something else when sensor have rest to avoid blocking in runtime.
  // Sensors polling is mixed here to get interval properly :
  //     On 1-st step (1 * 1sec) we take DHT temperature, on 3-rd (1 * 3 sec) - DHT humidity. Polling interval are 2 sec.
  //
  switch (sensorReadStep) {
    // Get outdoor temperature
    case 0x01:
      rc = getDHTMetric(constOutdoorDHTPin, constOutdoorDHTType, SENS_READ_TEMP, &outdoorTemperatureLevel);
      if (RESULT_IN_BUFFER != rc) {
        // Place error code fo temperature field
        outdoorTemperatureLevel = constSensorErrorCode;
      } else {
        // Take whole part of degree only
        outdoorTemperatureLevel = (outdoorTemperatureLevel) / 10;
      }
      // Renew IP info on this step too
      deviceIP = Network.localIP();
      break;

    // Get pressure
    case 0x02:
      SoftTWI.reconfigure(constBMPSDAPin, constBMPSCLPin);
      rc = getBMPMetric(&SoftTWI, constBMPI2CAddress, constBMPPressureOversampling, constBMPFilterCoef, SENS_READ_PRSS, &insidePressureLevel);
      if (RESULT_IN_BUFFER != rc) {
        // Place error code fo pressure field
        insidePressureLevel = constSensorErrorCode;
      } else {
        // Convert Pa to mm rt st
        insidePressureLevel = insidePressureLevel / 133;
        // Convert Pa to kPa
        // insidePressureLevel = insidePressureLevel / 100;
      }
      //Serial.print("insidePressureLevel = "); Serial.println(insidePressureLevel);
      break;

    // Get indoor temperature
    case 0x03:
      rc = getDHTMetric(constIndoorDHTPin, constIndoorDHTType, SENS_READ_TEMP, &indoorTemperatureLevel);
      if (RESULT_IN_BUFFER != rc) {
        // Place error code fo temperature field
        indoorTemperatureLevel = constSensorErrorCode;
      } else {
        // Take whole part of degree only
        indoorTemperatureLevel = (indoorTemperatureLevel) / 10;
      }
      break;

    // Get temperature inside box
    case 0x04:
      rc = getBMPMetric(&SoftTWI, constBMPI2CAddress, constBMPTemperatureOversampling, constBMPFilterCoef, SENS_READ_TEMP, &insideTemperatureLevel);
      if (RESULT_IN_BUFFER != rc) {
        // Place error code fo temperature field
        insideTemperatureLevel = constSensorErrorCode;
      } else {
        // Take whole part of degree only
        insideTemperatureLevel = (insideTemperatureLevel + 50) / 100;
      }
      //Serial.print("insideTemperatureLevel = "); Serial.println(insideTemperatureLevel);
      break;

    // Get outdoor humidity
    case 0x05:
      rc = getDHTMetric(constOutdoorDHTPin, constOutdoorDHTType, SENS_READ_HUMD, &outdoorHumidityLevel);
      if (RESULT_IN_BUFFER != rc) {
        // Place error code fo temperature field
        outdoorHumidityLevel = constSensorErrorCode;
      } else {
        // Take whole part of percent only
        outdoorHumidityLevel = (outdoorHumidityLevel) / 10;
      }
      break;

    // Get CO2 level
    case 0x06:
      co2PrevLevel = co2Level;  // need to detect & handle 'previous level value was wrong (9999)' case
      rc = getMHZxxMetricPWM(constMHZXXPWMPin, constMHZXXRange, &co2Level);
      if (RESULT_IN_BUFFER != rc) {
        co2Level = 9999; // constSensorErrorCode;
      }
      // Serial.print("co2Level = "); Serial.println(co2Level);
      break;

    // Get indoor humidity
    case 0x07:
      rc = getDHTMetric(constIndoorDHTPin, constIndoorDHTType, SENS_READ_HUMD, &indoorHumidityLevel);
      if (RESULT_IN_BUFFER != rc) {
        // Place error code fo temperature field
        indoorHumidityLevel = constSensorErrorCode;
      } else {
        // Take whole part of percent only
        indoorHumidityLevel = (indoorHumidityLevel) / 10;
      }
      break;

    //  All sensors are polled, restart round
    default:
      sensorReadStep = 0x00;
      break;
  }
  // Next metric will be taken on next polling
  sensorReadStep++;

  //Show virtual screens
#ifdef FEATURE_USER_DISPLAY_ENABLE
  uint8_t dataLength, co2LevelIdx, i;
  uint32_t nowTime, howLongWork;
  static uint8_t reportVirtualScreenCnt = 0x00,
                 co2LevelPrevIdx = 0x00,
                 clearScreenOnNextStep = false;
  static uint32_t prevUserDisplayRenewTime = 0;
  //  static char animationChars[] = { 0xA1, 0xDF };
  //  static char animationChars[] = { 0xA2, 0xA3 };
  static char animationChars[] = { 'H', 'h' };
  static uint8_t animationStep = 0x00;

  // Get current time
  nowTime = millis();

  // LCD was used by external process (incoming message from Zabbix server exist, for example) and renew not allowed for some secs (constUserDisplayNoRefreshTimeout)
  if (constUserDisplayNoRefreshTimeout > (uint32_t) (nowTime - sysMetrics.sysLCDLastUsedTime )) {
    return;
  }

  // do nothing if renew wait time is not expiried
  if (constUserDisplayRenewInterval > (uint32_t) (nowTime - prevUserDisplayRenewTime)) {
    return;
  }


  prevUserDisplayRenewTime = nowTime;

  // Clear all screen or command to LCD controller to jump 'top-left home' position
  _buffer[0] = (clearScreenOnNextStep) ? 0x01 : 0x02;
  dataLength = 1;
  clearScreenOnNextStep = false;

  switch (reportVirtualScreenCnt) {
    case 0x00:
      // Replace 'IP : DHCP' message to 'IP : XXX.XXX.XXX.XXX' on very first step
      dataLength = sprintf_P(_buffer, PSTR("\x0C\x02\n\n\nIP : %03d.%03d.%03d.%03d"), deviceIP[0], deviceIP[1], deviceIP[2], deviceIP[3]);
      // Clear need before print any other information on the screen
      clearScreenOnNextStep = true;
      break;
    case 0x01:
      howLongWork = (uptime() / 60 / 60 ) % 100 ;
      // [s]printf is a function that can show to us many tricks. For example:
      // - "\xDFC" will be coverted not to 0xDF & 'C', to 0xFC. "\xDF\C" is the same substring that give 0xDF & 'C' to us.
      // - '+' char in format give +/- signs before digits (it good for temperature value)
      dataLength = sprintf_P(&_buffer[dataLength], PSTR("\x0CIN : %+4ld\xDF\C %3ld%% %2ld%c\n\t%4ld mm %4ld PPM\n\n\nOUT: %+4ld\xDF\C %3ld%%"), indoorTemperatureLevel, \
                             indoorHumidityLevel, howLongWork, animationChars[animationStep], insidePressureLevel, co2Level, outdoorTemperatureLevel, outdoorHumidityLevel);
      animationStep++;
      if (sizeof(animationChars) <= animationStep) {
        animationStep = 0x00;
      }
      break;
    //animationChars[animationStep] howLongWork
    /*
       case 0x02:
          // uncomment this block to build your own virtual screen
         break;
    */
    default:
      // Restart round & skip init screen
      reportVirtualScreenCnt = 0x00;
      break;
  }
  if (EOF == dataLength) {
    dataLength = 0;
  }
  _buffer[dataLength + 1] = '\0';
  // Another Virtual Screen will be shown on next step
  reportVirtualScreenCnt++;
  // push data to LCD via I2C
  SoftTWI.reconfigure(constUserDisplaySDAPin, constUserDisplaySCLPin);
  // !!! _forceInit_ function parameter must be equal to false to avoid LCD reinit. Otherwise all data on the screen can be cleared or spoiled !!!
  printToPCF8574LCD(&SoftTWI, constUserDisplayI2CAddress, constUserDisplayBackLight, constUserDisplayType, _buffer, false);
#endif //FEATURE_USER_DISPLAY_ENABLE
#if defined(FEATURE_USER_WS2812_LED_ENABLE)

  char co2Direction = (co2Level > co2PrevLevel) ? '>' : '<';
  DTSL ( Serial.print("co2Level: "); Serial.print(co2PrevLevel); Serial.print(" "); Serial.print(co2Direction); Serial.print(" "); Serial.println(co2Level); )
  for (i = 0; co2LevelIntervals > i; i++) {
  DTSL ( Serial.print("constCO2LevelColors[");Serial.print(i);Serial.print("]=");Serial.println(constCO2LevelColors[i].upperBound); )
  // Co2 is growing, need to compare upper bound of Co2 control intervals
  //  }
  // Use internal procedure WS2812Out(_dataPin, _compressionType, _src, _len) with:
  // _dataPin = constUserWS2812LedPin
  // _compressionType = 1 ('repeat' type compression)
  // _src = current constCO2LevelColors.color
  // _len = size of current color[] array from co2LevelColors_t struct
  //
  // sizeof(((co2LevelColors_t){0}).color) is sizeof(co2LevelColors_t.color[]))
  //      WS2812Out(constUserWS2812LedPin, 1, (uint8_t*) constCO2LevelColors[i].color, sizeof(((co2LevelColors_t){0}).color));
        if (co2Level > constCO2LevelColors[i].upperBound) { co2LevelIdx = i; }
  }
  DTSL ( Serial.print("Use idx="); Serial.println(co2LevelIdx); )
  WS2812Out(constUserWS2812LedPin, 1, (uint8_t*) constCO2LevelColors[co2LevelIdx].color, 3);
  
#endif // FEATURE_USER_WS2812_LED_ENABLE


}

#endif // FEATURE_USER_FUNCTION_PROCESSING


