// Config & common included files
#include "sys_includes.h"
#include "wrap_network.h"

#if defined(ARDUINO_ARCH_AVR)
    #include <avr/boot.h>
    #include <util/atomic.h>
    #include <util/crc16.h>
#endif

#include "system.h"
#include "service.h"
#include "eeprom.h"
#include "ow_bus.h"

/*****************************************************************************************************************************
*
*   
*   
*
*****************************************************************************************************************************/
uint8_t flushStreamRXBuffer(Stream* _stream, const uint32_t _timeout, const uint8_t _slowMode = false) {
 //int16_t incomingData;
 uint32_t startTime = millis();

 do { 
    yield();
    _stream->read();
    if (millis() - startTime > _timeout) {return false; }
    if (_slowMode) { delay(10); }

 } while (0x00 > _stream->available());

 return true;
}

/*****************************************************************************************************************************
*
*   
*   
*
*****************************************************************************************************************************/
uint8_t factoryReset(uint8_t _buttonPin, uint8_t _buttonState, netconfig_t& _sysConfig) {
  uint8_t rc = false;
   // Factory reset button pin must be shorted to ground to action start
   digitalWrite(constStateLedPin, !constStateLedOn);
   // Is constFactoryResetButtonPin shorted?
   if (_buttonState == digitalRead(_buttonPin)) {
      __DMLL( FSH_P((STRING_The_factory_reset_button_is_pressed)); )
      // Fire up state LED
      digitalWrite(constStateLedPin, constStateLedOn);
      // Wait some msecs
      delay(constHoldTimeToFactoryReset);
      // constFactoryResetButtonPin still shorted?
      if (_buttonState == digitalRead(_buttonPin)) {
         __DMLL( DEBUG_PORT.print(FSH_P(STRING_Rewrite_EEPROM_with_defaults)); DEBUG_PORT.print(FSH_P(STRING_3xDot_Space)); )
#if (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)) && defined(NETWORK_WIRELESS_ESP_NATIVE)
        setWifiDefaults();
#endif //defined(ARDUINO_ARCH_ESP8266)
         setConfigDefaults(_sysConfig);
         // return "sucess" only if default config saved
         rc = saveConfigToEEPROM(_sysConfig);
         if (!rc) { __DMLM( DEBUG_PORT.print(FSH_P(STRING_Config)); DEBUG_PORT.println(FSH_P(STRING_saving_error)); ) }

         // Blink fast while constFactoryResetButtonPin shorted to GND
         __DMLL( DEBUG_PORT.println(FSH_P(STRING_Release_the_factory_reset_button_now)); )
         while (_buttonState == digitalRead(_buttonPin)) { yield(); digitalWrite(constStateLedPin, (millis() % 100 < 50) ? constStateLedOn : !constStateLedOn); }
      }
  } // if (LOW == digitalRead(constFactoryResetButtonPin))
  digitalWrite(constStateLedPin, !constStateLedOn);
  return rc;
}

/*****************************************************************************************************************************
*
*   Return number of millis() rollovers every UINT32_MAX ms (~50days)
*   Must be called every (UINT32_MAX / 2) - 1 ms at least
*
*****************************************************************************************************************************/
uint16_t millisRollover(void) {
  // get the current millis() value for how long the microcontroller has been running
  //
  // To avoid any possiblity of missing the rollover, we use a boolean toggle that gets flipped
  //   off any time during the first half of the total millis period and
  //   then on during the second half of the total millis period.
  // This would work even if the function were only run once every 4.5 hours, though typically,
  //   the function should be called as frequently as possible to capture the actual moment of rollover.
  // The rollover counter is good for over 35 years of runtime. --Rob Faludi http://rob.faludi.com
  //
  static uint16_t numRollovers = 0x00,            // variable that permanently holds the number of rollovers since startup
                  readyToRoll  = false;           // tracks whether we've made it halfway to rollover
  const uint32_t  halfwayMillis = UINT32_MAX / 2; // this is halfway to the max millis value (17179868 for earlier versions of Arduino)
  uint32_t now =  millis();                       // the time right now

  // as long as the value is greater than halfway to the max
  // you are ready to roll over
  if (now > halfwayMillis) { readyToRoll = true; }

  if (readyToRoll == true && now < halfwayMillis) {
    // if we've previously made it to halfway
    // and the current millis() value is now _less_ than the halfway mark
    // then we have rolled over
    numRollovers++; // add one to the count the number of rollovers
    readyToRoll = false; // we're no longer past halfway
  } 
  return numRollovers;
}

/*****************************************************************************************************************************
*
*   Return system uptime (seconds)
*
*****************************************************************************************************************************/
uint32_t uptime(void) {
  return ((uint32_t) millisRollover() * (UINT32_MAX / 1000UL) + (millis() / 1000UL));
}

void getMcuIdAsHexString(char* _dst) {
  uint8_t chipID[constMcuIdSize];
  memset(chipID, 0x00, sizeof(chipID));
  getMcuId(chipID);
  batohs(chipID, _dst, sizeof(chipID));
}

void getMcuModelAsHexString(char* _dst) {
  uint8_t modelID[0x03];
  memset(modelID, 0x00, sizeof(modelID));
  getMcuModel(modelID);
  batohs(modelID, _dst, sizeof(modelID));
}


/*****************************************************************************************************************************
*
*   Set default values of network configuration
*
*****************************************************************************************************************************/
void setConfigDefaults(netconfig_t& _sysConfig) {
  
  uint8_t copyCharsNumber;
  // Copying arrays of the default data to the config
  memcpy_P(&_sysConfig.macAddress, constDefaultMacAddress, sizeof(_sysConfig.macAddress));
  memcpy_P(&_sysConfig.ipAddress,  constDefaultIPAddress,  sizeof(_sysConfig.ipAddress));
  memcpy_P(&_sysConfig.ipGateway,  constDefaultGateway,    sizeof(_sysConfig.ipGateway));
  memcpy_P(&_sysConfig.ipNetmask,  constDefaultNetmask,    sizeof(_sysConfig.ipNetmask));

  _sysConfig.useDHCP         = constNetDefaultUseDHCP;
  _sysConfig.password        = constSysDefaultPassword;
  _sysConfig.useProtection   = constSysDefaultProtection;

  _sysConfig.tzOffset = constSysTZOffset;

  _sysConfig.hostname[constAgentHostnameMaxLength] = CHAR_NULL;

#ifdef FEATURE_NET_USE_MCUID
  copyCharsNumber = (constAgentHostnameMaxLength <= (constMcuIdSize * 0x02)) ? constAgentHostnameMaxLength : constMcuIdSize * 0x02;
  getMcuIdAsHexString(_sysConfig.hostname);
#else
  strncpy_P(_sysConfig.hostname, constZbxAgentDefaultHostname, constAgentHostnameMaxLength);
  copyCharsNumber = strlen(_sysConfig.hostname);
#endif
  strncpy_P(&_sysConfig.hostname[copyCharsNumber], constZbxAgentDefaultDomain, constAgentHostnameMaxLength - copyCharsNumber);
}

/*****************************************************************************************************************************
*
*   Convert _Qm.n_ float number (int64_t) to char[] 
*
*****************************************************************************************************************************/
void qtoaf(const int64_t _number, char *_dst, uint8_t _fracBits){
    int64_t tmp; 
    // Write to _dst text representation of whole part, decimal comma, and add fract part if its exists
    tmp = _number >> _fracBits;
    ltoa(tmp, _dst, 10);
    if (0x00 == _fracBits) { return; }
    while (*_dst) { yield(); _dst++;}
    *_dst = '.'; _dst++;
    tmp = 1;
    while (0x00 < _fracBits) {
        tmp <<= 0x01;
        tmp |=  0x01;
        --_fracBits;
    }
    tmp = _number & tmp;
    ltoa(tmp, _dst, 10);
    return; 
}

/*****************************************************************************************************************************
*
*   Convert int32_t _number to char[]  with decimal point on _num_after_dot position 
*   _number / (10 * _num_after_dot position) => char[]
*
*****************************************************************************************************************************/
void ltoaf(const int32_t _number, char* _dst, const uint8_t _num_after_dot)
{
  uint8_t i, skipLeadingZeros = true, pointIsUsed = false;;
  char currChar;
  int32_t value = _number;
  const uint8_t maxStringLen = 10;
  // int32_t used cuz value & _number is int32_t too
  const int32_t dividers[maxStringLen]={1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};
  
  // If Zero given - Zero returned without long processing 
  if (0x00 == value) { _dst[0x00] = '0';  _dst[0x01] = CHAR_NULL; return;} 
 
  // negative value testing and write '-' to output buffer
  if (0x00 > value) { value = 0x00 - value; *_dst = '-'; _dst++;} 
  
  // Use all dividers 
  for (i = 0x00; i < maxStringLen; i++) {
    yield();
    // Its _num_after_dot-th position ?
    if ((maxStringLen - i) == _num_after_dot) {
        // If non-zero character has not yet processeed - push '0' before decimal point
        if (skipLeadingZeros) {*_dst = '0'; _dst++;}
// +10 byte to used flash
//        if (0 >= value) {break;}
        // push decimal point
        *_dst = '.'; _dst++; 
        // Need to process all next zeros
        skipLeadingZeros = false;
        pointIsUsed = true;
    }
    // Init character value
    currChar = '0';
    // If divider more than digit in current 'position' 
    // 100 <= 6xx
    while (dividers[i] <= value) {
      yield();
      // Decrease that digit to next comparison (6xx, 5xx, 4xx, 3xx, 2xx, 1xx, 0xx)
      value -= dividers[i];
      // Increase character value
      ++currChar;
    }
    // When the 'while' cycle is completed, the value of currChar will be increased N-th times
    // ('0', '1', '2', '3', '4', '5', '6') and currChar wil be represent digit that placed into
    // tested position.

    // All leadings Zeros must be skipped and do not written to output buffer
    if (currChar == '0' and skipLeadingZeros) { continue; }
    // Any non-zero sign processed - all following Zeros must be written to output buffer
    skipLeadingZeros = false; 
    // Push currChar to buffer
    *_dst = currChar;
    ++_dst;
    // do not add trailing zeros after dot
    if (0x00 >= value and pointIsUsed) {break;}
  }
  *_dst = CHAR_NULL;
}

/*****************************************************************************************************************************
*
*  Convert _len chars (exclude 0x prefix) of hex string to byte array
*
*****************************************************************************************************************************/
int16_t hstoba(uint8_t* _dst, const char* _src) {
  int16_t len = 0x00;
  // don't fill _array and return false if mailformed string detected
  if (!haveHexPrefix(_src)) { return -0x01; }
  // skip prefix
  _src += 0x02;

  while (*_src) {
     yield();
     *_dst = (htod(*_src) << 0x04);
     _src++;
     if (*_src) {
        *_dst |= htod(*_src);
        _src++; 
     }
    _dst++;
    len++;
  }
  return len;
}

/*****************************************************************************************************************************
*
*  Convert byte array to hex string
*
*****************************************************************************************************************************/
void batohs(uint8_t* _src, char* _dst, uint8_t _size) {
  while (_size) {
     yield();
     *_dst++ = dtoh((uint8_t) ((0x0F <= *_src) ? (*_src >> 0x04) : 0x00));
     *_dst++ = dtoh((uint8_t) (*_src & 0x0F));
     _src++;
     _size--;
  }
  *_dst = CHAR_NULL;
}

/*****************************************************************************************************************************
*
*   Compute a Dallas Semiconductor 8 bit CRC directly. This is much slower, but much smaller, than the lookup table.
*
*   This function placed here to aviod compilation error when OneWire library is not #included
*
*****************************************************************************************************************************/
uint8_t dallas_crc8(uint8_t* _src, uint8_t _len) {
  uint8_t crc = 0x00;
  while (_len) {
    yield();
    --_len;
    uint8_t inbyte = *_src++;
#if defined(ARDUINO_ARCH_AVR)
    crc = _crc_ibutton_update(crc, inbyte);
#else
    for (uint8_t i = 0x08; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 0x01;
      if (mix) crc ^= 0x8C;
      inbyte >>= 0x01;
    }
#endif
  }
  return crc;
}

/*****************************************************************************************************************************
*
*  Print array to Serial as MAC or IP or other string with sign-separated parts
*
*****************************************************************************************************************************/
void printArray(uint8_t* _src, const uint8_t _len, Stream& _stream, const uint8_t _type) {
  char separator;
  uint8_t format, i;
  uint16_t pos = 0;

  format = HEX;
  switch (_type) {
    case OW_ADDRESS:
      separator = '\n';
      break;
    case I2C_ADDRESS:
      separator = '\n';
      break;
    case MAC_ADDRESS:
      separator = ':';
      break;
    case IP_ADDRESS:
    default:
      format = DEC;
      separator = '.';
   }     

   while (pos < _len) {
      switch (_type) {
         case OW_ADDRESS:
           _stream.print(FSH_P(STRING_HEX_Prefix));
           // Last byte was printed on the ** (see below)
           i = ONEWIRE_ID_SIZE - 1;
           while (i) {
             yield();
             i--;
             if (0x0F > _src[pos]) { _stream.print('0'); }
             _stream.print(_src[pos], format);
             pos++;
           }
           break;

         case I2C_ADDRESS:
           _stream.print(FSH_P(STRING_HEX_Prefix));
         case MAC_ADDRESS:
           if (0x0F > _src[pos]) { _stream.print('0'); }
           break;

         case IP_ADDRESS:
         default:
           break;
      }     
      // ** common print procedure
      _stream.print(_src[pos], format);
      pos++;
      if (pos < _len) { _stream.print(separator); }
   }
   _stream.print('\n');
}

void blinkMore(const uint8_t _times, const uint16_t _onTime, const uint16_t _offTime) 
{
  for (uint8_t i = 0; i < _times ; i++) {
    digitalWrite(constStateLedPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(_onTime);              // wait for a second
    digitalWrite(constStateLedPin, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(_offTime);              // wait for a second
  }
}
/*
uint8_t validateNetworkAddress(const NetworkAddress _address) {
  return true;
}
*/

uint8_t strToNetworkAddress(char* _src, uint32_t& _ipAddress) {
  uint8_t formatError = false;
  _ipAddress = 0x00;

  if (!_src || CHAR_NULL == *_src) {
    formatError = true;
  } else if (haveHexPrefix(_src)) {
    _ipAddress = htonl(strtoul(_src, NULL, 0x00));
  } else {
    char* ptrStartString = _src;
    uint8_t octetNo = 0x00, eolDetected = false, byteShift = 0x00, bytesRead = 0x00;
    uint32_t octetValue;
    while (!formatError && !eolDetected) {
      yield();
      //Serial.println(*_src);
      if (CHAR_NULL == *_src || '.' == *_src) {
        eolDetected = (CHAR_NULL == *_src);
        *_src = CHAR_NULL;
        octetValue = (ptrStartString) ? strtoul(ptrStartString, NULL, 0x00) : 0x00;
        //Serial.print("Octet #"); Serial.print(octetNo); Serial.print(" => ");  Serial.print(ptrStartString); Serial.print(" => ");  Serial.println(octetValue);
        octetNo++;
        // may be use strtoul && 0x00 >= octetValue ?
        formatError = (0xFF < octetValue || 0x04 < octetNo);
        ptrStartString = _src + 0x01;
        _ipAddress |= (octetValue << byteShift);
        byteShift += 0x08;
      }
      _src++;
      bytesRead++;
      formatError |= (bytesRead > 0x0F);
    }
    formatError |= (0x04 != octetNo);
  }

  if (formatError) {
    _ipAddress = 0x00;
  }

  return !formatError;
}


/*****************************************************************************************************************************

   Parse request subroutine
   Detect Zabbix packets, on-fly spit incoming stream to command & arguments

**************************************************************************************************************************** */
uint8_t parseRequest(char _charFromClient, const uint8_t doReInit, request_t& _request) {
  uint8_t  addChunk = false, rc = true;
  uint8_t  static allowAnalyze, argCounter, isEscapedChar, doubleQuotedString, parsingDone, *ptrChunkStart;
  uint16_t static bufferWritePosition, payloadReadedLength, payloadExpectedLength;

  // Jump into reInitStage procedure. This is a bad programming style, but the subroutine must be lightweight.
  if (doReInit) {
    __DMLD( DEBUG_PORT.print(FSH_P(STRING_Reinit_parser)); )
    _request.type = PACKET_TYPE_PLAIN;
    _request.dataFreeSize = 0x00;
    // Just init pointers with nullptr to mark args with no content
    //memset(_request.args, NULL, sizeof(_request.args)); ???
    uint8_t i = arraySize(_request.args);
    while (i) {
      --i;
      _request.args[i] = nullptr;
    }
    ptrChunkStart = nullptr;
    bufferWritePosition = argCounter = isEscapedChar = doubleQuotedString = payloadReadedLength = 0x00;
    rc = doubleQuotedString = parsingDone = false;
    allowAnalyze = true;
    goto finish;
  }

  // If there is not room in buffer - create fake command CMD_ZBX_NOPE to return ZBX_NOTSUPPORTED
  if (sizeof(_request.data) <= bufferWritePosition ) {
    _request.type = PACKET_TYPE_NONE;
    // Return false as 'Do not need next char'
    rc = false;
    goto finish;
  }

  // Put next char to buffer
  _request.data[bufferWritePosition] = (doubleQuotedString) ? _charFromClient : tolower(_charFromClient);

  // Development mode only debug message level used
  __DMLD( DEBUG_PORT.print(F("in [")); DEBUG_PORT.print(bufferWritePosition); DEBUG_PORT.print(F("]: ")); DEBUG_PORT.print(_request.data[bufferWritePosition], HEX); DEBUG_PORT.print(F(" '")); DEBUG_PORT.print((char) _request.data[bufferWritePosition]); DEBUG_PORT.print(F("' ")); )

  // When ZBX_HEADER_PREFIX_LENGTH chars is saved to buffer - test its for Zabbix2 protocol header prefix ("ZBXD\01") presence
  // (ZBX_HEADER_PREFIX_LENGTH-1) was used because bufferWritePosition is start count from 0, not from 1
  if ((sizeof(zbxHeaderPrefix) - 1) == bufferWritePosition) {
    if (0x00 == memcmp_P(_request.data, zbxHeaderPrefix, sizeof(zbxHeaderPrefix))) {
      // If packet have prefix - it is Zabbix's native packet
      _request.type = PACKET_TYPE_ZABBIX;
      __DMLD( DEBUG_PORT.print(FSH_P(STRING_ZBX_header)); DEBUG_PORT.print(FSH_P(STRING_detected)); )
      allowAnalyze = false;
    }
  }

  // Allow packet analyzing when Zabbix header is skipped
  if ((ZBX_HEADER_LENGTH - 0x01) == bufferWritePosition && PACKET_TYPE_ZABBIX == _request.type) {
    allowAnalyze = true;
    payloadReadedLength = 0x00;
    memcpy(&(payloadExpectedLength), &_request.data[ZBX_HEADER_PREFIX_LENGTH], sizeof(payloadExpectedLength));
    // Expected length is more that buffer, processing must be stopped
    if (sizeof(_request.data) < (payloadExpectedLength + ZBX_HEADER_LENGTH)) {
      __DMLD( DEBUG_PORT.println(FSH_P(STRING_Expected_data_so_big));)
      _request.type = PACKET_TYPE_NONE;
      // Return false as 'Do not need next char'
      rc = false;
      goto finish;
    }
    __DMLD( DEBUG_PORT.print(FSH_P(STRING_ZBX_header)); DEBUG_PORT.print(FSH_P(STRING_passed)); )
  }

  // Process all chars if its not from header data
  if (allowAnalyze) {
    // char is not escaped
    __DMLD( DEBUG_PORT.print(FSH_P(STRING_3xDot_Space));  )
    switch (_charFromClient) {
      // Doublequote sign is arrived
      case '"':
        if (!isEscapedChar) {
          // Doublequote is not escaped - just drop it and toggle "string is doublequoted" mode (do not convert char case,
          //  skip action on space, ']', '[', ',' detection). Then jump out from subroutine to get next char from client
          doubleQuotedString = !doubleQuotedString;
          // rc already inited as true
          goto finish;
        }
        // Doublequote is escaped. Move write position backward to one step and write doublequote sign to '\' position
        bufferWritePosition--;
        _request.data[bufferWritePosition] = '"';
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
        if (!doubleQuotedString) {
          // rc already inited as true
          goto finish;
        }
        break;

      // Starting square bracket found - command part readed, args reading starts
      case '[':
        if (!doubleQuotedString) {
          argCounter = 0x00;
          _request.data[bufferWritePosition] = CHAR_NULL;
          ptrChunkStart = &_request.data[bufferWritePosition + 1];
        }
        break;

      // Separator found.
      case ',':
        // If its reached not in doublequoted string - process it as control char.
        if (!doubleQuotedString) {
          addChunk = true;
        }
        break;

      // Final square bracket found. Do nothing and next char will be written to same position.
      case ']':
        // If its reached in doublequoted string - just leave its as regular character
        //    ...otherwise - process as 'EOL sign'
        if (doubleQuotedString) {
          break;
        }

      // EOL detected
      case '\n':
        // Save last argIndex that pointed to <null> item. All unused _argOffset[] items must be pointed to this <null> item too.
        if (!doubleQuotedString) {
          parsingDone = addChunk = true;
          //__DMLD( DEBUG_PORT.print(F(" NL detected @ ")); DEBUG_PORT.println(bufferWritePosition); )
        }
        break;

      // All next chars is non-escaped
      default:
        isEscapedChar = false;
    }

    // !parsingDone => parsing not already finished due '\n' or ']' found and bufferWritePosition can be increased to terminate string 
    // if all bytes of packet recieved and analyzed
    // agent.ping|0x00 => bufferWritePosition++, buffer[bufferWritePosition] = 0x00
    // agent.ping\n|0x00 => buffer[bufferWritePosition] = 0x00
    if (!parsingDone && (payloadReadedLength >= payloadExpectedLength) && (PACKET_TYPE_ZABBIX == _request.type)) {
      parsingDone = true;
      // ASCIIZ string will be maked later, inside 'if (parsingDone)...' operator
      bufferWritePosition++;
    }

    if (addChunk) {
      addChunk = false;
      uint8_t *ptrChunkEnd = &_request.data[bufferWritePosition];
      //__DMLD( DEBUG_PORT.print(F(" chunk finished with : ")); DEBUG_PORT.println(*ptrChunkEnd, HEX); )
      // Terminate current chunk and make ASCIIZ string
      *ptrChunkEnd = CHAR_NULL;
      // Store pointer if args array is not full, otherwize - finish parsing
      if (arraySize(_request.args) > argCounter) {
        // Store pointer only when data contain between separators - pointer distantion more that pointer size
        if (ptrChunkEnd > ptrChunkStart) {
          _request.args[argCounter] = (char*) ptrChunkStart;
          __DMLD( DEBUG_PORT.print(F(" chunked ")); )
        }
        ptrChunkStart = ptrChunkEnd + 0x01;
        argCounter++;
      } else {
        parsingDone = true;
      }
    }

    if (parsingDone) {
      _request.data[bufferWritePosition] = CHAR_NULL;
      // Return false as 'Do not need next char'
      rc = false; goto finish;
    }

    payloadReadedLength++;

  } // if (allowAnalyze) {
  //
  bufferWritePosition++;
finish:
  // Return 'Need next char' and save a lot cpu time
  __DMLD( DEBUG_PORT.println(); )
  return rc;
}


int8_t makeTextPayload(char* _dst, int32_t _value, int8_t _code) {

  int8_t rc = RESULT_IS_OK;
    switch (_code) {
      case RESULT_IS_BUFFERED:
        break;
      case RESULT_IS_OK:
      case RESULT_IS_SYSTEM_REBOOT_ACTION:
        //  '1' must be returned
        _dst[0] = '1';
        _dst[1] = CHAR_NULL;
        break;
      case RESULT_IS_FAIL:
        // or '0'
        _dst[0] = '0';
        _dst[1] = CHAR_NULL;
        break;
      case RESULT_IS_SIGNED_VALUE:
        //  or _code value placed in 'value' variable and must be converted to C-string.
        ltoa((int32_t) _value, _dst, 10);
        break;
      case RESULT_IS_UNSIGNED_VALUE:
        //  or _code value placed in 'value' variable and must be converted to C-string.
        ultoa((uint32_t) _value, _dst, 10);
        break;
      case DEVICE_ERROR_CONNECT:
        strcpy_P(_dst, MSG_DEVICE_ERROR_CONNECT);
        break;
      case DEVICE_ERROR_ACK_L:
        strcpy_P(_dst, MSG_DEVICE_ERROR_ACK_L);
        break;
      case DEVICE_ERROR_ACK_H:
        strcpy_P(_dst, MSG_DEVICE_ERROR_ACK_H);
        break;
      case DEVICE_ERROR_CHECKSUM:
        strcpy_P(_dst, MSG_DEVICE_ERROR_CHECKSUM);
        break;
      case DEVICE_ERROR_TIMEOUT:
        strcpy_P(_dst, MSG_DEVICE_ERROR_TIMEOUT);
        break;
      case DEVICE_ERROR_WRONG_ID:
        strcpy_P(_dst, MSG_DEVICE_ERROR_WRONG_ID);
        break;
      case DEVICE_ERROR_NOT_SUPPORTED:
        strcpy_P(_dst, MSG_DEVICE_ERROR_NOT_SUPPORTED);
        break;
      case DEVICE_ERROR_WRONG_ANSWER:
        strcpy_P(_dst, MSG_DEVICE_ERROR_WRONG_ANSWER);
        break;
      case DEVICE_ERROR_EEPROM_CORRUPTED:
        strcpy_P(_dst, MSG_DEVICE_ERROR_EEPROM);
        break;
      case ZBX_NOTSUPPORTED:
        strcpy_P(_dst, MSG_ZBX_NOTSUPPORTED);
        break;
      case RESULT_IS_FLOAT_QMN:
#if defined(FUNCTION_QTOAF_USE)
        qtoaf((int32_t)_value, _dst, 10);
#endif
        break;
      default:
        // fast and ugly code block
        if (RESULT_IS_FLOAT_01_DIGIT == _code || RESULT_IS_FLOAT_02_DIGIT == _code || RESULT_IS_FLOAT_03_DIGIT == _code || RESULT_IS_FLOAT_04_DIGIT == _code) {
#if defined(FUNCTION_LTOAF_USE)
          uint8_t numAfterDot = _code - RESULT_IS_FLOAT;
          ltoaf((int32_t) _value, _dst, numAfterDot);
#endif
        } else {
          // otherwise subroutine return unexpected value, need to check its source code
          strcpy_P(_dst, MSG_ZBX_UNEXPECTED_RC);
          rc = RESULT_IS_FAIL;
        }
        break;
    }
  return rc;
}