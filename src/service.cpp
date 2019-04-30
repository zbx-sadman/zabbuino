// Config & common included files
#include "sys_includes.h"

#include <avr/boot.h>
#include <util/atomic.h>
#include <util/crc16.h>

#include "system.h"
#include "service.h"
#include "eeprom.h"

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
    //incomingData = 
    _stream->read();
    if (millis() - startTime > _timeout) {return false; }
    if (_slowMode) { delay(10); }

 } while (0 > _stream->available());

 return true;
}

/*****************************************************************************************************************************
*
*   
*   
*
*****************************************************************************************************************************/
uint8_t factoryReset(netconfig_t& _sysConfig) {
  uint8_t rc = false;
   // Factory reset button pin must be shorted to ground to action start
   pinMode(constFactoryResetButtonPin, INPUT_PULLUP);

   digitalWrite(constStateLedPin, LOW);

   // Is constFactoryResetButtonPin shorted?
   if (LOW == digitalRead(constFactoryResetButtonPin)) {
      DTSL( FSH_P((STRING_The_factory_reset_button_is_pressed)); )
      // Fire up state LED
      digitalWrite(constStateLedPin, HIGH);
      // Wait some msecs
      delay(constHoldTimeToFactoryReset);
      // constFactoryResetButtonPin still shorted?
      if (LOW == digitalRead(constFactoryResetButtonPin)) {
         DTSL( Serial.print(FSH_P(STRING_Rewrite_EEPROM_with_defaults)); Serial.print(FSH_P(STRING_3xDot_Space)); )
         setConfigDefaults(_sysConfig);
         // return "sucess" only if default config saved
         rc = saveConfigToEEPROM(_sysConfig);
         // Blink fast while constFactoryResetButtonPin shorted to GND
         DTSL( Serial.print(FSH_P(STRING_Release_the_factory_reset_button_now)); )
         while (LOW == digitalRead(constFactoryResetButtonPin)) { digitalWrite(constStateLedPin, millis() % 100 < 50); }
      }
  } // if (LOW == digitalRead(constFactoryResetButtonPin))

  digitalWrite(constStateLedPin, LOW);
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
  static uint16_t numRollovers = 0,               // variable that permanently holds the number of rollovers since startup
                  readyToRoll = false;            // tracks whether we've made it halfway to rollover
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
  return ((uint32_t) millisRollover() * (UINT32_MAX / 1000) + (millis() / 1000UL));
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

  _sysConfig.hostname[constAgentHostnameMaxLength] = '\0';

#ifdef FEATURE_NET_USE_MCUID
  copyCharsNumber = (constAgentHostnameMaxLength <= (constMcuIdSize * 2)) ? constAgentHostnameMaxLength : constMcuIdSize * 2;
  getBootSignatureAsHexString((char*) &_sysConfig.hostname, 0x0E, copyCharsNumber / 2, 1);
#else
  /*
  // old copying code +22 progspace bytes on GCC optimize ("O0")
  copyCharsNumber = strlen_P(constZbxAgentDefaultHostname);
  copyCharsNumber = (constAgentHostnameMaxLength <= copyCharsNumber) ? constAgentHostnameMaxLength : copyCharsNumber;
  strncpy_P(_sysConfig.hostname, constZbxAgentDefaultHostname, copyCharsNumber);
  */
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
    if (0 == _fracBits) { return; }
    while (*_dst) {_dst++;}
    *_dst = '.'; _dst++;
    tmp = 1;
    while (0 < _fracBits) {
        tmp <<= 1;
        tmp |= 1;
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
  if (0 == value) { _dst[0] = '0';  _dst[1] = '\0'; return;} 
 
  // negative value testing and write '-' to output buffer
  if (0 > value) { value = 0 - value; *_dst = '-'; _dst++;} 
  
  // Use all dividers 
  for (i = 0; i < maxStringLen; i++) {
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
    if (0 >= value and pointIsUsed) {break;}
  }
  *_dst = '\0';
}

/*****************************************************************************************************************************
*
*  Convert _len chars (exclude 0x prefix) of hex string to byte array
*
*****************************************************************************************************************************/
int16_t hstoba(uint8_t* _dst, const char* _src)
{
  int16_t len = 0;
  // don't fill _array and return false if mailformed string detected
  if (!haveHexPrefix(_src)) { return -1; }
  // skip prefix
  _src += 2;

  while (*_src) {
     *_dst = (htod(*_src) << 4);
     ++_src;
     if (*_src) {
        *_dst |= htod(*_src);
        ++_src; 
     }
    ++_dst;
    ++len;
  }

  /*
  // for all bytes do...
  while (_len--)  {
     *_dst = (htod(*_src) << 4);
     _src++;
     *_dst |= htod(*_src);
      _src++; _dst++;
  };
*/
  return len;
}

/*****************************************************************************************************************************
*
*   Compute a Dallas Semiconductor 8 bit CRC directly. This is much slower, but much smaller, than the lookup table.
*
*   This function placed here to aviod compilation error when OneWire library is not #included
*
*****************************************************************************************************************************/
uint8_t dallas_crc8(uint8_t *_src, uint8_t _len)
{
  uint8_t crc = 0;
  while (_len) {
    --_len;
    uint8_t inbyte = *_src++;
    crc = _crc_ibutton_update(crc, inbyte);

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
           i = 7;
           while (i) {
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

      _stream.print(_src[pos], format);
      pos++;
      if (pos < _len) { _stream.print(separator); }
   }
   _stream.println();
}

void blinkMore(const uint8_t _times, const uint16_t _onTime, const uint16_t _offTime) 
{
  for (uint8_t i=0; i < _times ; i++) {
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
uint8_t strToNetworkAddress(const char* _src, uint32_t& _dstAddress) {
  if ('\0' == *_src) { return false; }
  _dstAddress = htonl(strtoul(_src, NULL, 0));
  return true;
}

/*****************************************************************************************************************************

   Stream analyzing subroutine
   Detect Zabbix packets, on-fly spit incoming stream to command & arguments

**************************************************************************************************************************** */
uint8_t analyzeStream(char _charFromClient, char* _dst, const uint8_t doReInit, packetInfo_t& _packetInfo) {
  uint8_t static allowAnalyze          = true, 
                 cmdSliceNumber        = 0,
                 isEscapedChar         = 0,
                 doubleQuotedString    = false,
                 packetRecieved        = false;
  uint16_t static bufferWritePosition  = 0;

  // Jump into reInitStage procedure. This is a bad programming style, but the subroutine must be lightweight.
  if (doReInit) {
    // Temporary clean code stub
    _packetInfo.dataLength = 0x00;
    _packetInfo.type = PACKET_TYPE_PLAIN;
    *_dst = '\0';
    goto reInitStage;
  }

  // If there is not room in buffer - simulate EOL recieving
  if (constBufferSize <= bufferWritePosition ) {
    _charFromClient = '\n';
  }

  // Put next char to buffer
  _dst[bufferWritePosition] = (doubleQuotedString) ? _charFromClient : tolower(_charFromClient);

  // When ZBX_HEADER_PREFIX_LENGTH chars is saved to buffer - test its for Zabbix2 protocol header prefix ("ZBXD\01") presence
  // (ZBX_HEADER_PREFIX_LENGTH-1) was used because bufferWritePosition is start count from 0, not from 1
  if ((ZBX_HEADER_PREFIX_LENGTH-1) == bufferWritePosition) {
    if (0 == memcmp(_dst, (ZBX_HEADER_PREFIX), ZBX_HEADER_PREFIX_LENGTH)) {
      // If packet have prefix - it is Zabbix's native packet
      _packetInfo.type = PACKET_TYPE_ZABBIX;
      DTSD( Serial.print(FSH_P(STRING_ZBX_header)); Serial.println(FSH_P(STRING_detected)); )
      allowAnalyze = false;
    }
  }

  // Allow packet analyzing when Zabbix header is skipped
  if (ZBX_HEADER_LENGTH == bufferWritePosition && PACKET_TYPE_ZABBIX == _packetInfo.type) {
    //bufferWritePosition = 0;
    //needSkipZabbix2Header = false;
    allowAnalyze = true;
    _packetInfo.dataLength = 0x00;
    memcpy(&(_packetInfo.expectedDataLength), &_dst[ZBX_HEADER_PREFIX_LENGTH], sizeof(_packetInfo.expectedDataLength));
    DTSD( Serial.print(FSH_P(STRING_ZBX_header)); Serial.println(FSH_P(STRING_passed)); )
    // Return 'Need next char' and save a lot cpu time
    //return true;
  }

  // Development mode only debug message level used
  DTSD( Serial.print("in: "); Serial.print(_dst[bufferWritePosition], HEX); Serial.print(" '"); Serial.print((char) _dst[bufferWritePosition]); Serial.print("' "); )

  // Process all chars if its not from header data
  if (allowAnalyze) {
    // char is not escaped
    DTSD( Serial.println(FSH_P(STRING_3xDot_Space));  )
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
        if (!doubleQuotedString) {
          return true;
        }
        break;

      // Delimiter or separator found.
      case '[':
      case ',':
        // If its reached not in doublequoted string - process it as control char.
        if (!doubleQuotedString) {
          //  If '_argOffset' array is not exhausted - push begin of next argument (bufferWritePosition+1) on buffer to arguments offset array.
          if (constArgC > cmdSliceNumber) {
            _packetInfo.optarg[cmdSliceNumber] = &_dst[bufferWritePosition + 1];
          }
          cmdSliceNumber++;
          // Make current buffer segment like C-string
          _dst[bufferWritePosition] = '\0';
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
        _dst[bufferWritePosition] = '\0';
        packetRecieved = true;
        //while (constArgC > cmdSliceNumber) { _argOffset[cmdSliceNumber++] = bufferWritePosition;}
        while (constArgC > cmdSliceNumber) {
          _packetInfo.optarg[cmdSliceNumber++] = &_dst[bufferWritePosition];
        }
        // Change argIndex value to pass (constArgC < argIndex) condition
        cmdSliceNumber = constArgC + 1;
        break;

      // All next chars is non-escaped
      default:
        isEscapedChar = false;
    }
    _packetInfo.dataLength++;

    if ((_packetInfo.dataLength >= _packetInfo.expectedDataLength) && (PACKET_TYPE_ZABBIX == _packetInfo.type)) {
        packetRecieved = true;
        // !!! potential break
        _dst[bufferWritePosition+1] = '\0';
    }   

    // Rework need
    if (packetRecieved) {
  //    Serial.println("Packet finished"); 
        //while (constArgC > cmdSliceNumber) { _argOffset[cmdSliceNumber++] = bufferWritePosition;}
        while (constArgC > cmdSliceNumber) {
          _packetInfo.optarg[cmdSliceNumber++] = &_dst[bufferWritePosition];
        }
        // Change argIndex value to pass (constArgC < argIndex) condition
        cmdSliceNumber = constArgC + 1;
    }

    // EOL reached or there is not room to store args. Stop stream analyzing and do command executing
    if (constArgC < cmdSliceNumber) {
reInitStage:
      DTSH( Serial.println(FSH_P(STRING_Reinit_parser)); )
      // Clear vars for next round, and return false as 'Do not need next char'
      bufferWritePosition = cmdSliceNumber = isEscapedChar = doubleQuotedString = 0x00;
      doubleQuotedString = false;
      packetRecieved = false;
      allowAnalyze = true;
      return false;
    }
  } else {
    DTSD( Serial.println();  )
  }
  //
  bufferWritePosition++;
  // Return 'Need next char' and save a lot cpu time
  return true;
}
