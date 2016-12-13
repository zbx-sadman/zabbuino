#include "service.h"


/*****************************************************************************************************************************
*
*   Set default values of network configuration
*
*****************************************************************************************************************************/
void setConfigDefaults(netconfig_t *_configStruct)
{
  // Copying defaut MAC to the config
  uint8_t mac[] = NET_DEFAULT_MAC_ADDRESS;
  memcpy(_configStruct->macAddress, mac, arraySize(_configStruct->macAddress));

  _configStruct->useDHCP = constNetDefaultUseDHCP;
  _configStruct->ipAddress = IPAddress(NET_DEFAULT_IP_ADDRESS);
  _configStruct->ipNetmask = IPAddress(NET_DEFAULT_NETMASK);
  _configStruct->ipGateway = IPAddress(NET_DEFAULT_GATEWAY);
  _configStruct->password  = constSysDefaultPassword;
  _configStruct->useProtection = constSysDefaultProtection;
  
#ifdef FEATURE_NET_USE_MCUID
  // if FEATURE_NET_USE_MCUID is defined:
  // 1. Make FDQN-hostname from MCU ID and default domain name
  // 2. Modify MAC - the last default's MAC octet is replaced to the last byte of MCU ID
  // 3. Modify IP - the last default's IP octet is replaced too to the last byte of MCU ID
  getBootSignatureBytes(_configStruct->hostname, 14, 10, 1);
  memcpy(&_configStruct->hostname[constMcuIdLength], (ZBX_AGENT_DEFAULT_DOMAIN), arraySize(ZBX_AGENT_DEFAULT_DOMAIN));
  _configStruct->hostname[constMcuIdLength+sizeof(ZBX_AGENT_DEFAULT_DOMAIN)+1]='\0';
  
  // 
  // Interrupts must be disabled before boot_signature_byte_get will be called to avoid code crush
  noInterrupts();
  _configStruct->macAddress[5] = boot_signature_byte_get(23);
  interrupts();
  _configStruct->ipAddress[3] = _configStruct->macAddress[5];

#else
  // Otherwise - use default hostname and domain name for FDQN-hostname
  memcpy(&_configStruct->hostname[0], (ZBX_AGENT_DEFAULT_HOSTNAME), arraySize(ZBX_AGENT_DEFAULT_HOSTNAME));
  memcpy(&_configStruct->hostname[sizeof(ZBX_AGENT_DEFAULT_HOSTNAME)-1], (ZBX_AGENT_DEFAULT_DOMAIN), arraySize(ZBX_AGENT_DEFAULT_DOMAIN));
  _configStruct->hostname[sizeof(ZBX_AGENT_DEFAULT_HOSTNAME)+sizeof(ZBX_AGENT_DEFAULT_DOMAIN)+1]='\0';
#endif
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
        _fracBits--;
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
      currChar++;
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
    _dst++;
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
uint8_t hstoba(uint8_t *_dst, const char *_src, uint8_t _len)
{
  // don't fill _array and return false if mailformed string detected
  if (!haveHexPrefix(_src)) { return false; }
  
  // skip prefix
  _src += 2;
  // for all bytes do...
  while (_len--)  {
     *_dst = (htod(*_src) << 4);
     _src++;
     *_dst += htod(*_src);
      _src++; _dst++;
  };
  return true;
}

/*****************************************************************************************************************************
*
*   Compute a Dallas Semiconductor 8 bit CRC directly. This is much slower, but much smaller, than the lookup table.
*
*   This function placed here to aviod compilation error when OneWire library is not #included
*
*****************************************************************************************************************************/
uint8_t dallas_crc8(const uint8_t *_src, uint8_t _len)
{
  uint8_t crc = 0;

  while (_len) {
    _len--;
    uint8_t inbyte = *_src++;
    for (uint8_t i = 8; i; i--) {
 	uint8_t mix = (crc ^ inbyte) & 0x01;
	crc >>= 1;
	if (mix) crc ^= 0x8C;
	inbyte >>= 1;
    }
  }
  return crc;
}

/*****************************************************************************************************************************
*
*  Print string stored in PROGMEM to Serial 
*
*****************************************************************************************************************************/
void SerialPrint_P (const char *_src) {
  char currChar;
  while ((currChar = pgm_read_byte(_src++)) != 0)
  Serial.print(currChar);
}

/*****************************************************************************************************************************
*
*  Print string stored in PROGMEM to Serial + Line Feed
*
*****************************************************************************************************************************/
void SerialPrintln_P (const char *_src) {
  SerialPrint_P(_src);
  Serial.println();
}


/*****************************************************************************************************************************
*
*  Print array to Serial as MAC or IP or other string with sign-separated parts
*
*****************************************************************************************************************************/
void printArray(uint8_t *_src, uint8_t _len, const uint8_t _type)
{
  char separator;
  uint8_t format;

  switch (_type) {
    case DBG_PRINT_AS_MAC:
      format = HEX;
      separator = ':';
      break;
    case DBG_PRINT_AS_IP:
    default:
      format = DEC;
      separator = '.';
   }     

  while (_len--) {
    Serial.print(*_src, format);
    if (1 <= _len) Serial.print(separator);
    _src++;
  }
  Serial.println();
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


