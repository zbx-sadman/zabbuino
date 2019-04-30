#pragma once

uint8_t flushStreamRXBuffer(Stream*, const uint32_t, const uint8_t);
uint8_t factoryReset(netconfig_t&);

/*****************************************************************************************************************************
*
*   Return number of millis() rollovers every UINT32_MAX ms (~50days)
*
*****************************************************************************************************************************/
uint16_t millisRollover(void);

/*****************************************************************************************************************************
*
*   Return system uptime (seconds)
*
*****************************************************************************************************************************/
uint32_t uptime(void);

/*****************************************************************************************************************************
*
*   Set default values of network configuration
*
*****************************************************************************************************************************/
void setConfigDefaults(netconfig_t&);

/*****************************************************************************************************************************
*
*   Convert int32_t _number to char[]  with decimal point on _num_after_dot position 
*   _number / (10 * _num_after_dot position) => char[]
*
*****************************************************************************************************************************/
void ltoaf(const int32_t _number, char *_dst, const uint8_t _num_after_dot);

/*****************************************************************************************************************************
*
*   Convert _Qm.n_ float number (int64_t) to char[] 
*
*****************************************************************************************************************************/
void qtoaf(const int64_t _number, char *_dst, uint8_t _fracBits);

/*****************************************************************************************************************************
*
*  Convert _len chars (exclude 0x prefix) of hex string to byte array
*
*****************************************************************************************************************************/
int16_t hstoba(uint8_t*, const char*);

/*****************************************************************************************************************************
*
*   Compute a Dallas Semiconductor 8 bit CRC directly. This is much slower, but much smaller, than the lookup table.
*
*   This function placed here to aviod compilation error when OneWire library is not #included
*
*****************************************************************************************************************************/
uint8_t dallas_crc8(uint8_t *addr, uint8_t len);

/*****************************************************************************************************************************
*
*  Print string stored in PROGMEM to Serial 
*
*****************************************************************************************************************************/
//extern void SerialPrint_P (const char *_src);

/*****************************************************************************************************************************
*
*  Print string stored in PROGMEM to Serial + Line Feed
*
*****************************************************************************************************************************/
//extern void SerialPrintln_P (const char *_src);

/*****************************************************************************************************************************
*
*  Print array to Serial as MAC or IP or other string with sign-separated parts
*
*****************************************************************************************************************************/
void printArray(uint8_t* _src, const uint8_t _len, Stream& _stream, const uint8_t _type);

void blinkMore(const uint8_t _times, const uint16_t _onTime, const uint16_t _offTime);

/*****************************************************************************************************************************
*
*  
*
*****************************************************************************************************************************/
//uint8_t validateNetworkAddress(const NetworkAddress);
uint8_t strToNetworkAddress(const char* _src, uint32_t& _dstAddress);
uint8_t analyzeStream(char _charFromClient, char* _dst, const uint8_t doReInit, packetInfo_t& _packetInfo);

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         INLINE AND "DEFINE" FUNCTIONS SECTION 
*/

// Use #define instead inline directive may be wrong, but save progspace


// HEX char to number
//#define htod(_hex) ( (_hex >= 'a' && _hex <= 'f') ? (10 + _hex - 'a') : ( (_hex >= '0' && _hex <= '9') ? (_hex - '0') : 0) )
//inline uint8_t htod(const char    _hex) { return (_hex >= 'a' && _hex <= 'f') ? (10 + _hex - 'a') : ( (_hex >= '0' && _hex <= '9') ? (_hex - '0') : 0); } 
//inline uint8_t htod(const uint8_t _hex) { return (_hex >= 'a' && _hex <= 'f') ? (10 + _hex - 'a') : ( (_hex >= '0' && _hex <= '9') ? (_hex - '0') : 0); } 

// Convert dec number to hex char
//#define dtoh(_dec) ( (_dec > 9) ? ('A' - 10 + _dec) : ('0' + _dec) )
inline char dtoh(const uint8_t  _dec) { return (_dec > 9) ? ('A' - 10 + _dec) : ('0' + _dec); } 
//inline char dtoh(const uint16_t _dec) { return dtoh((uint8_t) _dec); } 

// if _source have hex prefix - return true
//#define haveHexPrefix(_src) ( (_src[0] == '0' && _src[1] == 'x') )
inline uint8_t haveHexPrefix(const char*    _src) { return (_src[0] == '0' && _src[1] == 'x'); } 
inline uint8_t haveHexPrefix(const uint8_t* _src) { return haveHexPrefix((const char*) _src ); } 


inline uint8_t htod(const char   _hex) {
  return (_hex >= 'a' && _hex <= 'f') ? (10 + _hex - 'a') : ( (_hex >= '0' && _hex <= '9') ? (_hex - '0') : 0);
}
inline uint8_t htod(const uint8_t _hex) { return htod((const char) _hex); }
//inline uint8_t htod(const int16_t _hex) { return htod((const char) _hex); }


#define arraySize(_array) ( sizeof(_array) / sizeof(*(_array)) )

/* ****************************************************************************************************************************
*
*  Return "Free" memory size
*
**************************************************************************************************************************** */
//inline __attribute__((always_inline)) uint32_t getRamFree(void) {
inline uint32_t getRamFree(void) {
  extern uint16_t __heap_start, *__brkval;
  uint16_t v;
  return (uint32_t) &v - (__brkval == 0 ? (uint32_t) &__heap_start : (uint32_t) __brkval);
}

/* ****************************************************************************************************************************
*
*   Correct sys.vccmin/sys.vccmax metrics when VCC just taken
*
**************************************************************************************************************************** */
// __attribute__((always_inline)) 
inline void correctVCCMetrics(uint32_t _currVCC) {
  // Global variable from outside
  extern volatile sysmetrics_t sysMetrics;
  if (sysMetrics.sysVCCMin > _currVCC) { sysMetrics.sysVCCMin = _currVCC; }
  if (sysMetrics.sysVCCMax < _currVCC) { sysMetrics.sysVCCMax = _currVCC; }
}

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

// *** Helpers ***

#define arraySize(_array) ( sizeof(_array) / sizeof(*(_array)) )
#define FSH_P(p) (reinterpret_cast<const __FlashStringHelper *>(p))


inline uint32_t octetsToIpAddress(const uint8_t octets[4]) {
  return (((uint32_t) octets[3] << 24) | ((uint32_t) octets[2] << 16) | ((uint16_t) octets[1] << 8) |  octets[0]);
}


#ifdef FEATURE_USER_FUNCTION_PROCESSING
 #define __USER_FUNCTION(_code) _code
#else
 #define __USER_FUNCTION(_code) /* blank */
#endif 

#ifdef FEATURE_WATCHDOG_ENABLE
 #define __WATCHDOG(_code) _code
#else
 #define __WATCHDOG(_code) /* blank */
#endif 
