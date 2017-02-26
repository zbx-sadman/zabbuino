#ifndef _ZABBUINO_SERVICE_H_
#define _ZABBUINO_SERVICE_H_

#include <Arduino.h>
#include <avr/boot.h>
#include "../basic.h"
#include "tune.h"
#include "structs.h"
#include "system.h"

/*****************************************************************************************************************************
*
*   Set default values of network configuration
*
*****************************************************************************************************************************/
void setConfigDefaults(netconfig_t *_configStruct);

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
uint8_t hstoba(uint8_t *_dst, const char* _src, uint8_t _len);

/*****************************************************************************************************************************
*
*   Compute a Dallas Semiconductor 8 bit CRC directly. This is much slower, but much smaller, than the lookup table.
*
*   This function placed here to aviod compilation error when OneWire library is not #included
*
*****************************************************************************************************************************/
uint8_t dallas_crc8(const uint8_t *addr, uint8_t len);

/*****************************************************************************************************************************
*
*  Print string stored in PROGMEM to Serial 
*
*****************************************************************************************************************************/
extern void SerialPrint_P (const char *_src);

/*****************************************************************************************************************************
*
*  Print string stored in PROGMEM to Serial + Line Feed
*
*****************************************************************************************************************************/
extern void SerialPrintln_P (const char *_src);

/*****************************************************************************************************************************
*
*  Print array to Serial as MAC or IP or other string with sign-separated parts
*
*****************************************************************************************************************************/
void printArray(uint8_t *_src, uint8_t _len, const uint8_t _type);

void blinkMore(const uint8_t _times, const uint16_t _onTime, const uint16_t _offTime);

/*****************************************************************************************************************************
*
*  
*
*****************************************************************************************************************************/
uint8_t validateNetworkAddress(const NetworkAddress);
uint8_t strToNetworkAddress(const char*, NetworkAddress*);
uint8_t analyzeStream(char, char*, char**, uint8_t);

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         INLINE AND "DEFINE" FUNCTIONS SECTION 
*/

// Use #define instead inline directive may be wrong, but save progspace


// HEX char to number
//#define htod(_hex) ( (_hex >= 'a' && _hex <= 'f') ? (10 + _hex - 'a') : ( (_hex >= '0' && _hex <= '9') ? (_hex - '0') : 0) )
inline uint8_t htod(const char _hex) { return (_hex >= 'a' && _hex <= 'f') ? (10 + _hex - 'a') : ( (_hex >= '0' && _hex <= '9') ? (_hex - '0') : 0); } 

// Convert dec number to hex char
//#define dtoh(_dec) ( (_dec > 9) ? ('A' - 10 + _dec) : ('0' + _dec) )
inline char dtoh(const uint8_t _dec) { return (_dec > 9) ? ('A' - 10 + _dec) : ('0' + _dec); } 

// if _source have hex prefix - return true
//#define haveHexPrefix(_src) ( (_src[0] == '0' && _src[1] == 'x') )
inline uint8_t haveHexPrefix(const char *_src) { return (_src[0] == '0' && _src[1] == 'x'); } 

#define arraySize(_array) ( sizeof(_array) / sizeof(*(_array)) )

/* ****************************************************************************************************************************
*
*  Return "Free" memory size
*
**************************************************************************************************************************** */
inline __attribute__((always_inline)) uint32_t getRamFree(void) {
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
  extern volatile int32_t sysMetrics[];
  if ((uint32_t) sysMetrics[IDX_METRIC_SYS_VCCMIN] > _currVCC) { sysMetrics[IDX_METRIC_SYS_VCCMIN] = _currVCC; }
  if ((uint32_t) sysMetrics[IDX_METRIC_SYS_VCCMAX] < _currVCC) { sysMetrics[IDX_METRIC_SYS_VCCMAX] = _currVCC; }
}

#ifdef FEATURE_DEBUG_TO_SERIAL_HIGH
    #define DTSL(x) x
    #define DTSM(x) x
    #define DTSH(x) x
#else 
    #ifdef FEATURE_DEBUG_TO_SERIAL_MIDDLE
        #define DTSL(x) x
        #define DTSM(x) x
        #define DTSH(X) /* blank */
    #else 
        #ifdef FEATURE_DEBUG_TO_SERIAL_LOW
            #define DTSL(x) x
            #define DTSM(X) /* blank */
            #define DTSH(X) /* blank */
        #else 
            #define DTSL(X) /* blank */
            #define DTSM(X) /* blank */
            #define DTSH(X) /* blank */
        #endif 
    #endif 
#endif 


#ifdef FEATURE_DEBUG_TO_SERIAL_DEV
 #define DTSD(x) x
#else
 #define DTSD(X) /* blank */
#endif 


#ifdef FEATURE_NET_DEBUG_TO_SERIAL
 #define NDTS(x) x
#else
 #define NDTS(X) /* blank */
#endif 


#endif // #ifndef _ZABBUINO_SERVICE_H_