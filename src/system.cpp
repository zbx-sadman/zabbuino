// Config & common included files
#include "sys_includes.h"

#if defined(ARDUINO_ARCH_AVR)
    #include <avr/boot.h>
    #include <util/atomic.h>
#endif

#include "service.h"
#include "system.h"
#include "adc.h"
#include "wrap_network.h"

/*****************************************************************************************************************************
*
*  Reset the system
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void systemReboot() {
  // The reason why using the watchdog timer or RST_SWRST_bm is preferable over jumping to the reset vector, is that when the watchdog or RST_SWRST_bm resets the AVR,
  // the registers will be reset to their known, default settings. Whereas jumping to the reset vector will leave the registers in their previous state, which is
  // generally not a good idea. http://www.atmel.com/webdoc/avrlibcreferencemanual/FAQ_1faq_softreset.html
  //
  //  ...but some Arduino's bootloaders going to "crazy loopboot" when WTD is enable on reset
  //
  // Watchdog deactivation
  __WATCHDOG( wdt_disable(); )
#if defined(ARDUINO_ARCH_AVR)
  asm volatile ("jmp 0");
#elif (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))
  ESP.restart();
#endif
}

/*****************************************************************************************************************************
*
*  Read bytes from the MCU's Signature Row and put its to array
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void getMcuId(uint8_t* _dst) {
#if defined(ARDUINO_ARCH_AVR)
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // Read 10 bytes with step 1 (0x0E..0x17) of the signature row <= http://www.avrfreaks.net/forum/unique-id-atmega328pb
    for (uint8_t i = constMcuIdStartAddress; (constMcuIdStartAddress + constMcuIdSize) > i; i++) {
       *_dst = boot_signature_byte_get(i);
       _dst++;  
    }
  }
#elif defined(ARDUINO_ARCH_ESP8266)
    uint32_t chipId = ESP.getChipId();
    uint8_t  *ptrChipID = (uint8_t*) &chipId;
    *_dst++ = ptrChipID[0x03];
    *_dst++ = ptrChipID[0x02];
    *_dst++ = ptrChipID[0x01];
    *_dst   = ptrChipID[0x00];
#elif defined(ARDUINO_ARCH_ESP32)
    uint64_t chipId = ESP.getEfuseMac();
    uint8_t  *ptrChipID = (uint8_t*) &chipId;
    for (int8_t i = constMcuIdSize; 0x00 <= i; i--) {
       *_dst++ = ptrChipID[i];
    }
#endif
}

int32_t getMcuFreq() {
#if defined(ARDUINO_ARCH_AVR)
    return F_CPU;
#elif (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))
    return ESP.getCpuFreqMHz();
#endif
}
/*****************************************************************************************************************************
*
*  Read bytes from the MCU's Signature Row and put its to array
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void getMcuModel(uint8_t* _dst) {
#if defined(ARDUINO_ARCH_AVR)
  // Read 3 bytes with step 2 (0x00, 0x02, 0x04) of the signature row <= http://www.avrfreaks.net/forum/device-signatures
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    *_dst++ = boot_signature_byte_get(0x00);
    *_dst++ = boot_signature_byte_get(0x02);
    *_dst   = boot_signature_byte_get(0x04);
  }
#elif defined(ARDUINO_ARCH_ESP8266)
    // Where are placed model ID?
  __SUPPRESS_WARNING_UNUSED(_dst);
#elif defined(ARDUINO_ARCH_ESP32)
    // Where are placed model ID?
  __SUPPRESS_WARNING_UNUSED(_dst);
#endif
}

// !!! No any warrianty when rom_phy_get_vdd33 used
#if defined(ARDUINO_ARCH_ESP32)
//extern "C" int rom_phy_get_vdd33();
#endif

int32_t getMcuVoltage() {
  int32_t result;
#if defined(ARDUINO_ARCH_AVR)
  result = getADCVoltage(ANALOG_CHAN_VBG);
#elif defined(ARDUINO_ARCH_ESP8266)
  result = ESP.getVcc();
#elif defined(ARDUINO_ARCH_ESP32)
  // !!! No any warrianty when rom_phy_get_vdd33 used
  result = 0x00;
  //result = rom_phy_get_vdd33();
#endif
  // VCC may be bigger than max or smaller than min.
  // To avoid wrong results and graphs in monitoring system - correct min/max metrics
  correctVCCMetrics(result);
  return result;
}


int8_t getSystemAllInfo(char* _dst, const uint16_t _dstSize) {
  extern volatile sysmetrics_t sysMetrics;
  uint32_t sysVcc = getMcuVoltage();
  uint32_t sysUptime  = ((uint32_t) millisRollover() * UINT32_MAX + millis()) / 1000UL;

#if defined(ARDUINO_ARCH_AVR)
  snprintf_P(_dst, _dstSize, PSTR("{\"upTime\":%lu,\"sysRamFree\":%lu,\"sysRamFreeMin\":%lu,\"sysVcc\":%lu,\"sysVccMin\":%lu,\"sysVccMax\":%lu,\"sysCmdCount\":%lu,\"netPHYReinits\":%lu}"), 
             sysUptime, sysMetrics.sysRamFree,  sysMetrics.sysRamFreeMin, sysVcc, sysMetrics.sysVCCMin, sysMetrics.sysVCCMax, 
             sysMetrics.sysCmdCount, sysMetrics.netPHYReinits);  
#elif (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))
  snprintf_P(_dst, _dstSize, PSTR("{\"upTime\":%lu,\"sysRamFree\":%lu,\"sysRamFreeMin\":%lu,\"sysVcc\":%lu,\"sysVccMin\":%lu,\"sysVccMax\":%lu,\"sysCmdCount\":%lu,\"lwipVersion\":%u.%u,\"netPHYReinits\":%lu, \"wifiRssi\":%d}"), 
             sysUptime, sysMetrics.sysRamFree,  sysMetrics.sysRamFreeMin, sysVcc, sysMetrics.sysVCCMin, sysMetrics.sysVCCMax, 
             sysMetrics.sysCmdCount, (uint8_t)LWIP_VERSION_MAJOR, (uint8_t)LWIP_VERSION_MINOR, sysMetrics.netPHYReinits, NetworkTransport.RSSI());
#endif

  return RESULT_IS_BUFFERED;
}

/*****************************************************************************************************************************
*
*  Init Timer1 
*
*   Returns: 
*     - always true at this time
*
*****************************************************************************************************************************/
uint8_t initTimerOne(const uint32_t _milliseconds) 
{
#if defined(ARDUINO_ARCH_AVR)
  TCCR1B = _BV(CS12) | _BV(CS10); 
  // Don't allow more that 5 sec to avoid overflow on 16Mhz with prescaler 1024 
  if ((1000 > _milliseconds) && (5000 < _milliseconds)) { return false; }
  // Clear control register A 
  TCCR1A = 0;                 
  // Set  prescaler
  TCCR1B =  _BV(CS12) | _BV(CS10);
  // Allow to do interrupt on counter overflow
  TIMSK1 |= _BV(OCIE1A); 
  // Set boundary
  // It is good practice to set OCR1A after you configure the rest of the timer
  // Take care with OCR1A writing: http://www.atmel.com/webdoc/avrlibcreferencemanual/FAQ_1faq_16bitio.html
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { OCR1A = (F_CPU / 1024) * (_milliseconds/1000); }
#elif (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))
  __SUPPRESS_WARNING_UNUSED(_milliseconds);
#endif
  return true; 
}

/*****************************************************************************************************************************
*
*  Start Timer1
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void startTimerOne() {
#if defined(ARDUINO_ARCH_AVR)
    TCCR1B = _BV(CS12) | _BV(CS10); 
#elif (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))
#endif
}

/*****************************************************************************************************************************
*
*  Handle Timer1 interrupt 
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
#if defined(ARDUINO_ARCH_AVR)
ISR(TIMER1_COMPA_vect)
{
  // Gather internal metric
  gatherSystemMetrics();
  // Let's count from the begin
  TCNT1 = 0x00;
}
#endif
/*****************************************************************************************************************************
*
*  Stop Timer1
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void stopTimerOne() { 
#if defined(ARDUINO_ARCH_AVR)
    TCCR1B = 0; 
#elif (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))
#endif
}

/*****************************************************************************************************************************
*
*  Gather internal metrics and save it to global variable
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void gatherSystemMetrics(){
  // Global variable from the outside
  extern volatile sysmetrics_t sysMetrics;
  sysMetrics.sysRamFree = getRamFree(); 
  // Correct sys.ram.freemin metric when FreeMem just taken
  if (sysMetrics.sysRamFreeMin > sysMetrics.sysRamFree) {
     sysMetrics.sysRamFreeMin = sysMetrics.sysRamFree; 
  }

}
     

