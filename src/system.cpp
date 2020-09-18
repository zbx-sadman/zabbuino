// Config & common included files
#include "sys_includes.h"

#if defined(ARDUINO_ARCH_AVR)
    #include <avr/boot.h>
    #include <util/atomic.h>
#endif

#include "service.h"
#include "system.h"

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
#elif defined(ARDUINO_ARCH_ESP8266)
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
    *_dst++ = ptrChipID[0x00];
#endif
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
  // Don't allow more that 5 sec to avoid overflow on 16Mhz with prescaler 1024 
  //if ((1000 > _milliseconds) && (5000 < _milliseconds)) { return false; }
  // Clear control register A 
  //TCCR1A = 0;                 
  // Set  prescaler
  //TCCR1B =  _BV(CS12) | _BV(CS10);
  // Allow to do interrupt on counter overflow
  //TIMSK1 |= _BV(OCIE1A); 
  // Set boundary
  // It is good practice to set OCR1A after you configure the rest of the timer
  // Take care with OCR1A writing: http://www.atmel.com/webdoc/avrlibcreferencemanual/FAQ_1faq_16bitio.html
  //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { OCR1A = (F_CPU / 1024) * (_milliseconds/1000); }
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
// TCCR1B = _BV(CS12) | _BV(CS10); 
}

/*****************************************************************************************************************************
*
*  Handle Timer1 interrupt 
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
/*
ISR(TIMER1_COMPA_vect)
{
  // Gather internal metric
  gatherSystemMetrics();
  // Let's count from the begin
  TCNT1 = 0;
}
*/
/*****************************************************************************************************************************
*
*  Stop Timer1
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void stopTimerOne() { 
//TCCR1B = 0; 
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
     

