// Config & common included files
#include "sys_includes.h"

#include <avr/boot.h>
#include <util/atomic.h>
#include "service.h"
#include "system.h"


/*****************************************************************************************************************************
*
*  Read bytes from the MCU's Signature Row to buffer 
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void getBootSignatureAsHexString(char* _dst, const uint8_t _startByte, uint8_t _len, const uint8_t _step) {
  // Interrupts must be disabled before boot_signature_byte_get will be called to avoid code execution crush
  uint8_t i = _startByte;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    while (_len) {
       uint8_t currByte = boot_signature_byte_get(i);
       // ((...)) is calculate expression before calling dtoh()
       *_dst++ = dtoh((uint8_t) (0x0F <= currByte) ? (currByte >> 4) : 0x00 );
       *_dst++ = dtoh((uint8_t) (currByte & 0x0F));
       i += _step;
       _len--;
    }
  }
  // finalize string
  *_dst = '\0';
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
void startTimerOne() { TCCR1B = _BV(CS12) | _BV(CS10); }

/*****************************************************************************************************************************
*
*  Handle Timer1 interrupt 
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
ISR(TIMER1_COMPA_vect)
{
  // Gather internal metric
  gatherSystemMetrics();
  // Let's count from the begin
  TCNT1 = 0;
}

/*****************************************************************************************************************************
*
*  Stop Timer1
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void stopTimerOne() { TCCR1B = 0; }

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

  //extern volatile int32_t sysMetrics[];
  //sysMetrics[IDX_METRIC_SYS_RAM_FREE] = (int32_t) getRamFree(); 
  // Correct sys.ram.freemin metric when FreeMem just taken
  //if (sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] > sysMetrics[IDX_METRIC_SYS_RAM_FREE]) {
  //   sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] = sysMetrics[IDX_METRIC_SYS_RAM_FREE]; 
  extern volatile sysmetrics_t sysMetrics;
  sysMetrics.sysRamFree = getRamFree(); 
  // Correct sys.ram.freemin metric when FreeMem just taken
  if (sysMetrics.sysRamFreeMin > sysMetrics.sysRamFree) {
     sysMetrics.sysRamFreeMin = sysMetrics.sysRamFree; 
  }

}
     

