#include "system.h"
#include <avr/boot.h>
#include <util/atomic.h>

/* ****************************************************************************************************************************
*
*  Handle Timer1 interrupt 
*
**************************************************************************************************************************** */
ISR(TIMER1_COMPA_vect)
{
  //Serial.print("Timer on: "); Serial.print(millis()); Serial.println();
  // Gather internal metric
  gatherSystemMetrics();
  // Let's count from the begin
  TCNT1 = 0;
}

/* ****************************************************************************************************************************
*
*  Read bytes from the MCU signature area to buffer 
*  
**************************************************************************************************************************** */
void getBootSignatureBytes(char* _dst, uint8_t _startByte, uint8_t _len) {
  // Interrupts must be disabled before boot_signature_byte_get will be called to avoid code execution crush
  //noInterrupts();
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = _startByte; i < (_startByte+_len); i++) {
       uint8_t currByte = boot_signature_byte_get(i);
       // ((...)) is calculate expression before calling dtoh()
       *_dst++ = dtoh(((0x0F <= currByte) ? (currByte >> 4) : 0));
       *_dst++ = dtoh((currByte & 0x0F));
    }
  }
  //interrupts();
  // finalize string
  *_dst = '\0';
}

/* ****************************************************************************************************************************
*
*  Timer1 initialization 
*
**************************************************************************************************************************** */
uint8_t initTimerOne(const uint16_t _milliseconds) 
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
  OCR1A = (F_CPU / 1024) * (_milliseconds/1000);
  return true; 
}

void stopTimerOne() { TCCR1B = 0; }
void startTimerOne() { TCCR1B = _BV(CS12) | _BV(CS10); }

/* ****************************************************************************************************************************
*
*   Gathering internal metrics and save its to global array
*
**************************************************************************************************************************** */
void inline gatherSystemMetrics(){
  // Global variable from outside
  extern int32_t *sysMetrics;
#ifdef FEATURE_DEBUG_COMMANDS_ENABLE
      sysMetrics[IDX_METRIC_SYS_RAM_FREE] = (int32_t) getRamFree(); 
      // Correct sys.ram.freemin metric when FreeMem just taken
      if (sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] > sysMetrics[IDX_METRIC_SYS_RAM_FREE]) {
          sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] = sysMetrics[IDX_METRIC_SYS_RAM_FREE]; 

      }
#endif
}
     

