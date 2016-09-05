#include <avr/boot.h>

/* ****************************************************************************************************************************
*
*  Write MCU ID to buffer 
*  http://www.avrfreaks.net/forum/unique-id-atmega328pb
*
**************************************************************************************************************************** */
void getMCUID (char* _dataBuffer) {
 // *_dataBuffer++ = '0'; 
 // *_dataBuffer++ = 'x';
  // Interrupts must be disabled before boot_signature_byte_get will be called to avoid code crush
  noInterrupts();
  // Read 14..24 bytes from boot signature
  for (uint8_t i = 14; i < 24; i ++) {
    uint8_t currByte = boot_signature_byte_get(i);
    // strange (()) need to evaluate dec value for #define'd inlie function
    // Must be moved to dtoh()
    *_dataBuffer++ = dtoh(((0x0F <= currByte) ? (currByte >> 4) : 0));
    *_dataBuffer++ = dtoh((currByte & 0x0F));
    }
  interrupts();
  // finalize string
  *_dataBuffer = '\0';
}

/* ****************************************************************************************************************************
*
*  Return "Free" memory size
*
**************************************************************************************************************************** */
uint32_t getRamFree(void) {
  extern uint16_t __heap_start, *__brkval;
  uint16_t v;
  return (uint32_t) &v - (__brkval == 0 ? (uint32_t) &__heap_start : (uint32_t) __brkval);
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
  // Set prescaler
  TCCR1B = _BV(CS12) | _BV(CS10);
  // Allow to do interrupt on counter overflow
  TIMSK1 |= _BV(OCIE1A); 
  // Set boundary
  // It is good practice to set OCR1A after you configure the rest of the timer
  OCR1A = (F_CPU / 1024) * (_milliseconds/1000);
  return true; 
}

/* ****************************************************************************************************************************
*
*   Gathering internal metrics and save its to global array
*
**************************************************************************************************************************** */
void gatherSystemMetrics(){
  // repeat measuring on next round
  if (skipMetricGathering) { return; }
  // = IDX_METRICS_FIRST_CRONNED to skip "gathering" uncronned metric (IDX_METRIC_SYS_CMD_COUNT and so) 
  static uint8_t metricIdx = IDX_METRICS_FIRST_CRONNED;
  // Gather only one metric at once to leave CPU time to other important procedures

#ifdef FEATURE_DEBUG_COMMANDS_ENABLE
  switch (metricIdx) {
    case IDX_METRIC_SYS_VCC:
    //case IDX_METRIC_SYS_VCCMIN:
    //case IDX_METRIC_SYS_VCCMAX:
        sysMetrics[IDX_METRIC_SYS_VCC] = getADCVoltage(ANALOG_CHAN_VBG);
        correctVCCMetrics(sysMetrics[IDX_METRIC_SYS_VCC]);
      metricIdx += 2; // Three metrics taken at once  
      break;
    case IDX_METRIC_SYS_RAM_FREE:
    //case IDX_METRIC_SYS_RAM_FREEMIN:
      sysMetrics[IDX_METRIC_SYS_RAM_FREE] = (int32_t) getRamFree(); 
      correctMemoryMetrics(sysMetrics[IDX_METRIC_SYS_RAM_FREE]);
      metricIdx += 1; // Two metrics taken at once  
      break;
    default:
      ;
  }
  metricIdx++;
  // = 1 to skip "gathering" IDX_METRIC_SYS_CMD_COUNT 
  if (IDX_METRICS_MAX <= metricIdx) { metricIdx = IDX_METRICS_FIRST_CRONNED; }
#endif
}

/* ****************************************************************************************************************************
*
*   Correct sys.ram.freemin metric when FreeMem just taken
*
**************************************************************************************************************************** */
void correctMemoryMetrics(uint32_t _currMemFree) {
  if (sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] > _currMemFree) { sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] = _currMemFree; }
}           
      

/* ****************************************************************************************************************************
*
*   Correct sys.vccmin/sys.vccmax metrics when VCC just taken
*
**************************************************************************************************************************** */
void correctVCCMetrics(uint32_t _currVCC) {
  if (sysMetrics[IDX_METRIC_SYS_VCCMIN] > _currVCC) { sysMetrics[IDX_METRIC_SYS_VCCMIN] = _currVCC; }
  if (sysMetrics[IDX_METRIC_SYS_VCCMAX] < _currVCC) { sysMetrics[IDX_METRIC_SYS_VCCMAX] = _currVCC; }
}

