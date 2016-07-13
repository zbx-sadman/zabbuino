/* ****************************************************************************************************************************
*
*  Return "Free" memory size
*
**************************************************************************************************************************** */
uint32_t ramFree(void) {
  extern uint16_t __heap_start, *__brkval;
  uint16_t v;
  return (uint32_t) &v - (__brkval == 0 ? (uint32_t) &__heap_start : (uint32_t) __brkval);
}


/* ****************************************************************************************************************************
*
*   
*
**************************************************************************************************************************** */
ISR(TIMER1_COMPA_vect)
{
  // Gather internal metric
  gatherMetrics();
  // Let's count from the begin
  TCNT1 = 0;
}

/* ****************************************************************************************************************************
*
*   
*
**************************************************************************************************************************** */
uint8_t timerOneInit(uint16_t _milliseconds) 
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
void gatherMetrics(){
  // = IDX_METRICS_FIRST_CRONNED to skip "gathering" uncronned metric (IDX_METRIC_SYS_CMD_COUNT and so) 
  static uint8_t metricIdx = IDX_METRICS_FIRST_CRONNED;
  // Gather only one metric at once to leave CPU time to other important procedures

#ifdef FEATURE_DEBUG_COMMANDS_ENABLE
  switch (metricIdx) {
    case IDX_METRIC_SYS_VCCMIN:
    case IDX_METRIC_SYS_VCCMAX:
      correctVCCMetrics(MeasureVoltage(ANALOG_CHAN_VBG));
      metricIdx++; // Two metrics taken at once  
      break;
    case IDX_METRIC_SYS_RAM_FREE:
    case IDX_METRIC_SYS_RAM_FREEMIN:
      sysMetrics[IDX_METRIC_SYS_RAM_FREE] = (int32_t) ramFree(); 
      correctMemoryMetrics(sysMetrics[IDX_METRIC_SYS_RAM_FREE]);
      metricIdx++; // Two metrics taken at once  
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
*   Correct minmem metric when FreeMem just taken
*
**************************************************************************************************************************** */
void correctMemoryMetrics(uint32_t _currMemFree) {
  if (sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] > _currMemFree) {
     sysMetrics[IDX_METRIC_SYS_RAM_FREEMIN] = _currMemFree;
  }
}           
      

/* ****************************************************************************************************************************
*
*   Correct minvcc/maxvcc metrics when VCC just taken
*
**************************************************************************************************************************** */
void correctVCCMetrics(uint32_t _currVCC) {
  if (sysMetrics[IDX_METRIC_SYS_VCCMIN] > _currVCC) {
     sysMetrics[IDX_METRIC_SYS_VCCMIN] = _currVCC;
  }
  if (sysMetrics[IDX_METRIC_SYS_VCCMAX] < _currVCC) {
     sysMetrics[IDX_METRIC_SYS_VCCMAX] = _currVCC;
  }
}


