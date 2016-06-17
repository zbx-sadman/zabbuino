ISR(TIMER1_COMPA_vect)
{
  // Gather internal metric
  gatherMetrics();
  // Let's count from the begin
  TCNT1 = 0;
}

uint8_t timerOneInit(uint16_t _milliseconds) 
{
  // Don't allow no more 5 sec to avoid overflow on 16Mhz with prescaler 1024 
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

