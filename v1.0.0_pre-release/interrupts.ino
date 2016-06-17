#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE

// Basic configuration => EXTERNAL_NUM_INTERRUPTS == 3
void handleINT0() { extInterrupt[INT0].count++; }
void handleINT1() { extInterrupt[INT1].count++; }

// AVR_ATmega1284, AVR_ATmega1284P, AVR_ATmega644, AVR_ATmega644A, AVR_ATmega644P, AVR_ATmega644PA => EXTERNAL_NUM_INTERRUPTS == 3
#if (EXTERNAL_NUM_INTERRUPTS > 2)
  void handleINT2() { extInterrupt[INT2].count++; }
#endif

// AVR_ATmega32U4 => EXTERNAL_NUM_INTERRUPTS == 5
#if (EXTERNAL_NUM_INTERRUPTS > 3)
  void handleINT3() { extInterrupt[INT3].count++; }
  void handleINT4() { extInterrupt[INT4].count++; }
#endif

// AVR_ATmega1280, AVR_ATmega2560, AVR_ATmega128RFA1, AVR_ATmega256RFR2 => EXTERNAL_NUM_INTERRUPTS == 8
#if (EXTERNAL_NUM_INTERRUPTS > 5)
  void handleINT5() { extInterrupt[INT5].count++; }
  void handleINT6() { extInterrupt[INT6].count++; }
  void handleINT7() { extInterrupt[INT7].count++; }
#endif
#endif
