#ifndef ZabbuinoINTERRUPTS_h
#define ZabbuinoINTERRUPTS_h

#include <Arduino.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include "defaults.h"
#include "../zabbuino.h"
#include "system.h"


/*
 macro CASE_INT_N(INT0) transformed to code:

   case INT0:
     interruptHandler = handleExtINT0;
     break;

*/
#define CASE_INT_N_FOR_EXTINT(_interrupt) \
    case _interrupt: \
       interruptHandler = handleExt##_interrupt;\
    break;



/*

 macro HANDLE_INT_N(INT0) transformed to code:

    void handleExtINT0() { extern extInterrupt_t* extInterrupt;
                        extInterrupt[INT0].count++; 
}
*/

#define HANDLE_INT_N_FOR_EXTINT(_interrupt) \
   void handleExt##_interrupt() { extern extInterrupt_t *extInterrupt; extInterrupt[_interrupt].count++; }


#define HANDLE_INT_N_FOR_INCENC(_interrupt) \
   void handleIncEnc##_interrupt() { extern extInterrupt_t *extInterrupt; volatile static uint8_t stateTerminalA = 0, statePrevTerminalA = 0;\
         delayMicroseconds(ENCODER_STABILIZATION_DELAY);\
         stateTerminalA = *extInterrupt[_interrupt].encTerminalAPIR & extInterrupt[_interrupt].encTerminalAPinBit;\
         if ((!stateTerminalA) && (statePrevTerminalA)) { \
           if (*extInterrupt[_interrupt].encTerminalBPIR & extInterrupt[_interrupt].encTerminalBPinBit) \
              { extInterrupt[_interrupt].value++; } else { extInterrupt[_interrupt].value--; } \
         } statePrevTerminalA = stateTerminalA; }


#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE

  int8_t manageExtInt(uint32_t* _dst, uint8_t _pin, uint8_t _mode);


 
// Basic configuration => EXTERNAL_NUM_INTERRUPTS == 3
  void handleExtINT0();
  void handleExtINT1();


  // AVR_ATmega1284, AVR_ATmega1284P, AVR_ATmega644, AVR_ATmega644A, AVR_ATmega644P, AVR_ATmega644PA => EXTERNAL_NUM_INTERRUPTS == 3
  #if (EXTERNAL_NUM_INTERRUPTS > 2)
      void handleExtINT2();
  #endif

  // AVR_ATmega32U4 => EXTERNAL_NUM_INTERRUPTS == 5
  #if (EXTERNAL_NUM_INTERRUPTS > 3)
      void handleExtINT3();
      void handleExtINT4();
  #endif

  #if (EXTERNAL_NUM_INTERRUPTS > 5)
      // AVR_ATmega1280, AVR_ATmega2560, AVR_ATmega128RFA1, AVR_ATmega256RFR2 => EXTERNAL_NUM_INTERRUPTS == 8
      void handleExtINT5();
      void handleExtINT6();
      void handleExtINT7();
#endif

#endif // FEATURE_EXTERNAL_INTERRUPT_ENABLE


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                                 ENCODER INTERRUPTS HANDLING SECTION
*/

#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE
  int8_t manageIncEnc(int32_t* _dst, uint8_t const _terminalAPin, uint8_t const _terminalBPin, int32_t const _initialValue);

  void handleIncEncINT0();
  void handleIncEncINT1();
#endif // FEATURE_INCREMENTAL_ENCODER_ENABLE

#endif