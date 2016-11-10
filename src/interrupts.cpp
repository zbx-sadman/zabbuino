#include "interrupts.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                      EXTERNAL INTERRUPTS HANDLING SECTION

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

/*****************************************************************************************************************************
*
*  Create interrupts handling subs from the Macro (see interrupts.h)
*
*****************************************************************************************************************************/

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
// Basic configuration => EXTERNAL_NUM_INTERRUPTS == 3
   HANDLE_INT_N_FOR_EXTINT(INT0)
   HANDLE_INT_N_FOR_EXTINT(INT1)
// AVR_ATmega1284, AVR_ATmega1284P, AVR_ATmega644, AVR_ATmega644A, AVR_ATmega644P, AVR_ATmega644PA => EXTERNAL_NUM_INTERRUPTS == 3
#if (EXTERNAL_NUM_INTERRUPTS > 2)
   HANDLE_INT_N_FOR_EXTINT(INT2)
   HANDLE_INT_N_FOR_EXTINT(INT3)
#endif

// AVR_ATmega32U4 => EXTERNAL_NUM_INTERRUPTS == 5
#if (EXTERNAL_NUM_INTERRUPTS > 3)
   HANDLE_INT_N_FOR_EXTINT(INT3)
   HANDLE_INT_N_FOR_EXTINT(INT4)
#endif

#if (EXTERNAL_NUM_INTERRUPTS > 5)
// AVR_ATmega1280, AVR_ATmega2560, AVR_ATmega128RFA1, AVR_ATmega256RFR2 => EXTERNAL_NUM_INTERRUPTS == 8
   HANDLE_INT_N_FOR_EXTINT(INT5)
   HANDLE_INT_N_FOR_EXTINT(INT6)
   HANDLE_INT_N_FOR_EXTINT(INT7)
#endif


/*****************************************************************************************************************************
*
*  Attach/detach interrupts, return counter value
*
*  Returns: 
*    - value of counter belonging to interrupt
*    - RESULT_IS_FAIL if interrupt mode is wrong of wrong pin is specified
*
*****************************************************************************************************************************/
// This function make more that just return counter...
int8_t manageExtInt(uint32_t* _dst, uint8_t _pin, uint8_t _mode) {
   // TODO: maybe need to rework code block
   extern extInterrupt_t *extInterrupt;
   int32_t result;
   voidFuncPtr interruptHandler;
   int8_t interruptNumber=digitalPinToInterrupt(_pin);
   // Interrupt number and mode is correct?
   if ((EXTERNAL_NUM_INTERRUPTS < interruptNumber) || (RISING < _mode)) { return RESULT_IS_FAIL; }
   // Interrupt mode is changed
   // Serial.println("[1] Interrupt number and mode is correct"); 
   // Serial.print("[1*] Old interrupt mode is: ");  Serial.println(extInterrupt[interruptNumber].mode); 
   // just use NOT_AN_INTERRUPT = -1 macro from Arduino.h
   if (extInterrupt[interruptNumber].mode != _mode && NOT_AN_INTERRUPT != extInterrupt[interruptNumber].mode) {
      detachInterrupt(interruptNumber);
      extInterrupt[interruptNumber].mode = NOT_AN_INTERRUPT;
   } 
   // Interrupt not attached?
   if (NOT_AN_INTERRUPT == extInterrupt[interruptNumber].mode) {
//           ethClient.println("Int is not attached");
      extInterrupt[interruptNumber].mode = _mode;
      switch (interruptNumber) {
// Basic configuration => EXTERNAL_NUM_INTERRUPTS == 3
        // CASE_INT_N_FOR_EXTINT - glue macro from interrupts.h
        CASE_INT_N_FOR_EXTINT(INT0)
        CASE_INT_N_FOR_EXTINT(INT1)
// AVR_ATmega1284, AVR_ATmega1284P, AVR_ATmega644, AVR_ATmega644A, AVR_ATmega644P, AVR_ATmega644PA => EXTERNAL_NUM_INTERRUPTS == 3
#if (EXTERNAL_NUM_INTERRUPTS > 2)
        CASE_INT_N_FOR_EXTINT(INT2)
#endif // EXTERNAL_NUM_INTERRUPTS > 2
// AVR_ATmega32U4 => EXTERNAL_NUM_INTERRUPTS == 5
#if (EXTERNAL_NUM_INTERRUPTS > 3)
        CASE_INT_N_FOR_EXTINT(INT3)
        CASE_INT_N_FOR_EXTINT(INT4)
#endif // EXTERNAL_NUM_INTERRUPTS > 3
// AVR_ATmega1280, AVR_ATmega2560, AVR_ATmega128RFA1, AVR_ATmega256RFR2 => EXTERNAL_NUM_INTERRUPTS == 8
#if (EXTERNAL_NUM_INTERRUPTS > 5)
        CASE_INT_N_FOR_EXTINT(INT5)
        CASE_INT_N_FOR_EXTINT(INT6)
        CASE_INT_N_FOR_EXTINT(INT7)
#endif // EXTERNAL_NUM_INTERRUPTS > 5
        default:
          // still not attached
          extInterrupt[interruptNumber].mode = NOT_AN_INTERRUPT;
      }  // switch (interruptNumber)


      // check again to take in account 'No interrupt choosed' case
      if (NOT_AN_INTERRUPT != extInterrupt[interruptNumber].mode) {
         // if pin still not INPUT_PULLUP - system will hang up
         pinMode(_pin, INPUT_PULLUP);
         attachInterrupt(interruptNumber, interruptHandler, _mode);
//       ethClient.println("Int reinited");
         // reinit counter
         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            extInterrupt[interruptNumber].count = 0;
         }
      }
            
   } // if (NOT_AN_INTERRUPT == extInterrupt[interruptNumber].mode)
   // Atomic block is used to get relable counter value. 
   // AVR8 read long variables with series of ASM commands and value can be changed while reading not finished
   ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      *_dst  =  (int64_t) extInterrupt[interruptNumber].count; 
   }

   return RESULT_IN_ULONGVAR;
}

#endif // FEATURE_EXTERNAL_INTERRUPT_ENABLE


/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                      ENCODER INTERRUPTS HANDLING SECTION

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE
   HANDLE_INT_N_FOR_INCENC(INT0);
   HANDLE_INT_N_FOR_INCENC(INT1);
// AVR_ATmega1284, AVR_ATmega1284P, AVR_ATmega644, AVR_ATmega644A, AVR_ATmega644P, AVR_ATmega644PA => EXTERNAL_NUM_INTERRUPTS == 3
#if (EXTERNAL_NUM_INTERRUPTS > 2)
   HANDLE_INT_N_FOR_INCENC(INT2)
   HANDLE_INT_N_FOR_INCENC(INT3)
#endif

// AVR_ATmega32U4 => EXTERNAL_NUM_INTERRUPTS == 5
#if (EXTERNAL_NUM_INTERRUPTS > 3)
   HANDLE_INT_N_FOR_INCENC(INT3)
   HANDLE_INT_N_FOR_INCENC(INT4)
#endif

#if (EXTERNAL_NUM_INTERRUPTS > 5)
// AVR_ATmega1280, AVR_ATmega2560, AVR_ATmega128RFA1, AVR_ATmega256RFR2 => EXTERNAL_NUM_INTERRUPTS == 8
   HANDLE_INT_N_FOR_INCENC(INT5)
   HANDLE_INT_N_FOR_INCENC(INT6)
   HANDLE_INT_N_FOR_INCENC(INT7)
#endif


/*****************************************************************************************************************************
*
*  Attach/detach interrupts, return encoder's variable value
*
*  Returns: 
*    - value of variable belonging to interrupt applies to the pin to which encoder's "Terminal A" connected
*    - RESULT_IS_FAIL if wrong pin is specified
*
*****************************************************************************************************************************/
int8_t manageIncEnc(int32_t* _dst, uint8_t const _terminalAPin, uint8_t const _terminalBPin, int32_t const _initialValue) {
      // TODO: maybe need to rework code block
      // 
      extern extInterrupt_t *extInterrupt;
      voidFuncPtr interruptHandler;
      int8_t interruptNumber=digitalPinToInterrupt(_terminalAPin);
      // Interrupt number is correct?
      if (EXTERNAL_NUM_INTERRUPTS < interruptNumber) {return RESULT_IS_FAIL; }

      // Interrupt mode is changed
      // just use NOT_AN_INTERRUPT = -1 macro from Arduino.h
      // Interrupt not attached?
      if (NOT_AN_INTERRUPT == extInterrupt[interruptNumber].mode) {
         extInterrupt[interruptNumber].mode = CHANGE;
      switch (interruptNumber) {
// Basic configuration => EXTERNAL_NUM_INTERRUPTS == 3
        // CASE_INT_N_FOR_EXTINT - glue macro from interrupts.h
        CASE_INT_N_FOR_INCENC(INT0)
        CASE_INT_N_FOR_INCENC(INT1)
// AVR_ATmega1284, AVR_ATmega1284P, AVR_ATmega644, AVR_ATmega644A, AVR_ATmega644P, AVR_ATmega644PA => EXTERNAL_NUM_INTERRUPTS == 3
#if (EXTERNAL_NUM_INTERRUPTS > 2)
        CASE_INT_N_FOR_INCENC(INT2)
#endif // EXTERNAL_NUM_INTERRUPTS > 2
// AVR_ATmega32U4 => EXTERNAL_NUM_INTERRUPTS == 5
#if (EXTERNAL_NUM_INTERRUPTS > 3)
        CASE_INT_N_FOR_INCENC(INT3)
        CASE_INT_N_FOR_INCENC(INT4)
#endif // EXTERNAL_NUM_INTERRUPTS > 3
// AVR_ATmega1280, AVR_ATmega2560, AVR_ATmega128RFA1, AVR_ATmega256RFR2 => EXTERNAL_NUM_INTERRUPTS == 8
#if (EXTERNAL_NUM_INTERRUPTS > 5)
        CASE_INT_N_FOR_INCENC(INT5)
        CASE_INT_N_FOR_INCENC(INT6)
        CASE_INT_N_FOR_INCENC(INT7)
#endif // EXTERNAL_NUM_INTERRUPTS > 5
        default:
          // still not attached
          extInterrupt[interruptNumber].mode = NOT_AN_INTERRUPT;
      }  // switch (interruptNumber)
         // check again to take in account 'No interrupt choosed' case
         if (NOT_AN_INTERRUPT != extInterrupt[interruptNumber].mode) {
            // if pin still not INPUT_PULLUP - system will hang up
            pinMode(_terminalAPin, INPUT_PULLUP);
            pinMode(_terminalBPin, INPUT_PULLUP);
            attachInterrupt(interruptNumber, interruptHandler, CHANGE);
            extInterrupt[interruptNumber].encTerminalAPinBit = digitalPinToBitMask(_terminalAPin);
            extInterrupt[interruptNumber].encTerminalBPinBit = digitalPinToBitMask(_terminalBPin);
            extInterrupt[interruptNumber].encTerminalAPIR = portInputRegister(digitalPinToPort(_terminalAPin));
            extInterrupt[interruptNumber].encTerminalBPIR = portInputRegister(digitalPinToPort(_terminalBPin));
            // reinit counter
            extInterrupt[interruptNumber].count = _initialValue;
           }
      } // if (NOT_AN_INTERRUPT == extInterrupt[interruptNumber].mode)
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        *_dst = (int64_t) extInterrupt[interruptNumber].value; 
      }
      return RESULT_IN_LONGVAR;
}
#endif // FEATURE_INCREMENTAL_ENCODER_ENABLE
