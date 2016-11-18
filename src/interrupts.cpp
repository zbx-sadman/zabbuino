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

#if EXTERNAL_NUM_INTERRUPTS > 7
   HANDLE_INT_N_FOR_EXTINT(INT7)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 6
   HANDLE_INT_N_FOR_EXTINT(INT6)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 5
   HANDLE_INT_N_FOR_EXTINT(INT5)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 4
   HANDLE_INT_N_FOR_EXTINT(INT4)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 3
   HANDLE_INT_N_FOR_EXTINT(INT3)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 2
   HANDLE_INT_N_FOR_EXTINT(INT2)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 1
   HANDLE_INT_N_FOR_EXTINT(INT1)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 0
   HANDLE_INT_N_FOR_EXTINT(INT0)
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
int8_t manageExtInt(uint32_t *_dst, uint8_t _pin, uint8_t _mode) {
   int8_t rc = RESULT_IS_FAIL;
   // This condition placed here to avoid using defaut case (EXTERNAL_NUM_INTERRUPTS <= 0) in switch(interruptNumber) due its very strange but 
   // theoretically possible situation 
#if EXTERNAL_NUM_INTERRUPTS > 0
   extern extInterrupt_t *extInterrupt;
   int32_t result;
   voidFuncPtr interruptHandler;
   int8_t interruptNumber=digitalPinToInterrupt(_pin);
   // NOT_AN_INTERRUPT == -1 - it's macro from Arduino.h
   // Interrupt number and mode is correct? If not - just jump to the end, because rc already init with RESULT_IS_FAIL value
   if ((NOT_AN_INTERRUPT == interruptNumber) || (EXTERNAL_NUM_INTERRUPTS < interruptNumber) || (RISING < _mode)) { goto finish; }

   // Just return counter value if interrupt mode is not changed, but interrupt is attached
   // Atomic block is used to get relable counter value due AVR8 make read long variables with series of ASM commands 
   // and value of the variable can be changed before reading is finished
   if ((extInterrupt[interruptNumber].mode != _mode)) {

      // Detach is need to avoid get strange results in extInterrupt[interruptNumber].count if external signals still incoming to pin
      // .owner field need to detect owner of counter by manageIncEnc() sub due it don't operate .mode field 
      detachInterrupt(interruptNumber);
      extInterrupt[interruptNumber].owner = OWNER_IS_EXTINT;
      extInterrupt[interruptNumber].mode = _mode;

      // Pin must be configured as input or system will hang up. 
      // INPUT_PULLUP pin mode allows you to avoid unwanted interference (and continuous calls of the interrupt handling sub) when 
      // pin lost connection to the external pull-up resistor
      pinMode(_pin, INPUT_PULLUP);
      // No ATOMIC_BLOCK(ATOMIC_RESTORESTATE{} used here due interrupt must be previosly detached
      // ...but if new mode is RISING and pin have HIGH state when attachInterrupt() will be called - counter will be increased immediately.
      // May be better init counter after attachInterrupt() to get 0 on any state of _pin?
      extInterrupt[interruptNumber].count = 0;

      switch (interruptNumber) {
// This code taken from WInterrupts.c and modifed
#if EXTERNAL_NUM_INTERRUPTS > 8
    #warning There are more than 8 external interrupts. Some callbacks may not be initialized.
#endif
#if EXTERNAL_NUM_INTERRUPTS > 7
        CASE_INT_N_FOR_EXTINT(INT7)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 6
        CASE_INT_N_FOR_EXTINT(INT6)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 5
        CASE_INT_N_FOR_EXTINT(INT5)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 4
        CASE_INT_N_FOR_EXTINT(INT4)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 3
        CASE_INT_N_FOR_EXTINT(INT3)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 2
        CASE_INT_N_FOR_EXTINT(INT2)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 1
        CASE_INT_N_FOR_EXTINT(INT1)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 0
        CASE_INT_N_FOR_EXTINT(INT0)
#endif
      }  // switch (interruptNumber)

      // Need to do checking NOT_AN_INTERRUPT == _mode and notattach if true?
      attachInterrupt(interruptNumber, interruptHandler, _mode);
      // No ATOMIC_BLOCK(ATOMIC_RESTORESTATE{} used here due interrupt must be previosly detached
      // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { extInterrupt[interruptNumber].count = 0; }
   }

   ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *_dst = (int64_t) extInterrupt[interruptNumber].count; }
   rc = RESULT_IN_ULONGVAR;
          
#endif // #if EXTERNAL_NUM_INTERRUPTS > 0

   finish:
   return rc;

   
}

#endif // FEATURE_EXTERNAL_INTERRUPT_ENABLE


/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                      ENCODER INTERRUPTS HANDLING SECTION

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE

#if EXTERNAL_NUM_INTERRUPTS > 7
   HANDLE_INT_N_FOR_INCENC(INT7)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 6
   HANDLE_INT_N_FOR_INCENC(INT6)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 5
   HANDLE_INT_N_FOR_INCENC(INT5)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 4
   HANDLE_INT_N_FOR_INCENC(INT4)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 3
   HANDLE_INT_N_FOR_INCENC(INT3)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 2
   HANDLE_INT_N_FOR_INCENC(INT2)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 1
   HANDLE_INT_N_FOR_INCENC(INT1)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 0
   HANDLE_INT_N_FOR_INCENC(INT0)
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
int8_t manageIncEnc(int32_t *_dst, uint8_t const _terminalAPin, uint8_t const _terminalBPin, int32_t const _initialValue) {
   int8_t rc = RESULT_IS_FAIL;

   // This condition placed here to avoid using defaut case (EXTERNAL_NUM_INTERRUPTS <= 0) in switch(interruptNumber) due its very strange but 
   // theoretically possible situation 
#if EXTERNAL_NUM_INTERRUPTS > 0
   extern extInterrupt_t *extInterrupt;
   voidFuncPtr interruptHandler;
   int8_t interruptNumber=digitalPinToInterrupt(_terminalAPin);
   // NOT_AN_INTERRUPT == -1 - it's macro from Arduino.h
   // Interrupt number and mode is correct? If not - just jump to the end, because rc already init with RESULT_IS_FAIL value
   if ((NOT_AN_INTERRUPT == interruptNumber) || (EXTERNAL_NUM_INTERRUPTS < interruptNumber)) { goto finish; }

   // Just return value of .value field if encoder's handle sub already linked to this interrupt
   // Otherwise - reattach interrupt and init .value field
   if (OWNER_IS_INCENC != extInterrupt[interruptNumber].owner) {

      // Detach is need to avoid get strange results in extInterrupt[interruptNumber].value if external signals still incoming to pin
      detachInterrupt(interruptNumber);
      extInterrupt[interruptNumber].owner = OWNER_IS_INCENC;
      extInterrupt[interruptNumber].mode = CHANGE;
 
      // INPUT_PULLUP pin mode allows you to avoid unwanted interference (and continuous calls of the interrupt handling sub) when 
      // pins lost connection to the external pull-up resistor
      pinMode(_terminalAPin, INPUT_PULLUP);
      pinMode(_terminalBPin, INPUT_PULLUP);
      extInterrupt[interruptNumber].encTerminalAPinBit = digitalPinToBitMask(_terminalAPin);
      extInterrupt[interruptNumber].encTerminalBPinBit = digitalPinToBitMask(_terminalBPin);
      extInterrupt[interruptNumber].encTerminalAPIR = portInputRegister(digitalPinToPort(_terminalAPin));
      extInterrupt[interruptNumber].encTerminalBPIR = portInputRegister(digitalPinToPort(_terminalBPin));
      extInterrupt[interruptNumber].value = _initialValue;

      switch (interruptNumber) {
#if EXTERNAL_NUM_INTERRUPTS > 8
    #warning There are more than 8 external interrupts. Some callbacks may not be initialized.
#endif
#if EXTERNAL_NUM_INTERRUPTS > 7
        CASE_INT_N_FOR_INCENC(INT7)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 6
        CASE_INT_N_FOR_INCENC(INT6)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 5
        CASE_INT_N_FOR_INCENC(INT5)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 4
        CASE_INT_N_FOR_INCENC(INT4)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 3
        CASE_INT_N_FOR_INCENC(INT3)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 2
        CASE_INT_N_FOR_INCENC(INT2)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 1
        CASE_INT_N_FOR_INCENC(INT1)
#endif
#if EXTERNAL_NUM_INTERRUPTS > 0
        CASE_INT_N_FOR_INCENC(INT0)
#endif
        }  // switch (interruptNumber)
        attachInterrupt(interruptNumber, interruptHandler, CHANGE);
   } // (OWNER_IS_INCENC != extInterrupt[interruptNumber].owner)

   ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *_dst = (int64_t) extInterrupt[interruptNumber].value; }
   rc = RESULT_IN_LONGVAR;
#endif

   finish:
   return rc;

}
#endif // FEATURE_INCREMENTAL_ENCODER_ENABLE
