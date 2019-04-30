#pragma once

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                      EXTERNAL INTERRUPTS HANDLING SECTION

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE

void initExtInt(void);

/*****************************************************************************************************************************
*
*  Attach/detach interrupts, return counter value
*
*  Returns: 
*    - value of counter belonging to interrupt
*    - RESULT_IS_FAIL if interrupt mode is wrong of wrong pin is specified
*
*****************************************************************************************************************************/
  int8_t manageExtInt(uint32_t* _dst, uint8_t _pin, uint8_t _mode);

/*
 macro CASE_INT_N_FOR_EXTINT(INT0) will be transformed to code:

   case INT0:
     interruptHandler = handleExtINT0;
     break;

*/
#define CASE_INT_N_FOR_EXTINT(_interrupt) \
    case _interrupt: \
       interruptHandler = handleExt##_interrupt;\
    break;



/*

 macro HANDLE_INT_N_FOR_EXTINT(INT0) will be  transformed to code:

    void handleExtINT0() { extern extInterrupt_t* extInterrupt; extInterrupt[INT0].count++; }
*/

#define HANDLE_INT_N_FOR_EXTINT(_interrupt) \
   void handleExt##_interrupt(void) { ++extInterrupt[_interrupt].value; }

 
#if (EXTERNAL_NUM_INTERRUPTS > 7)
  void handleExtINT7(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 6)
  void handleExtINT6(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 5)
  void handleExtINT5(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 4)
  void handleExtINT4(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 3)
  void handleExtINT3(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 2)
  void handleExtINT2(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 1)
  void handleExtINT1(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 0)
  void handleExtINT0(void);
#endif


#endif // FEATURE_EXTERNAL_INTERRUPT_ENABLE


/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                      ENCODER INTERRUPTS HANDLING SECTION

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE
/*****************************************************************************************************************************
*
*  Attach/detach interrupts, return encoder's variable value
*
*  Returns: 
*    - value of variable belonging to interrupt applies to the pin to which encoder's "Terminal A" connected
*    - RESULT_IS_FAIL if wrong pin is specified
*
*****************************************************************************************************************************/
  int8_t manageIncEnc(int32_t* _dst, uint8_t const _terminalAPin, uint8_t const _terminalBPin, int32_t const _initialValue);

/*
 macro CASE_INT_N_FOR_INCENC(INT0) will be transformed to code:

   case INT0:
     interruptHandler = handleExtINT0;
     break;

*/
#define CASE_INT_N_FOR_INCENC(_interrupt) \
    case _interrupt: \
       interruptHandler = handleIncEnc##_interrupt;\
    break;

                                            
// need use overflow test for "extInterrupt[_interrupt].value++" and "extInterrupt[_interrupt].value--" to avoid sign bit flapping  
// { ((int32_t) extInterrupt[_interrupt].value)++; } else { ((int32_t) extInterrupt[_interrupt].value)--; }
#define HANDLE_INT_N_FOR_INCENC(_interrupt) \
   void handleIncEnc##_interrupt(void) { volatile static uint8_t stateTerminalA = 0, statePrevTerminalA = 0;\
         delayMicroseconds(constEncoderStabilizationDelay);\
         stateTerminalA = *extInterrupt[_interrupt].encTerminalAPIR & extInterrupt[_interrupt].encTerminalAPinBit;\
         if ((!stateTerminalA) && (statePrevTerminalA)) { \
           if (*extInterrupt[_interrupt].encTerminalBPIR & extInterrupt[_interrupt].encTerminalBPinBit) \
              { ++extInterrupt[_interrupt].value; } else { --extInterrupt[_interrupt].value; } \
         } statePrevTerminalA = stateTerminalA; }

#if (EXTERNAL_NUM_INTERRUPTS > 7)
  void handleIncEncINT7(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 6)
  void handleIncEncINT6(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 5)
  void handleIncEncINT5(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 4)
  void handleIncEncINT4(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 3)
  void handleIncEncINT3(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 2)
  void handleIncEncINT2(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 1)
  void handleIncEncINT1(void);
#endif
#if (EXTERNAL_NUM_INTERRUPTS > 0)
  void handleIncEncINT0(void);
#endif

#endif // FEATURE_INCREMENTAL_ENCODER_ENABLE

