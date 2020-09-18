#if defined(FEATURE_USER_FUNCTION_PROCESSING)
/*****************************************************************************************************************************

   Subroutine calls on start of Zabbuino

*****************************************************************************************************************************/
void initStageUserFunction(uint8_t* _buffer) {
  __DMLL( Serial.println(F("initStageUserFunction")); )
}

/*****************************************************************************************************************************

   Subroutine calls when config is already loaded, but network not started yet

*****************************************************************************************************************************/
void netPrepareStageUserFunction(uint8_t* _buffer) {
  __DMLL( Serial.println(F("netPrepareStageUserFunction")); )
}

/*****************************************************************************************************************************

   Subroutine calls before service loop entering, and after all needs init stages

*****************************************************************************************************************************/
void preLoopStageUserFunction(char* _buffer) {
  __DMLL( Serial.println(F("preLoopStageUserFunction")); )
}

/*****************************************************************************************************************************

   Subroutine calls if alarm occured

*****************************************************************************************************************************/
void alarmStageUserFunction(uint8_t* _buffer, uint8_t _errorCode) {
  __DMLL( Serial.println(F("alarmStageUserFunction")); )
}

/*****************************************************************************************************************************

   Subroutine calls on every loop if no active network session exist

*****************************************************************************************************************************/
void loopStageUserFunction(char* _buffer) {
  __DMLL( Serial.println(F("loopStageUserFunction")); )
}


/*****************************************************************************************************************************

   Subroutine calls on user.run[] command processing
    _dst    - internal buffer, tune.h > constBufferSize bytes size. Note: If you modify it - you are change options (option#0..5) content
    _optarg - array of pointer to options represented as C-string. _optarg[n] point to part of _buffer that is begin of n-th option
    _argv   - array of options represented as signed numbers.

     This subroutine must return result code to parent function that return it to user.
*****************************************************************************************************************************/
int8_t executeCommandUserFunction(char* _buffer, char** _optarg, int32_t* _argv, int32_t* value) {
  __DMLL( Serial.println(F("executeCommandUserFunction")); )
  return RESULT_IS_OK;
}

#endif // #if defined(FEATURE_USER_FUNCTION_PROCESSING)
