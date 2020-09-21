#if defined(FEATURE_USER_FUNCTION_PROCESSING)

/*****************************************************************************************************************************

   Subroutine calls on start of Zabbuino

*****************************************************************************************************************************/
void initStageUserFunction(uint8_t* _buffer) {
  __SUPPRESS_WARNING_UNUSED(_buffer);

  __DMLD( Serial.println(F("initStageUserFunction")); )
}

/*****************************************************************************************************************************

   Subroutine calls when config is already loaded, but network not started yet

*****************************************************************************************************************************/
void netPrepareStageUserFunction(uint8_t* _buffer) {
  __SUPPRESS_WARNING_UNUSED(_buffer);

  __DMLD( Serial.println(F("netPrepareStageUserFunction")); )
}

/*****************************************************************************************************************************

   Subroutine calls before service loop entering, and after all needs init stages

*****************************************************************************************************************************/
void preLoopStageUserFunction(uint8_t* _buffer) {
  __SUPPRESS_WARNING_UNUSED(_buffer);

  __DMLD( Serial.println(F("preLoopStageUserFunction")); )
}

/*****************************************************************************************************************************

   Subroutine calls when System Function button stay pressed more than constSystemFunctionButtonWaitTime ms

*****************************************************************************************************************************/
void userFunctionButtonActivate(uint8_t* _buffer) {
  __SUPPRESS_WARNING_UNUSED(_buffer);

  __DMLD( Serial.println(F("userFunctionButtonActivate")); )
  
  // Factory reset block for Sonoff TH10/TH16 with button on GPIO0
#ifdef FEATURE_EEPROM_ENABLE
  // factoryReset() return false on EEPROM saving fail or not executed
//  factoryReset(constSystemFunctionButtonPin, constSystemFunctionButtonActiveState, sysConfig);
#endif // FEATURE_EEPROM_ENABLE

}

/*****************************************************************************************************************************

   Subroutine calls when System Function button stay pressed more than constSystemFunctionButtonWaitTime ms

*****************************************************************************************************************************/
void userFunctionButtonDeactivate(uint8_t* _buffer) {
  __SUPPRESS_WARNING_UNUSED(_buffer);
  __DMLD( Serial.println(F("userFunctionButtonDeactivate")); )
}

/*****************************************************************************************************************************

   Subroutine calls if alarm occured

*****************************************************************************************************************************/
void alarmStageUserFunction(uint8_t* _buffer, uint8_t _errorCode) {
  __SUPPRESS_WARNING_UNUSED(_buffer);
  __SUPPRESS_WARNING_UNUSED(_errorCode);

// __DMLD( Serial.println(F("alarmStageUserFunction")); )
}

/*****************************************************************************************************************************

   Subroutine calls on every loop if no active network session exist

*****************************************************************************************************************************/
void loopStageUserFunction(uint8_t* _buffer) {
  __SUPPRESS_WARNING_UNUSED(_buffer);

  __DMLD( Serial.println(F("loopStageUserFunction")); )
}


/*****************************************************************************************************************************

   Subroutine calls on user.run[] command processing
    _dst    - internal buffer, src/cfg_tune.h > constBufferSize bytes size. Note: If you modify it - you are change options (option#0..5) content
    _optarg - array of pointer to options represented as C-string. _optarg[n] point to part of _buffer that is begin of n-th option
    _argv   - array of options represented as signed numbers.

     This subroutine must return result code to parent function that return it to user.
*****************************************************************************************************************************/
int8_t executeCommandUserFunction(uint8_t* _buffer, char** _optarg, int32_t* _argv, int32_t* value) {
  __SUPPRESS_WARNING_UNUSED(_buffer);
  __SUPPRESS_WARNING_UNUSED(_optarg);
  __SUPPRESS_WARNING_UNUSED(_argv);
  __SUPPRESS_WARNING_UNUSED(value);

  __DMLD( Serial.println(F("executeCommandUserFunction")); )
  return RESULT_IS_OK;
}

#endif // #if defined(FEATURE_USER_FUNCTION_PROCESSING)
