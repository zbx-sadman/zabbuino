#define USER_FUNCTION_SUBCOMMAND_DROP_ALARM_TIMEOUT           (0x13)

uint32_t commandUserFunctionTimeout = 60000UL;
uint32_t commandUserFunctionStartTime = 0x00UL;
uint8_t alarmLedPin = 0x08;  // constStateLedPin = 0x09
uint8_t alarmLedPinDefaultState = LOW;
uint8_t alarmLedPinWorkState = HIGH;

/*****************************************************************************************************************************

   Subroutine calls before service loop entering, and after all needs init stages

*****************************************************************************************************************************/
void preLoopStageUserFunction(char* _dst) {
  __DMLL( Serial.println(F("preLoopStageUserFunction")); )
  //  commandUserFunctionStartTime = millis();
}

/*****************************************************************************************************************************

   Subroutine calls on user.run[] command processing
    _buffer - internal buffer, tune.h > constBufferSize bytes size. Note: If you modify it - you are change options (option#0..5) content
    _optarg - array of pointer to options represented as C-string. _optarg[n] point to part of _buffer that is begin of n-th option
    _argv   - array of options represented as signed numbers.

     This subroutine must return result code to parent function that return it to user.
*****************************************************************************************************************************/
int8_t executeCommandUserFunction(char* _dst, char** _optarg, int32_t* _argv, int32_t* value) {
  int8_t rc = RESULT_IS_FAIL;
  uint8_t userFunctionSubcommandCode = _argv[0],
          newAlarmLedPin = _argv[1],
          newAlarmLedPinDefaultState = _argv[3],
          newAlarmLedPinWorkState = _argv[4];
  uint32_t newCommandUserFunctionTimeout  = _argv[2];

  newAlarmLedPinDefaultState = (0x00 == newAlarmLedPinDefaultState) ? LOW : HIGH;
  newAlarmLedPinWorkState = (0x00 == newAlarmLedPinWorkState) ? LOW : HIGH;

  __DMLL( Serial.println(F("executeCommandUserFunction")); )

  if (USER_FUNCTION_SUBCOMMAND_DROP_ALARM_TIMEOUT == userFunctionSubcommandCode) {
    rc = RESULT_IS_FAIL;
    if (! isSafePin(newAlarmLedPin)) {
      goto finish;
    }
    if (! 0x00 > newCommandUserFunctionTimeout) {
      goto finish;
    }
    commandUserFunctionTimeout = newCommandUserFunctionTimeout;

    alarmLedPinDefaultState = newAlarmLedPinDefaultState;
    alarmLedPinWorkState = newAlarmLedPinWorkState;

    if (alarmLedPin != newAlarmLedPin) {
      pinMode(alarmLedPin, OUTPUT);
    }
    digitalWrite(alarmLedPin, newAlarmLedPinDefaultState);

    commandUserFunctionStartTime = millis();
    rc = RESULT_IS_OK;
  }

finish:
  return rc;
}

/*****************************************************************************************************************************

   Subroutine calls on every loop if no active network session exist

*****************************************************************************************************************************/
void loopStageUserFunction(char* _dst) {
  __DMLL( Serial.println(F("loopStageUserFunction")); )
  if (millis() - commandUserFunctionStartTime > commandUserFunctionTimeout) {
    __DMLL( Serial.println(F("loopStageUserFunction: commandUserFunctionTimeout is expired")); )
    digitalWrite(alarmLedPin, alarmLedPinDefaultState);
  } else {
    digitalWrite(alarmLedPin, alarmLedPinWorkState);
  }
}

