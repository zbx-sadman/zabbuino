
#define USER_FUNCTION_SUBCOMMAND_DROP_ALARM_TIMEOUT           (0x13)

uint32_t commandUserFunctionTimeout = 60000UL;
uint32_t commandUserFunctionStartTime = 0UL;
uint8_t alarmLedPin = 0;
uint8_t alarmLedPinDefaultState = LOW;
uint8_t alarmLedPinWorkState = HIGH;

void preLoopStageUserFunction(char* _dst) {
  DTSM( PRINTLN_PSTR("preLoopStageUserFunction"); )
  //  commandUserFunctionStartTime = millis();
}

int8_t executeCommandUserFunction(char* _dst, char** _optarg, int32_t* _argv, int32_t* value) {
  int8_t rc = RESULT_IS_FAIL;
  uint8_t userFunctionSubcommandCode = _argv[0],
          newAlarmLedPin = _argv[1],
          newAlarmLedPinDefaultState = _argv[3],
          newAlarmLedPinWorkState = _argv[4];
  uint32_t newCommandUserFunctionTimeout  = _argv[2];

  newAlarmLedPinDefaultState = (0x00 == newAlarmLedPinDefaultState) ? LOW : HIGH;
  newAlarmLedPinWorkState = (0x00 == newAlarmLedPinWorkState) ? LOW : HIGH;

  DTSM( PRINTLN_PSTR("executeCommandUserFunction"); )

  if (USER_FUNCTION_SUBCOMMAND_DROP_ALARM_TIMEOUT == userFunctionSubcommandCode) {
    rc = RESULT_IS_FAIL;
    if (! isSafePin(newAlarmLedPin)) {
      goto finish;
    }
    if (! newCommandUserFunctionTimeout  < 0x00) {
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

void loopStageUserFunction(char* _dst) {
  DTSM( PRINTLN_PSTR("loopStageUserFunction"); )
  if (millis() - commandUserFunctionStartTime > commandUserFunctionTimeout) {
    DTSM( PRINTLN_PSTR("loopStageUserFunction: commandUserFunctionTimeout is expired"); )
    digitalWrite(alarmLedPin, alarmLedPinDefaultState);
  } else {
    digitalWrite(alarmLedPin, alarmLedPinWorkState);
  }
}


