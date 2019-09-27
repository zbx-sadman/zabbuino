#if defined(FEATURE_USER_FUNCTION_PROCESSING)

#define USER_FUNCTION_SUBCOMMAND_DROP_ALARM_TIMEOUT           (0x13)
#define USER_FUNCTION_WRONG_VALUE                             (-99)

uint32_t commandUserFunctionTimeout = 60000UL;
uint32_t commandUserFunctionStartTime = 0x00UL;
uint8_t alarmLedPin = 0x08;  // constStateLedPin = 0x09
uint8_t alarmLedPinDefaultState = LOW;
uint8_t alarmLedPinWorkState = HIGH;
static int32_t co2eLevel = 0x00, tvocLevel = 0x00;
static uint8_t sensorSdaPin = 0x00, sensorSclPin = 0x00, firstRun = true;

/*****************************************************************************************************************************

   Subroutine calls before service loop entering, and after all needs init stages

*****************************************************************************************************************************/
void preLoopStageUserFunction(uint8_t* _buffer) {
  // DTSM( Serial.println(F("preLoopStageUserFunction")); )
  //  commandUserFunctionStartTime = millis();
}

/*****************************************************************************************************************************

   Subroutine calls on user.run[] command processing
    _buffer - internal buffer, tune.h > constBufferSize bytes size. Note: If you modify it - you are change options (option#0..5) content
    _optarg - array of pointer to options represented as C-string. _optarg[n] point to part of _buffer that is begin of n-th option
    _argv   - array of options represented as signed numbers.

     This subroutine must return result code to parent function that return it to user.
*****************************************************************************************************************************/
int8_t executeCommandUserFunction(uint8_t* _buffer, char** _optarg, int32_t* _argv, int32_t* value) {
  int8_t rc = DEVICE_ERROR_NOT_SUPPORTED;
  uint8_t userFunctionSubcommandCode = _argv[0];

  //Serial.print("executeCommandUserFunction: 0x"); Serial.println(userFunctionSubcommandCode, HEX);
  if (CMD_SGP30_CO2E == userFunctionSubcommandCode || CMD_SGP30_TVOC == userFunctionSubcommandCode) {
    sensorSdaPin = _argv[1], sensorSclPin = _argv[2];

    //Serial.println("change sda/scl");
    if (firstRun) {
      //Serial.println("firstRun");
      loopStageUserFunction(_buffer); firstRun = false;
    }

    switch (userFunctionSubcommandCode ) {
      case CMD_SGP30_CO2E:
        if (USER_FUNCTION_WRONG_VALUE != co2eLevel) {
          *value = co2eLevel;
          rc = RESULT_IS_UNSIGNED_VALUE;
        }
        break;

      case CMD_SGP30_TVOC:
        if (USER_FUNCTION_WRONG_VALUE != tvocLevel) {
          *value = tvocLevel;
          rc = RESULT_IS_UNSIGNED_VALUE;
        }
        break;

      default:
        break;
    }
  }

  //finish:
  return rc;
}

/*****************************************************************************************************************************

   Subroutine calls on every loop if no active network session exist

*****************************************************************************************************************************/
void loopStageUserFunction(uint8_t* _buffer) {
  int8_t rc;
  uint8_t success = false;
  int32_t humidity = 0x00, temperature = 0x00;

  if (0x00 == sensorSdaPin || 0x00 == sensorSclPin) {
    //Serial.println("0x00 == sensorSdaPin");
    return;
  }

  SoftTWI.reconfigure(sensorSdaPin, sensorSclPin);

  rc = getSHT2XMetric(&SoftTWI, SHT2X_I2C_ADDRESS, SENS_READ_TEMP, &temperature);
  success &= (0x00 < rc);

  rc = getSHT2XMetric(&SoftTWI, SHT2X_I2C_ADDRESS, SENS_READ_HUMD, &humidity);
  success = (0x00 < rc);

  if (!success) {
    humidity = -1;
  } else {
    // https://github.com/finitespace/BME280/issues/25#issuecomment-304376765
    //  https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
    //
    // https://cdn.sos.sk/productdata/45/a8/fffeb1f9/sgp30.pdf
    // AH = 216.7 * ( (RH/100) * 6.112 * exp( (17.62*t) / (243.12+t) ) ) / (273.15+t)
    float tempHum = humidity / 100.0;
    float tempTemp = temperature / 100.0;
    float tempVar = (17.62 * tempTemp) / (tempTemp + 243.12);
    tempVar = pow(2.718281828, tempVar);
    tempVar = ((216.7 * tempHum / 100 ) * 6.112 * tempVar);
    tempVar =  tempVar / (273.15 + tempTemp);
    humidity = tempVar * 1000; // gramm => milligramm
  }

  if (0x00 >= getSGP30Metric(&SoftTWI, 0x58, humidity, SENS_READ_CO2E, false, &co2eLevel)) {
    co2eLevel = USER_FUNCTION_WRONG_VALUE;
    //DTSM( Serial.println(F("co2e err")); )
  }
  if (0x00 >= getSGP30Metric(&SoftTWI, 0x58, humidity, SENS_READ_TVOC, false, &tvocLevel)) {
    tvocLevel = USER_FUNCTION_WRONG_VALUE;
    //DTSM( Serial.println(F("tvoc err")); )
  }
  //DTSM( Serial.println(F("Updated")); )
}
#endif
