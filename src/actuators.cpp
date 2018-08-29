// Config & common included files
#include "sys_includes.h"

#include "actuators.h"
#include <util\atomic.h>

#include "service.h"
#include "system.h"


/*****************************************************************************************************************************
*
*  Send command to Servo to make turn it an turn back
*
*   Returns: 
*     - RESULT_IS_OK
*
*****************************************************************************************************************************/
int8_t servoTurn(const uint8_t _servoPin, const uint16_t _targetPulseWidth, const uint32_t _turnTime, const uint32_t _holdTime = 0, const uint16_t _returnPulseWidth = 0)
{
  uint32_t startHolding, idleTime, turnTime;
  uint8_t servoPinPort, servoPinBit;
  volatile uint8_t *servoOutRegister;
  int8_t rc = RESULT_IS_FAIL;

  servoPinBit  = digitalPinToBitMask(_servoPin);
  servoPinPort = digitalPinToPort(_servoPin);

  if (NOT_A_PIN == servoPinPort) {
    goto finish;
  }
  servoOutRegister = portOutputRegister(servoPinPort);

  pinMode(_servoPin, OUTPUT);

  stopTimerOne(); 

  if (_targetPulseWidth) {
    // if target angle (thru _targetPulseWidth) is specified - servo must get pulses every PULSE_INTERVAL (usually 20) ms 
    //     for _turnTime and _holdTime to stay on position
    //
    // Work cycle = pulse_width_in_uS + (20 ms - pulse_width_in_ms)
    idleTime = SERVO_PULSE_INTERVAL - (_targetPulseWidth / 1000UL);
    turnTime = _turnTime + _holdTime;

    startHolding = millis();
    // Send pulse to turn the servo and hold it on position
    do {
      // Turn to the "target" angle
      //  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      *servoOutRegister |= servoPinBit;
      delayMicroseconds(_targetPulseWidth);
      *servoOutRegister &= ~servoPinBit;
      //   }
      // delay() is used because delayMicroseconds take no more that 16000 uS
      delay(idleTime);
    } while (turnTime > millis() - startHolding);
  }

  if (_returnPulseWidth) {
    // if return angle (thru _returnPulseWidth) is specified - servo must get pulses every PULSE_INTERVAL (usually 20) ms 
    //     for _turnTime
    //
    idleTime = SERVO_PULSE_INTERVAL - (_returnPulseWidth / 1000UL);

    startHolding = millis();
    do {
      // Turn to the "return" angle
      //  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      *servoOutRegister |= servoPinBit;
      delayMicroseconds(_returnPulseWidth);
      *servoOutRegister &= ~servoPinBit;
      //   }
      // delay() is used because delayMicroseconds take no more that 16000 uS
      delay(idleTime);
    } while (_turnTime > millis() - startHolding);
  }

  startTimerOne(); 
  rc = RESULT_IS_OK;

finish:
  gatherSystemMetrics(); 
  return rc;
}
        

/*****************************************************************************************************************************
*
*  
*
*   Returns: 
*     - RESULT_IS_OK
*
*****************************************************************************************************************************/
int8_t pulse(const uint8_t _targetPin, const uint8_t _targetState, const uint32_t _holdTime, const uint8_t _returnState)
{
  pinMode(_targetPin, OUTPUT);
  digitalWrite(_targetPin, _targetState);
  delay(_holdTime);
  digitalWrite(_targetPin, _returnState);

  gatherSystemMetrics(); 
  return RESULT_IS_OK;
}
        

/*****************************************************************************************************************************
*
*  Send command to Servo to make turn it an turn back
*
*   Returns: 
*     - RESULT_IS_OK
*
*****************************************************************************************************************************/
int8_t relay(const uint8_t _targetPin, const uint8_t _targetState, const int8_t _testPin = -1, const int8_t _testState = -1,  uint8_t _testPinMode = INPUT)
{

  int8_t rc = RESULT_IS_OK;
  uint8_t t;

  pinMode(_targetPin, OUTPUT);
  digitalWrite(_targetPin, _targetState);

  if (0 > _testPin || 0 > _testState) {
     goto finish;
  }
  
  pinMode(_testPin, _testPinMode);
  delay(10);
  t = digitalRead(_testPin);
  Serial.print("Test state: "); Serial.println(_testState);
  Serial.print("Real state: "); Serial.println(t);

  if (_testState != t) { rc = RESULT_IS_FAIL; 
   Serial.println("FAIL");
  }else {
   Serial.println("OK");
  }

finish:
  gatherSystemMetrics(); 
  return rc;
}
