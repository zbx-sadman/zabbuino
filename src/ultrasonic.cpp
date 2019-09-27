// Config & common included files
#include "sys_includes.h"

#include "ultrasonic.h"

#include "service.h"
#include "system.h"

/*****************************************************************************************************************************
*
*  Read the distance of the object with HC-SR04 Ultrasonic sensor.
*
*   Returns: 
*     - distance in mm
*
*****************************************************************************************************************************/
int8_t getUltrasonicMetric(const uint8_t _triggerPin, const uint8_t _echoPin, uint8_t _samples, int32_t* _value) {
  int8_t rc = RESULT_IS_UNSIGNED_VALUE;

  if (0x01 > _samples) {
    goto finish;
  }

  if (ULTRASONIC_MAX_SAMPLES < _samples) {
    _samples = ULTRASONIC_MAX_SAMPLES;
  }
  *_value = 0x00;
  stopTimerOne();

  pinMode(_triggerPin, OUTPUT);
  pinMode(_echoPin, INPUT);

  for (uint8_t i = 0; _samples > i; i++) {
    //DEBUG_PORT.println("wait");
    digitalWrite(_triggerPin, LOW);
    delayMicroseconds(5);

    // Using IO trigger for at least 10us high level signal,
    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_triggerPin, LOW);
    // how much is the result if no obstacle exists ?
    uint16_t pulseTime = pulseIn(_echoPin, HIGH, ULTRASONIC_MAX_EXPECTED_DISTANCE_CM * ULTRASONIC_US_TO_CM_COEFFICIENT);
    //DEBUG_PORT.println(pulseTime);
    if (0x00 == pulseTime) {
      //DEBUG_PORT.println("jump out");
      rc = RESULT_IS_FAIL;
      goto finish;
    }
    *_value += pulseTime;
    // Between pings must be silence period for 50ms or more
    // can delay be decreased for 10+5ms?
    delay(50);
  }

  *_value /= _samples;

  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  // 29 * 2 = 58

  // Resolution : 0.3 cm => 3mm.
  // Return result in mm
  *_value = ((*_value * 10) / ULTRASONIC_US_TO_CM_COEFFICIENT);

finish:

  startTimerOne();
  return rc;

}


