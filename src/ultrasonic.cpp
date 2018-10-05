// Based on: https://github.com/JRodrigoTech/Ultrasonic-HC-SR04

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
//uint32_t getUltrasonicMetric(const uint8_t _triggerPin, const uint8_t _echoPin)
uint32_t getUltrasonicMetric(const uint8_t _triggerPin, const uint8_t _echoPin) {
  uint8_t i;
  uint16_t pulseTime;  

  uint32_t result = 0x00;

  stopTimerOne(); 

  pinMode(_triggerPin, OUTPUT);
  pinMode(_echoPin, INPUT_PULLUP);

  // HC-SR04 want some time for waking-up ? (otherwize return 0 to pulseIn())
  delay(25);

  for (i = 0; i < ULTRASONIC_SAMPLES; i++) {
    Serial.println("wait");
    digitalWrite(_triggerPin, LOW);
    delayMicroseconds(5);

    // Using IO trigger for at least 10us high level signal,
    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_triggerPin, LOW);
    // how much is the result if no obstacle exists ?
    pulseTime = pulseIn(_echoPin, HIGH, ULTRASONIC_TIMEOUT);
//    Serial.println(pulseTime);
    if (ULTRASONIC_TIMEOUT <= pulseTime || 0x00 == pulseTime) {
      result = ULTRASONIC_TIMEOUT;
      Serial.println("jump out");
      goto finish;
    }
    result += pulseTime;
    // Between pings must be silence period for 50ms or more
    // can delay be decreased for 10+2ms?
    delay(50);
  }
  
  result /= ULTRASONIC_SAMPLES;

  if ( 0x00 == result) { result = ULTRASONIC_TIMEOUT; }

finish:

  startTimerOne(); 

  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  // 29 * 2 = 58

  // Resolution : 0.3 cm => 3mm. 
  // Return result in mm
  return (result * 10  / 58 ) ;
}
