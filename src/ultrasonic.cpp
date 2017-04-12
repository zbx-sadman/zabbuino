// Based on: https://github.com/JRodrigoTech/Ultrasonic-HC-SR04

#include "ultrasonic.h"

/*****************************************************************************************************************************
*
*  Read the distance of the object with HC-SR04 Ultrasonic sensor.
*
*   Returns: 
*     - distance in mm
*
*****************************************************************************************************************************/
uint32_t getUltrasonicMetric(const uint8_t _triggerPin, const uint8_t _echoPin)
{
  uint8_t i;
  // timeout in microseconds. 38000 * 10 / 58 => 6551. It is out of distance range (too close or too far).
  uint16_t timeOut = 38000;  
  uint32_t result = 0;

  pinMode(_triggerPin, OUTPUT);
  pinMode(_echoPin, INPUT);

  // delay(50); // Zabbuino will execute command more that 50ms (~300ms on ATMega328P), lets save some progspace
  for (i = 0; i < ULTRASONIC_SAMPLES; i++) {
      digitalWrite(_triggerPin, LOW);
      delayMicroseconds(2);
 
      // Using IO trigger for at least 10us high level signal, 
      digitalWrite(_triggerPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(_triggerPin, LOW);
      // how much is the result if no obstacle exists ?
      result += pulseIn(_echoPin, HIGH, timeOut);
      // Between pings must be silence period for 50ms or more
      // can delay be decreased for 10+2ms?
      delay(50);
  }
  
  result /= ULTRASONIC_SAMPLES;

  if ( 0 == result) { result = timeOut; }
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  // 29 * 2 = 58

  // Resolution : 0.3 cm => 3mm. 
  // Return result in mm
  return (result * 10  / 58 ) ;
}
