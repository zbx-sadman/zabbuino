#pragma once

#define ULTRASONIC_SAMPLES                       (0x05)

  // timeout in microseconds. 38000 * 10 / 58 => 6551. It is out of distance range (too close or too far).
#define ULTRASONIC_TIMEOUT                       (38000UL)

/*****************************************************************************************************************************
*
*  Read the distance of the object with HC-SR04 Ultrasonic sensor.
*
*   Returns: 
*     - distance in mm
*
*****************************************************************************************************************************/
//uint32_t getUltrasonicMetric(const uint8_t _triggerPin, const uint8_t _echoPin);
uint32_t getUltrasonicMetric(const uint8_t, const uint8_t);

