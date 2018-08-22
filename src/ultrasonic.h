#pragma once

#define ULTRASONIC_SAMPLES                       (0x05)

/*****************************************************************************************************************************
*
*  Read the distance of the object with HC-SR04 Ultrasonic sensor.
*
*   Returns: 
*     - distance in mm
*
*****************************************************************************************************************************/
uint32_t getUltrasonicMetric(const uint8_t _triggerPin, const uint8_t _echoPin);

