#pragma once

/*

*/

#define ULTRASONIC_MAX_SAMPLES                   (0x05)

#define ULTRASONIC_MAX_EXPECTED_DISTANCE_CM      (400)

#define ULTRASONIC_US_TO_CM_COEFFICIENT          (58)

/*****************************************************************************************************************************
*
*  Read the distance of the object with HC-SR04 Ultrasonic sensor.
*
*   Returns: 
*     - distance in mm
*
*****************************************************************************************************************************/
int8_t getUltrasonicMetric(const uint8_t, const uint8_t, uint8_t, int32_t*);

