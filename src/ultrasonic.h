#ifndef ZabbuinoULTRASONIC_h
#define ZabbuinoULTRASONIC_h

#include "../basic.h"
#include "tune.h"


/*****************************************************************************************************************************
*
*  Read the distance of the object with HC-SR04 Ultrasonic sensor.
*
*   Returns: 
*     - distance in mm
*
*****************************************************************************************************************************/
uint32_t getUltrasonicMetric(const uint8_t _triggerPin, const uint8_t _echoPin);

#endif // #ifndef ZabbuinoULTRASONIC_h