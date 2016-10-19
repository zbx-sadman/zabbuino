#ifndef ZabbuinoULTRASONIC_h
#define ZabbuinoULTRASONIC_h
#include <Arduino.h>
#include "defaults.h"
#include "../zabbuino.h"


uint32_t getUltrasonicMetric(const uint8_t _triggerPin, const uint8_t _echoPin);

#endif