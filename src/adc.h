#ifndef ZabbuinoADC_h
#define ZabbuinoADC_h

#include <Arduino.h>
#include "../zabbuino.h"
#include "system.h"
#include "defaults.h"


#define ADC_SAMPLES                         1000  // for median sampling algo: every sample is int16_t number, total memory consumption is (ADC_SAMPLES * 2) bytes

/* ****************************************************************************************************************************
*
*   Measure the voltage on given analogChannel.
*
**************************************************************************************************************************** */
int32_t getADCVoltage(const uint8_t _analogChannel);

/* ****************************************************************************************************************************
*
*   Get Zero current point, AC & DC value from ACS712 sensor
*
**************************************************************************************************************************** */
int8_t getACS7XXMetric(const uint8_t _sensorPin, uint32_t _aRefVoltage,  const uint8_t _metric, const uint8_t _sensitivity, const int32_t _ZeroCurrentPoint, char* _outBuffer);

#endif