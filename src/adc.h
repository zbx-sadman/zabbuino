#ifndef ZabbuinoADC_h
#define ZabbuinoADC_h

#include <Arduino.h>
#include "defaults.h"
#include "system.h"


// for median sampling algo: every sample is int16_t number, total memory consumption is (ADC_SAMPLES * 2) bytes
#define ADC_SAMPLES                         1000  

/*****************************************************************************************************************************
*
*   Measure the voltage on given analogChannel.
*
*   Returns: 
*     - Voltage in mV
*
*****************************************************************************************************************************/
uint16_t getADCVoltage(const uint8_t _analogChannel);


/*****************************************************************************************************************************
*
*  Read specified metric's value of the ACS712 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - always RESULT_IN_BUFFER 
*
*****************************************************************************************************************************/
int8_t getACS7XXMetric(const uint8_t _sensorPin, uint32_t _aRefVoltage,  const uint8_t _metric, const uint8_t _sensitivity, const int32_t _ZeroCurrentPoint, char *_outBuffer);

#endif // #ifndef ZabbuinoADC_h