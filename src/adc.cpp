// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "adc.h"

/*****************************************************************************************************************************
*
*   Measure the voltage on specified analog channel.
*
*   Returns: 
*     - Voltage in mV
*
*****************************************************************************************************************************/
uint16_t getADCVoltage(const uint8_t _analogChannel) {  
#if defined(ARDUINO_ARCH_AVR)
  uint8_t oldADCSRA, oldADMUX;
  uint16_t i;
  uint32_t avgADC = 0x00;
  oldADMUX = ADMUX;

  // ATmega328 /  ATmega2560 just used different _analogChannel to get 1.1V ref voltage value
  ADMUX = _BV(REFS0) | _analogChannel;

  //  save ADCSRA register
  oldADCSRA = ADCSRA;
  //ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2));
  //ADCSRA |= bit (ADPS0);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
  ADCSRA |= (1 << ADEN);
  // Wait for Vref to settle. No delay() used because sub can be called from interrupt
  // if delayMicroseconds(2000) used - PWM routines (tone(), for example) work is break ;
  i = 2000; while (i--) { delayMicroseconds(1); }

  // get 255 samples
  i = 0xFF; 
  while (i--) {
    ADCSRA |= (1 << ADSC);  // start a new conversion
    while (bit_is_set(ADCSRA, ADSC)) {;} // wait for conversion finish
    avgADC += ADC; 
  }
  // Calculate average
  avgADC /= 0xFF;

  //  restore ADCSRA register
  ADCSRA = oldADCSRA;
  ADMUX = oldADMUX;
  avgADC = 1125300UL / avgADC; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  // No delay() used because sub can be called from interrupt
  // i = 1500; while (i--){ delayMicroseconds(1);}
  //  delayMicroseconds(2000);
  return ((uint16_t) avgADC);
#elif (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))

  __SUPPRESS_WARNING_UNUSED(_analogChannel);

  return ((uint16_t) 0x00);
#endif
}

// Re: ACS712 Sensor
// Its a Hall-sensor, it is very noisy, all hall-current sensors are very noisy.
// I'd expect it to be accurate to 0.5A or so if its the +/-30A device.
/*****************************************************************************************************************************
*
*  Read specified metric's value of the ACS712 sensor, put it to output buffer on success. 
*
*  Returns: 
*    - always RESULT_IS_BUFFERED 
*
*  Note: code is not tested in production
*
*****************************************************************************************************************************/
int8_t getACS7XXMetric(const uint8_t _sensorPin, uint32_t _sampleTime, uint32_t _aRefVoltage, const uint8_t _metric, const uint8_t _sensitivity, const int32_t _ZeroCurrentPoint, char* _dst)
{  
  __SUPPRESS_WARNING_UNUSED(_sampleTime);

  uint32_t sampleInterval, mVperUnit, prevMicros = 0;
  int32_t result, adcValue, numUnits = 0;
  int16_t samplesCount = 0; // adcTable[ADC_SAMPLES], 

  pinMode(_sensorPin, INPUT);
  
  // _aRef point to use internal voltage (very unstable with onboard Ethernet, sensors, etc. But show some digits ;) 
  if (DEFAULT == _aRefVoltage) {
     // Take internal MCU's VCC
     _aRefVoltage = (uint32_t) getADCVoltage(ANALOG_CHAN_VBG);
  } else {
     // Otherwise - use _aRef as referenve voltage value that is given in mV
     //aRefVoltage = _aRefVoltage;
#ifdef FEATURE_AREF_ENABLE
     // if AREF feature is enabled - activate external reference voltage source
    analogReference(EXTERNAL);
    delay(2);
#endif
  }

  /**** Gathering ****/
  // need to find reliable gathering algo. 
  // median average is not so good -  the result near (a little bit better) to simply arithmetic mean
  
  // 100000UL mcsec => 100 msec (for 50Hz)
  sampleInterval = 100000UL/ADC_SAMPLES; 

#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
      // Stop the Timer1 to prevent calling gatherMetrics and ADC disturb by getVoltage() and so
      stopTimerOne(); 
#endif
  // Do not disturb processes by internal routines 
  //skipMetricGathering = true;

  // *** Do not disable interrupts here - the system will hang ***

  // ************ Simple algo for search arithmethic mean *****************
  while (samplesCount < ADC_SAMPLES) {
    if ((micros() - prevMicros) >= sampleInterval) {
       // if numUnits give strange results, calc adcValue before and add then it to numUnits
       adcValue = (int32_t) analogRead(_sensorPin) - _ZeroCurrentPoint;
       if (SENS_READ_AC == _metric) {
          numUnits += adcValue * adcValue;
       } else {
          numUnits += adcValue;
       }
       samplesCount++;
       prevMicros = micros();
       // ADC stabilization delay
       delayMicroseconds(constAdcStabilizationDelay);
    }
  }

#ifdef GATHER_METRIC_USING_TIMER_INTERRUPT
      startTimerOne(); 
#endif

  switch (_metric) {
    case SENS_READ_ZC:
    case SENS_READ_DC:
      // Take mean of analogread()
      numUnits = numUnits / ADC_SAMPLES;
      break;

    case SENS_READ_AC:
      break;
  }

  /**** Calculation ****/
  switch (_metric) {
    case SENS_READ_ZC:
      result = numUnits;
      break;
    case SENS_READ_AC:
    case SENS_READ_DC:
      // 1000 * _aRefVoltage - "un-float" procedure
      mVperUnit = 1000 * _aRefVoltage / 1023;
//  Serial.print("mVperUnit: ");   Serial.println(mVperUnit); 
      // I (mA) = (1000 * _aRefVoltage / 1023) * units * (1 / 185) =>  I (mA) = mVperUnit * units / SENSITIVITY
      // (VCC / 1024) * units - how many mV give ACS712
      // (1 / SENSITIVITY) - Amps on mV
      result =  mVperUnit * numUnits / _sensitivity;
     default:
       return RESULT_IS_FAIL;
  }

  ltoa(result, _dst, 10);

  return RESULT_IS_BUFFERED;
}

