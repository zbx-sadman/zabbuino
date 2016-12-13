#define ADC_SAMPLES                         1000  // for median sampling algo: every sample is int16_t number, total memory consumption is (ADC_SAMPLES * 2) bytes

/* ****************************************************************************************************************************
*
*   Measure the voltage on given analogChannel.
*
**************************************************************************************************************************** */
int32_t getADCVoltage(const uint8_t _analogChannel) {  
  uint8_t i, oldADCSRA, oldADMUX;
  uint32_t avgADC=0;
  
  oldADMUX = ADMUX;
  ADMUX = (0 << REFS1) | (1 << REFS0) | _analogChannel;
  //  save ADCSRA register
  oldADCSRA = ADCSRA;
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
  ADCSRA |= (1 << ADEN);
  // Wait for Vref to settle. No delay() used because sub can be called from interrupt
  delayMicroseconds(2000);

  // get 255 samples
  for (i = 0; i < 255; i++ ) {
    ADCSRA |= (1 << ADSC);  // start a new conversion
    while (bit_is_set(ADCSRA, ADSC)); // wait for conversion finish
    avgADC += ADC; 
  }
  // Calculate average
  avgADC /= 255;

  //  restore ADCSRA register
  ADCSRA = oldADCSRA;
  ADMUX = oldADMUX;
  // No delay() used because sub can be called from interrupt
  delayMicroseconds(2000);
  avgADC = 1125300L / avgADC; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

  return avgADC;
}

/* ****************************************************************************************************************************
*
*   Get Zero current point, AC & DC value from ACS712 sensor
*
**************************************************************************************************************************** */
int32_t getACS7XXMetric(const uint8_t _sensorPin, uint32_t _aRefVoltage,  const uint8_t _metric, const uint8_t _sensitivity, const int32_t _ZeroCurrentPoint, char* _outBuffer)
{  
  uint32_t sampleInterval, mVperUnit, prevMicros = 0;
  int32_t result, adcValue, numUnits = 0;
  int16_t temp, samplesCount = 0; // adcTable[ADC_SAMPLES], 
  uint8_t swapped, oldSREG;

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

  // Do not disturb processes by internal routines 
  skipMetricGathering = true;

  // *** Do not disable interrupts here - the system can hang ***

  // disable interrupt
  //oldSREG = SREG; cli();
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
    }
  }
  // enable interrupts
  //SREG = oldSREG;
  skipMetricGathering = false;

  switch (_metric) {
    case SENS_READ_ZC:
    case SENS_READ_DC:
      // Take mean of analogread()
      numUnits = numUnits / ADC_SAMPLES;
      break;

    case SENS_READ_AC:
      break;
  }

//  Serial.print("numUnits: ");   Serial.println(numUnits); 
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
  }

  ltoa(result, _outBuffer, 10);

  return RESULT_IN_BUFFER;
}

