#define SAMPLES                         1000  // for median sampling algo: every sample is int16_t number, total memory consumption is (SAMPLES * 2) bytes

int32_t MeasureVoltage(uint8_t _analogChannel) {  
  uint8_t i, oldADCSRA;
  uint32_t avgADC=0;
  
    /* • Bit 7:6 – REFS[1:0]: Reference Selection Bits
       These bits select the voltage reference for the ADC, as shown in Table 24-3. If these bits are changed during a
       conversion, the change will not go in effect until this conversion is complete (ADIF in ADCSRA is set). The
       internal voltage reference options may not be used if an external reference voltage is being applied to the AREF
       pin.

       Table 24-3. Voltage Reference Selections for ADC
       REFS1 REFS0 Voltage Reference Selection
       0     0     AREF, Internal Vref turned off
       0     1     AVCC with __external capacitor__ at AREF pin
       1     0     Reserved
       1     1     Internal 2.56V Voltage Reference with external capacitor at AREF pin

     • Bit 7 – ADEN: ADC Enable
       Writing this bit to one enables the ADC. By writing it to zero, the ADC is turned off. Turning the ADC off while a
       conversion is in progress, will terminate this conversion.
       
     • Bit 6 – ADSC: ADC Start Conversion
       In Single Conversion mode, write this bit to one to start each conversion. In Free Running mode, write this bit to
       one to start the first conversion. The first conversion after ADSC has been written after the ADC has been
       enabled, or if ADSC is written at the same time as the ADC is enabled, will take 25 ADC clock cycles instead of
       the normal 13. This first conversion performs initialization of the ADC.
       ADSC will read as one as long as a conversion is in progress. When the conversion is complete, it returns to
       zero. Writing zero to this bit has no effect.
       
     • Bits 2:0 – ADPS[2:0]: ADC Prescaler Select Bits
       These bits determine the division factor between the system clock frequency and the input clock to the ADC.
       
  */
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
  // No delay() used because sub can be called from interrupt
  delayMicroseconds(2000);
  avgADC = 1125300L / avgADC; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

  return avgADC;
}

int32_t ACS7XXCurrent(const uint8_t _sensorPin, uint16_t _aRefVoltage,  const uint8_t _metric, const uint8_t _sensitivity, const uint16_t _ZeroCurrentPoint, char* _outBuffer)
{  
  int32_t vcc, result, mVperUnit, numUnits, aRefVoltage;
  int16_t adcTable[SAMPLES], temp;
  uint8_t swapped, i, n;

  pinMode(_sensorPin, INPUT);
  /**** Gathering ****/

// When AREF pin is used - user can point to get reference voltage from its.
#ifdef FEATURE_AREF_ENABLE
  if (DEFAULT == _aRef) {
     // Take internal MCU's VCC
     aRefVoltage = MeasureVoltage(ANALOG_CHAN_VBG);
  } else {
     // Otherwise - use ref voltage on AREF pin that is given in mV
    analogReference(EXTERNAL);
    delay(2);
  }
// AREF is not used - take internal MCU's VCC
#else
    aRefVoltage = MeasureVoltage(ANALOG_CHAN_VBG);
#endif
 
   Serial.print("aRefVoltage: ");
   Serial.println(aRefVoltage);
//  aRefVoltage *= 1000;
    
  numUnits = 0;
  // TODO: Need to rework samples gathering part of code: 1) DC - need do get median of samples array
  switch (_metric) {
    case SENS_READ_ZC:
    case SENS_READ_DC:
     // ************ Simple algo for search arithmethic mean *****************
      for (uint16_t i=0; i < SAMPLES; i++) {
        numUnits += ((int32_t) analogRead(_sensorPin) - _ZeroCurrentPoint) ;
      }
      // Take mean of analogread()
      numUnits = numUnits / SAMPLES;
/*

     // ************ Median filter  *****************
     // reading. Samples number can be odd
      for (i=0; i < SAMPLES; i++) {
        adcTable[i] = ((int16_t) analogRead(_sensorPin) - _ZeroCurrentPoint) ;
        delay(1);
      }
      // sorting: unoptimized bubblesort
      do {
        swapped = false;
        for (i = 1; i < SAMPLES ; i++) {
           if (adcTable[i-1] > adcTable[i]) {
              temp = adcTable[i-1];
              adcTable[i-1] = adcTable[i];
              adcTable[i] = temp;
              swapped = true;
           }
        }
      } while (swapped);

//      for (i=0; i < SAMPLES; i++) {
//        Serial.println(adcTable[i]);
//        delay(1);
//      }

      //calculating
      numUnits = adcTable[(SAMPLES/2)+1];
 */
      break;

    case SENS_READ_AC:
      break;
  }

  /**** Calculation ****/
  
  switch (_metric) {
    case SENS_READ_ZC:
      result = numUnits;
      break;
    case SENS_READ_DC:
    case SENS_READ_AC:
      // 1000 * aRefVoltage - "un-float" procedure
      mVperUnit = (1000 * aRefVoltage) / 1023;
      // I (mA) = (1000 * aRefVoltage / 1023) * units * (1 / 185) =>  I (mA) = mVperUnit * units / SENSITIVITY
      // (VCC / 1024) * units - how many mV give ACS712
      // (1 / SENSITIVITY) - Amps on mV
      result =  mVperUnit * numUnits / _sensitivity;
  }

  ltoa(result, _outBuffer, 10);
  return RESULT_IN_BUFFER;
}

