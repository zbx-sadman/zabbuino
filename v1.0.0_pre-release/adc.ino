
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
  delay(2); // Wait for Vref to settle

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
  delay(2); 
  avgADC = 1125300L / avgADC; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

  return avgADC;
}

