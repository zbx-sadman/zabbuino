#pragma once
/*

   wrap_i2c.h : header file which make virtual I2C interface from various physical interface drivers

*/
/*
#if defined(ARDUINO_ARCH_AVR)
#include "SoftwareWire/SoftwareWire.h"
//#define ParentTwiClass SoftwareWire
typedef SoftwareWire SoftwareTWI;


#elif defined(ARDUINO_ARCH_ESP8266)  
#include <Wire.h>
//#define ParentTwiClass TwoWire
typedef TwoWire SoftwareTWI;

#endif
*/
/*
class SoftwareTWI : public ParentTwiClass {
  public:
#if defined(ARDUINO_ARCH_AVR)
#elif defined(ARDUINO_ARCH_ESP8266)  
#endif
};
*/
