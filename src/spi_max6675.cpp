// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "spi_bus.h"
#include "spi_max6675.h"

/*****************************************************************************************************************************
*
*   Read specified metric's value of the MAX6675 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on termocouple connection error
*     - DEVICE_ERROR_WRONG_ID on wrong value of MAX6675's ID bit detection
*     - RESULT_IS_FAIL on other fails
*
*****************************************************************************************************************************/
int8_t getMAX6675Metric(const uint8_t _misoPin, const uint8_t _sclkPin, const uint8_t _csPin, const uint8_t _metric, int32_t* _value) {
  int8_t rc = RESULT_IS_FAIL;
  uint32_t result;

  ioRegister_t sclkPinBit, misoPinBit;
  volatile ioRegister_t *sclkPortOutputRegister, *misoPortInputRegister;

    
  if (SENS_READ_TEMP != _metric) { goto finish; }

  pinMode(_misoPin, INPUT);
  pinMode(_sclkPin, OUTPUT); 
  pinMode(_csPin, OUTPUT);

  // Restart conversion
  digitalWrite(_csPin, LOW);
  _delay_ms(0x01);
  digitalWrite(_csPin, HIGH);

  delay(MAX6675_CONVERSION_TIME);

  sclkPortOutputRegister = portOutputRegister(digitalPinToPort(_sclkPin));
  misoPortInputRegister  = portInputRegister(digitalPinToPort(_misoPin));
  sclkPinBit = digitalPinToBitMask(_sclkPin);
  misoPinBit = digitalPinToBitMask(_misoPin);

  // SPI activate
  digitalWrite(_csPin, LOW);

  _delay_ms(0x01);

  result = spiReadByte(sclkPortOutputRegister, sclkPinBit, misoPortInputRegister, misoPinBit);
  result <<= 0x08;
  result |= spiReadByte(sclkPortOutputRegister, sclkPinBit, misoPortInputRegister, misoPinBit);

  if (result & MAX6675_BITMASK_ID) {
    rc = DEVICE_ERROR_WRONG_ID; 
    goto finish; 
  }

//  DEBUG_PORT.print("raw: "); DEBUG_PORT.println(result, BIN); 

  if (result & MAX6675_BITMASK_TERMOCOUPLE_INPUT) {
    // No thermocouple attached!
    rc = DEVICE_ERROR_CONNECT; 
    goto finish; 
  }

  result >>= 0x03;

  *_value = result * 25;

  rc = RESULT_IS_FLOAT_02_DIGIT;

finish:
  gatherSystemMetrics(); // Measure memory consumption
  // SPI deactivate
  digitalWrite(_csPin, HIGH);

  return rc;

}


