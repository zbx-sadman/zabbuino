// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "spi_bus.h"
#include "spi_max6675.h"

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getMAX6675Metric(const uint8_t _misoPin, const uint8_t _sclkPin, const uint8_t _csPin, const uint8_t _metric, uint32_t* _value)
{
  char stubBuffer;
  return getMAX6675Metric(_misoPin, _sclkPin, _csPin, _metric, &stubBuffer, _value, true);

}

int8_t getMAX6675Metric(const uint8_t _misoPin, const uint8_t _sclkPin, const uint8_t _csPin, const uint8_t _metric, char* _dst)
{
  uint32_t stubValue;
  return getMAX6675Metric(_misoPin, _sclkPin, _csPin, _metric, _dst, &stubValue, false);
}


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
int8_t getMAX6675Metric(const uint8_t _misoPin, const uint8_t _sclkPin, const uint8_t _csPin, const uint8_t _metric, char* _dst, uint32_t* _value, const uint8_t _wantsNumber)
{
  int8_t rc = RESULT_IS_FAIL;
  uint32_t result;

  uint8_t sclkPinBit, misoPinBit;
  volatile uint8_t *sclkPortOutputRegister, *misoPortInputRegister;

    
  if (SENS_READ_TEMP != _metric) { goto finish; }
/* 
  Serial.print("MISO: "); Serial.println(_misoPin);
  Serial.print("SCK: "); Serial.println(_sclkPin);
  Serial.print("CS: "); Serial.println(_csPin);
*/
  pinMode(_misoPin, INPUT);
  pinMode(_sclkPin, OUTPUT); 
  pinMode(_csPin, OUTPUT);

  // Restart conversion
  digitalWrite(_csPin, LOW);
  _delay_ms(1);
  digitalWrite(_csPin, HIGH);

  delay(MAX6675_CONVERSION_TIME);

  sclkPortOutputRegister = portOutputRegister(digitalPinToPort(_sclkPin));
  misoPortInputRegister  = portInputRegister(digitalPinToPort(_misoPin));
  sclkPinBit = digitalPinToBitMask(_sclkPin);
  misoPinBit = digitalPinToBitMask(_misoPin);

  // SPI activate
  digitalWrite(_csPin, LOW);

  _delay_ms(1);

  result = softSpiReadByte(sclkPortOutputRegister, sclkPinBit, misoPortInputRegister, misoPinBit);
  result <<= 8;
  result |= softSpiReadByte(sclkPortOutputRegister, sclkPinBit, misoPortInputRegister, misoPinBit);

  if (result & MAX6675_BITMASK_ID) {
    rc = DEVICE_ERROR_WRONG_ID; 
    goto finish; 
  }

//  Serial.print("raw: "); Serial.println(result, BIN); 

  if (result & MAX6675_BITMASK_TERMOCOUPLE_INPUT) {
    // No thermocouple attached!
    rc = DEVICE_ERROR_CONNECT; 
    goto finish; 
  }

  result >>= 3;

  *_value = result * 25;
//  Serial.print("result (1): "); Serial.println(result); 
//  Serial.print("result (2): "); Serial.println(((float) *_value) *0.25); 
//  t = *_value * 25;
//  w = int(t/100);
//  f = t-(w*100);
//  Serial.print("result whole: "); Serial.println(w);
//  Serial.print("result fract: "); Serial.println(f);


  if (!_wantsNumber) {
     ltoaf(*_value, _dst, 2);
  }

  rc = RESULT_IS_BUFFERED;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  // SPI deactivate
  digitalWrite(_csPin, HIGH);

  return rc;

}


