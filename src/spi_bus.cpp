#include "sys_includes.h"
#include "service.h"
#include "system.h"

#include "spi_bus.h"

/*****************************************************************************************************************************
*
*   Read one byte from SPI compatible bus
*
*****************************************************************************************************************************/
uint8_t softSpiReadByte(volatile uint8_t* _sclkPortOutputRegister, const uint8_t _sclkPinBit, volatile uint8_t* _misoPortInputRegister, const uint8_t _misoPinBit) { 
  int8_t i;
  uint8_t value = 0x00;

  for (i=7; i>=0; i--)
  {
    *_sclkPortOutputRegister &= ~_sclkPinBit;
    _delay_ms(1);
    if (*_misoPortInputRegister & _misoPinBit) {
      //set the bit to 0 no matter what
      value |= (1 << i);
    }
    *_sclkPortOutputRegister |= _sclkPinBit;
    _delay_ms(1);
  }

  return value;
}


