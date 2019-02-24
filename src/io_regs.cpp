// Config & common included files
#include "sys_includes.h"

#include <util/atomic.h>

#include "io_regs.h"

/*****************************************************************************************************************************
*
*   Set I/O port mode
*
*   Returns:
*     - none
*
**************************************************************************************************************************** */
int8_t setPortMode(const uint8_t _port, const uint8_t _mode, const uint8_t _pullup)
{
  volatile uint8_t *modeRegister, *pullUpRegister;

  // Input/Output toggle register
  modeRegister = portModeRegister(_port);
  if (NOT_A_PORT == modeRegister) return RESULT_IS_FAIL;

  // pull-up port register 
  pullUpRegister = portOutputRegister(_port);

  // Port write transaction
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  
    *modeRegister |= _mode;
    *pullUpRegister |= _pullup;
  }
  return RESULT_IS_OK;
}

/*****************************************************************************************************************************
*
*   Write one byte to I/O port
*
*   Returns:
*     - none
*
**************************************************************************************************************************** */
int8_t writeToPort(const uint8_t _port, const uint8_t _value)
{
  volatile uint8_t *portRegister;

  portRegister = portOutputRegister(_port);
  if (NOT_A_PORT == portRegister) return RESULT_IS_FAIL;
  // Port write transaction
  // Use protection mask when write to port for saving some pins state
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  
    *portRegister = (*portRegister & port_protect[_port]) | (_value & ~port_protect[_port]);
  }
  return RESULT_IS_OK;
}


/*****************************************************************************************************************************
*
*   Pin protection testing. Refer to defaults.h -> port_protect[] array
*
*   Returns:
*     - true on succes (pin is safe)
*     - false on fail (pin is unsafe)
*
**************************************************************************************************************************** */
uint8_t isSafePin(const uint8_t _pin)
{
  // Taking pin's correspondent bit 
  uint8_t result = digitalPinToBitMask(_pin);
  if (NOT_A_PIN == result) return false;
  // Protection ckecking
  result &= ~port_protect[digitalPinToPort(_pin)];
  // pinmask=B00100000, safemask=B11011100. result = B00100000 & ~B11011100 = B00100000 & B00100011 = B00100000. B00100000 > 0, pin is safe (not protected)
  // pinmask=B00100000, safemask=B11111100. result = B00100000 & ~B11111100 = B00100000 & B00000011 = B00000000. B00000000 == 0, pin is unsafe (protected)
  return (result ? 1 : 0);
  //return (bool) result;
}



