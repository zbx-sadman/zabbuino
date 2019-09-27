// Config & common included files
#include "sys_includes.h"

#include <util/atomic.h>

#include "service.h"

#include "io_regs.h"

int8_t initPortMode() {
  if (arraySize(port_mode) != arraySize(port_pullup)) { return RESULT_IS_FAIL; }
  uint8_t portNo = arraySize(port_mode);
  while (portNo) {
    portNo--;
    // pgm_read_byte(port_mode+port) == pgm_read_byte(&port_mode[port])
    setPortMode(portNo, pgm_read_byte(port_mode + portNo), pgm_read_byte(port_pullup + portNo));
  }
  return RESULT_IS_OK;
}

/*****************************************************************************************************************************
*
*   Set I/O port mode
*
*   Returns:
*     - RESULT_IS_OK   on success
*     - RESULT_IS_FAIL on fail
*
**************************************************************************************************************************** */
int8_t setPortMode(const uint8_t _portNo, const uint8_t _mode, const uint8_t _pullup)
{
  volatile uint8_t *modeRegister, *pullUpRegister;

  // Input/Output toggle register
  modeRegister = portModeRegister(_portNo);
  if (NOT_A_PORT == modeRegister) { return RESULT_IS_FAIL; }

  // pull-up port register 
  pullUpRegister = portOutputRegister(_portNo);

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
*     - RESULT_IS_OK   on success
*     - RESULT_IS_FAIL on fail
*
**************************************************************************************************************************** */
int8_t writeToPort(const uint8_t _portNo, const uint8_t _value)
{
  volatile uint8_t *portRegister;
  // port_mode is a cfg_tune.h's variable
  if (arraySize(port_mode) < _portNo) { return RESULT_IS_FAIL; }

  portRegister = portOutputRegister(_portNo);

  if (NOT_A_PORT == portRegister) { return RESULT_IS_FAIL; }
  // Port write transaction
  // Use protection mask when write to port for saving some pins state
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *portRegister = (*portRegister & port_protect[_portNo]) | (_value & ~port_protect[_portNo]); }
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
  if (NOT_A_PIN == result) { return false; }
  // Protection ckecking
  result &= ~port_protect[digitalPinToPort(_pin)];
  // pinmask=B00100000, safemask=B11011100. result = B00100000 & ~B11011100 = B00100000 & B00100011 = B00100000. B00100000 > 0, pin is safe (not protected)
  // pinmask=B00100000, safemask=B11111100. result = B00100000 & ~B11111100 = B00100000 & B00000011 = B00000000. B00000000 == 0, pin is unsafe (protected)
  return !!result;
}



