// Config & common included files
#include "sys_includes.h"

#if defined(ARDUINO_ARCH_AVR)
  #include <util/atomic.h>
#endif

#include "service.h"
#include "io_regs.h"

int8_t initPinMode() {
  int8_t rc = RESULT_IS_FAIL;
#if defined(ARDUINO_ARCH_AVR)
  uint8_t portNo = arraySize(port_mode);
  if (arraySize(port_pullup) != portNo) { goto finish; }
  while (portNo) {
    portNo--;
    setPortMode(portNo, pgm_read_byte(port_mode + portNo), pgm_read_byte(port_pullup + portNo));
  }
  rc = RESULT_IS_OK;
#elif defined(ARDUINO_ARCH_ESP8266)
  
  rc = RESULT_IS_OK;
  goto finish;
#endif

finish:
  return rc;
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
int8_t setPortMode(const uint8_t _portNo, const uint8_t _mode, const uint8_t _pullup) {
  int8_t rc = RESULT_IS_FAIL;
#if defined(ARDUINO_ARCH_AVR)
  volatile uint8_t *modeRegister, *pullUpRegister;
  // Input/Output toggle register
  modeRegister = portModeRegister(_portNo);
  if (NOT_A_PORT == modeRegister) { goto finish; }

  // pull-up port register 
  pullUpRegister = portOutputRegister(_portNo);
  // Port write transaction
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  
    *modeRegister   |= _mode;
    *pullUpRegister |= _pullup;
  }
  rc = RESULT_IS_OK;
#elif defined(ARDUINO_ARCH_ESP8266)
  __SUPPRESS_WARNING_UNUSED(_portNo);
  __SUPPRESS_WARNING_UNUSED(_mode);
  __SUPPRESS_WARNING_UNUSED(_pullup);
  rc = RESULT_IS_OK;
  goto finish;
#endif

finish:
  return rc; 
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
int8_t writeToPort(const uint8_t _portNo, const uint8_t _value) {
  int8_t rc = RESULT_IS_FAIL;
#if defined(ARDUINO_ARCH_AVR)
  volatile uint8_t *portRegister;
  // port_mode is a cfg_tune.h's variable
  if (arraySize(port_mode) < _portNo) { goto finish; }

  portRegister = portOutputRegister(_portNo);

  if (NOT_A_PORT == portRegister) { goto finish; }
  // Port write transaction
  // Use protection mask when write to port for saving some pins state
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *portRegister = (*portRegister & port_protect[_portNo]) | (_value & ~port_protect[_portNo]); }
  rc = RESULT_IS_OK;
#elif defined(ARDUINO_ARCH_ESP8266)
  __SUPPRESS_WARNING_UNUSED(_portNo);
  __SUPPRESS_WARNING_UNUSED(_value);
  rc = RESULT_IS_OK;
  goto finish;
#endif

finish:
  return rc; 
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
uint8_t isSafePin(const uint8_t _pin) {
  // Taking pin's correspondent bit 
  uint8_t rc = true; 
  // Taking pin's correspondent bit 
  rc = digitalPinToBitMask(_pin);
  // Protection checking
  rc &= ~port_protect[digitalPinToPort(_pin)];
  // pinmask=B00100000, safemask=B11011100. result = B00100000 & ~B11011100 = B00100000 & B00100011 = B00100000. B00100000 > 0, pin is safe (not protected)
  // pinmask=B00100000, safemask=B11111100. result = B00100000 & ~B11111100 = B00100000 & B00000011 = B00000000. B00000000 == 0, pin is unsafe (protected)
  rc = !!rc;

finish:
  return rc; 
}



