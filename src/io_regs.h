#ifndef _ZABBUINO_REGS_H_
#define _ZABBUINO_REGS_H_

#include <util/atomic.h>
#include "../basic.h"
#include "tune.h"


/*****************************************************************************************************************************
*
*   Set I/O port mode
*
*   Returns:
*     - none
*
**************************************************************************************************************************** */
extern void setPortMode(const uint8_t _port, const uint8_t _mode, const uint8_t _pullup);

/*****************************************************************************************************************************
*
*   Write one byte to I/O port
*
*   Returns:
*     - none
*
**************************************************************************************************************************** */
extern void writeToPort(const uint8_t _port, const uint8_t _value);


/*****************************************************************************************************************************
*
*   Pin protection testing. Refer to defaults.h -> port_protect[] array
*
*   Returns:
*     - true on succes (pin is safe)
*     - false on fail (pin is unsafe)
*
**************************************************************************************************************************** */
extern uint8_t isSafePin(const uint8_t _pin);
 
#endif // #ifndef ZabbuinoIOREGS_h