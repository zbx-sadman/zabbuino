#ifndef ZabbuinoUART_MEGATEC_h
#define ZabbuinoUART_MEGATEC_h

#include "uart_bus.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                MEGATEC-COMPATIBLE UPS SECTION
 
   http://networkupstools.org/protocols/megatec.html

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

#define MEGATEC_UPS_UART_SPEED                2400 // Megatec-compatible UPS works on 2400 baud speed
#define MEGATEC_MAX_ANSWER_LENGTH             50   // Read no more 50 chars from UPS
#define MEGATEC_DEFAULT_READ_TIMEOUT          1000L

/*****************************************************************************************************************************
*
*   Read values of the specified metric from the Megatec-compatible UPS, put it to output buffer on success. 
*
*   Not tested yet.
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*
*****************************************************************************************************************************/
int8_t getMegatecUPSMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _command, uint8_t _fieldNumber, uint8_t* _dst);

#endif