#pragma once

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                                                MEGATEC-COMPATIBLE UPS SECTION
 
   http://networkupstools.org/protocols/megatec.html

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

#define MEGATEC_UPS_UART_SPEED                (2400)   // Megatec-compatible UPS works on 2400 bps
#define MEGATEC_MAX_ANSWER_LENGTH             (50)     // Read no more 50 chars from UPS
#define MEGATEC_DEFAULT_READ_TIMEOUT          (1000UL)

/*****************************************************************************************************************************
*
*   Read values of the specified metric from the Megatec-compatible UPS, put it to output buffer on success. 
*
*   Not tested yet.
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*
*****************************************************************************************************************************/
int8_t getMegatecUPSMetric(const uint8_t, const uint8_t, char*, uint8_t, char*);

