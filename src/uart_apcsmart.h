#ifndef _ZABBUINO_UART_APCSMART_H_
#define _ZABBUINO_UART_APCSMART_H_

#include "uart_bus.h"

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         APC SMART-UPS SECTION
 
   Despite the lack of official information from APC, this table has been constructed. It’s standard RS-232 serial communications at 2400 bps/8N1. 
   Don’t rush the UPS while transmitting or it may stop talking to you. This isn’t a problem with the normal single character queries, but it really 
   does matter for multi-char things like "@000". Sprinkle a few calls to usleep() in your code and everything will work a lot better.
   http://networkupstools.org/protocols/apcsmart.html

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

#define APC_UPS_UART_SPEED                2400 // APC UPS works on 2400 baud speed
#define APC_MAX_ANSWER_LENGTH             30   // Read no more 30 chars from UPS
#define APC_DEFAULT_READ_TIMEOUT          1000L

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the APC Smart UPS, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*
*****************************************************************************************************************************/
int8_t getAPCSmartUPSMetric(const uint8_t, const uint8_t, uint8_t*, uint8_t*);

#endif // #ifndef _ZABBUINO_UART_APCSMART_H_