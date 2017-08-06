#ifndef _ZABBUINO_MODBUS_H_
#define _ZABBUINO_MODBUS_H_

#define MODBUS_UART_SPEED                                       (9600)    // baud

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
uint8_t getModbusRTUMetric();

#endif