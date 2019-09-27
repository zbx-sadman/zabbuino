#pragma once

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

Based on: https://github.com/olehs/PZEM004T
version 1.0 is used

*/

#define PZEM_UART_SPEED                    (9600) // PZEM-004 works on 9600 bps

#define PZEM_CMD_VOLTAGE                   (0xB0)
#define PZEM_CMD_CURRENT                   (0xB1)
#define PZEM_CMD_POWER                     (0xB2)
#define PZEM_CMD_ENERGY                    (0xB3)
#define PZEM_CMD_SETADDR                   (0xB4)
#define PZEM_PACKET_SIZE                   (0x07)
#define PZEM_DEFAULT_READ_TIMEOUT          (1000UL)

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Peacefair PZEM-004 Energy Meter, put it to output buffer on success. 
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success when POWER/ENERGY metric specified
*    - RESULT_IS_FLOAT_02_DIGIT    on success when CURRENT(AC) metric specified
*    - RESULT_IS_FLOAT_01_DIGIT    on success when VOLTAGE metric specified
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_WRONG_ANSWER   on "error" answer
*    - DEVICE_ERROR_CHECKSUM       on bad checksum recieved
*
*****************************************************************************************************************************/
int8_t getPZEM004Metric(const uint8_t, const uint8_t, const char*, uint8_t, int32_t*);
