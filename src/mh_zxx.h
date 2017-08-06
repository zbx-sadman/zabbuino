#ifndef _ZABBUINO_MH_ZXX_H_
#define _ZABBUINO_MH_ZXX_H_

/*

*/

#define MH_ZXX_UART_SPEED                                       (9600)    // baud
#define MH_ZXX_CMD_GAS_CONCENTRATION                            (0x86)
#define MH_ZXX_PACKET_SIZE                                      (0x09)    // bytes
#define MH_ZXX_DEFAULT_READ_TIMEOUT                             (1000UL)

#define MH_ZXX_STARTING_BYTE                                    (0)
#define MH_ZXX_SENSOR_NUMBER                                    (1)
#define MH_ZXX_CMD                                              (2)
// ...
#define MH_ZXX_CRC                                              (8)

#define MH_ZXX_GAS_CONCENTRATION_HIGH_BYTE                      (2)
#define MH_ZXX_GAS_CONCENTRATION_LOW_BYTE                       (3)


#define MH_ZXX_CYCLE_TIME                                       (2008UL) // Two cycle max - first for wait new full cycle, second - for metric read
#define MH_ZXX_STAGE_WAIT_FOR_LOW                               (0)
#define MH_ZXX_STAGE_WAIT_FOR_HIGH                              (1)
#define MH_ZXX_STAGE_COUNT_FOR_HIGH                             (2)
#define MH_ZXX_STAGE_COUNT_FOR_LOW                              (3)
#define MH_ZXX_STAGE_CYCLE_FINISHED                             (4)


/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getMHZxxMetricUART(const uint8_t, const uint8_t, int32_t*);
int8_t getMHZxxMetricUART(const uint8_t, const uint8_t, uint8_t*);

int8_t getMHZxxMetricPWM(const uint8_t, const uint16_t, int32_t*);
int8_t getMHZxxMetricPWM(const uint8_t, const uint16_t, uint8_t*);


/*****************************************************************************************************************************
*
*  Read specified metric's value of the Winsen MH-Zxx CO2 sensor via PWM, put it to output buffer on success. 
*
*  Returns: 
*    - RESULT_IN_BUFFER on success
*    - DEVICE_ERROR_ACK_L
*    - DEVICE_ERROR_ACK_H
*    - DEVICE_ERROR_TIMEOUT if sensor stops answer to the request
*
*****************************************************************************************************************************/
int8_t getMHZxxMetricPWM(const uint8_t _pin, const uint16_t _range, uint8_t* _dst, int32_t* _value, const uint8_t _wantsNumber = false);


/*****************************************************************************************************************************
*
*   Read specified metric's value of the Winsen MH-Zxx CO2 sensor via UART, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*
*****************************************************************************************************************************/
int8_t getMHZxxMetricUART(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _dst, int32_t* _value, const uint8_t _wantsNumber = false);

#endif // #ifndef _ZABBUINO_MH_ZXX_H_