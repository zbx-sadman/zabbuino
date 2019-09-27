#pragma once

/*
http://style.winsensor.com/pro_pdf/MH-Z19B.pdf  MH-Z19B new datasheet (cheenese)
*/

#define MH_ZXX_UART_SPEED                                       (9600)    // baud
#define MH_ZXX_CMD_GAS_CONCENTRATION                            (0x86)
#define MH_ZXX_PACKET_SIZE                                      (0x09)    // bytes
#define MH_ZXX_DEFAULT_READ_TIMEOUT                             (1000UL)
#define MH_ZXX_PREHEAT_TIMEOUT                                  (20000UL)  // Pre-heat sensor time. No reads are made, returns MH_ZXX_PREHEAT_GAS_CONCENTRATION value  
                                                                           // On datasheet pre-heat time is 3 min, but less time can be used too 
                                                                           // Increase it to avoid reading or measuring errors if need. 

#define MH_ZXX_PREHEAT_GAS_CONCENTRATION                        (399)      // The value which will be returs on MH_ZXX_PREHEAT_TIMEOUT

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
/*
int8_t getMHZxxMetricUART(const uint8_t, const uint8_t, int32_t*);
int8_t getMHZxxMetricUART(const uint8_t, const uint8_t, uint8_t*);

int8_t getMHZxxMetricPWM(const uint8_t, const uint16_t, int32_t*);
int8_t getMHZxxMetricPWM(const uint8_t, const uint16_t, uint8_t*);
*/

/*****************************************************************************************************************************
*
*  Read specified metric's value of the Winsen MH-Zxx CO2 sensor via PWM, put it to output buffer on success. 
*
*  Returns: 
*    - RESULT_IS_BUFFERED on success
*    - DEVICE_ERROR_ACK_L
*    - DEVICE_ERROR_ACK_H
*    - DEVICE_ERROR_TIMEOUT if sensor stops answer to the request
*
*****************************************************************************************************************************/
int8_t getMHZxxMetricPWM(const uint8_t _pin, const uint16_t _range, int32_t* _value);


/*****************************************************************************************************************************
*
*   Read specified metric's value of the Winsen MH-Zxx CO2 sensor via UART, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*
*****************************************************************************************************************************/
int8_t getMHZxxMetricUART(const uint8_t _rxPin, const uint8_t _txPin, int32_t* _value);

