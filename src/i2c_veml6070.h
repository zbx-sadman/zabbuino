#pragma once

/*

  Datasheet: http://www.vishay.com/docs/84277/veml6070.pdf
*/

#define VEML6070_ADDR_H 0x39 ///< High address
#define VEML6070_ADDR_L 0x38 ///< Low address

// Shutdown mode setting
// 0 = disable, 1 = enable
#define VEML6070_CONFIG_SD                                      (0x00)
#define VEML6070_CONFIG_RESERVED                                (0x01)
// Integration time bit 0 & bit 1
#define VEML6070_CONFIG_IT0                                     (0x02)
#define VEML6070_CONFIG_IT1                                     (0x04)
// VEML6070 provides a function for sending an acknowledge signal (ACK) to the host when the value of sensed UV light is over the programmed threshold (ACK_THD) value.
// 0 = disable, 1 = enable
#define VEML6070_CONFIG_ACK_THD                                 (0x08)
#define VEML6070_CONFIG_ACK                                     (0x0F)

// VEML6070’s refresh time can be determined by the RSET value. Cooperating with the command register setting, the designer
// has a flexible way of defining the timing for light data collection. The default refresh time is 1T, (IT1 : IT0) = (0 : 1).
// RSET = 300 kOhm
// (0 : 0) = 1/2T 62.5 ms
// (0 : 1) = 1T 125 ms       <<< default value
// (1 : 0) = 2T 250 ms
// (1 : 1) = 4T 500 ms
//
// RSET = 600 kOhm
// ..
// (0 : 1) = 1T 250 ms       <<< default value
// ..
#define VEML6070_HALF_T                                         (0x00)
#define VEML6070_1_T                                            (0x01)
#define VEML6070_2_T                                            (0x02)
#define VEML6070_4_T                                            (0x03)

#define VEML6070_INIT_TIME_MS                                   (160)


/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getVEML6070Metric(SoftwareWire*, const uint8_t, const uint8_t, char*);
int8_t getVEML6070Metric(SoftwareWire*, const uint8_t, const uint8_t, uint32_t*);

/*****************************************************************************************************************************
*
*  Read specified metric's value of the VEML6070 sensor, put it to output buffer on success. 
*
*  Returns: 
*    - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on test connection error
*     - RESULT_IS_FAIL - on other fails
*
*****************************************************************************************************************************/
int8_t getVEML6070Metric(SoftwareWire*, const uint8_t, char*, uint32_t*, const uint8_t);


