#pragma once
#include "SoftwareWire/SoftwareWire.h"

// 1-st led register address
#define PCA9685_LEDS_START_REG                            (0x06)

// "Set all led registers" address
#define PCA9685_LEDS_ALL_REG                              (0xFA)

#define PCA9685_CHANNEL_COUNT                             (16)
#define PCA9685_CHANNEL_LEDS_ALL                          (-0x01)

// Register value that set FULL_ON or FULL_OFF state on led channel
#define PCA9685_CHANNEL_FULL_STATE                        (4096)

/*****************************************************************************************************************************
*
*   Set sate of the PCA9685 outputs. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on detect of connection error
*     - DEVICE_ERROR_NOT_SUPPORTED on wrong parameters set
*     - RESULT_IS_FAIL on other fails
*
*****************************************************************************************************************************/
int8_t writePCA9685(SoftwareWire*, const uint8_t, const int8_t, const uint16_t, const uint16_t);
