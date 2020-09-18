#pragma once
//#include <Arduino.h>

#define PCA9685_REG_MODE1                                 (0x00)
#define PCA9685_REG_MODE2                                 (0x01)
#define PCA9685_REG_LED_00                                (0x06)
#define PCA9685_REG_ALL_LED                               (0xFA)

#define PCA9685_REG_MODE1_ALLCALL                         (0x00)
#define PCA9685_REG_MODE1_SUB3                            (0x01)
#define PCA9685_REG_MODE1_SUB2                            (0x02)
#define PCA9685_REG_MODE1_SUB1                            (0x03)
#define PCA9685_REG_MODE1_SLEEP                           (0x04)
#define PCA9685_REG_MODE1_AI                              (0x05)
#define PCA9685_REG_MODE1_EXTCLK                          (0x06)
#define PCA9685_REG_MODE1_RESTART                         (0x07)

#define PCA9685_REG_MODE2_OUTNE                           (0x00) // two bit length
#define PCA9685_REG_MODE2_OUTDRV                          (0x02)
#define PCA9685_REG_MODE2_OCH                             (0x03)
#define PCA9685_REG_MODE2_INVRT                           (0x04)

#define PCA9685_OUTDRV_TOTEM_POLE                         (0x01)
#define PCA9685_OUTDRV_OPEN_DRAIN                         (0x00)

// Outputs must be configured with a totem pole structure as default
// User can omit mode param (it's will eq 0x00), but PCA's OUTDRV bit must be set to 0x01 if "totem pole" mode is need
// _outputMode == 0x00 => bit set to 0x01 (PCA9685_OUTDRV_TOTEM_POLE)
// _outputMode == 0x01 => bit set to 0x00 (PCA9685_OUTDRV_OPEN_DRAIN)
#define PCA9685_MODE_TOTEM_POLE                           (0x00)
#define PCA9685_MODE_OPEN_DRAIN                           (0x01)

#define PCA9685_CHANNEL_COUNT                             (16)
#define PCA9685_CHANNEL_LEDS_ALL                          (-0x01)

/*****************************************************************************************************************************
*
*  Set PCA9685's outputs to the specified state.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success 
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t writePCA9685(SoftwareTWI*, const uint8_t, const int8_t, const uint16_t, const uint16_t, const uint8_t);
