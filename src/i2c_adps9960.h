#pragma once

/*


*/

#define ADPS9960_I2C_ADDRESS                                    (0x39)

#define APDS9960_REG_ENABLE                                     (0x80)
#define APDS9960_REG_ADC_INTEGRATION_TIME                       (0x81)
#define APDS9960_REG_WAIT_TIME                                  (0x83)
#define APDS9960_REG_ID                                         (0x92)
#define APDS9960_REG_STATUS                                     (0x93)
#define APDS9960_REG_CONFIG_ONE                                 (0x8D)
#define APDS9960_REG_CONTROL_ONE                                (0x8F)
#define APDS9960_REG_CONFIG_TWO                                 (0x90)
#define APDS9960_REG_AICLEAR                                    (0xE7)

#define APDS9960_REG_CDATAL                                     (0x94)
#define APDS9960_REG_CDATAH                                     (0x95)
#define APDS9960_REG_RDATAL                                     (0x96)
#define APDS9960_REG_RDATAH                                     (0x97)
#define APDS9960_REG_GDATAL                                     (0x98)
#define APDS9960_REG_GDATAH                                     (0x99)
#define APDS9960_REG_BDATAL                                     (0x9A)
#define APDS9960_REG_BDATAH                                     (0x9B)  

/* Acceptable device IDs */
#define APDS9960_ID_01                                          (0xAB)
#define APDS9960_ID_02                                          (0x9C)

/* LED Drive values */
#define APDS9960_LED_DRIVE_100MA                                (0x00)
#define APDS9960_LED_DRIVE_50MA                                 (0x01)
#define APDS9960_LED_DRIVE_25MA                                 (0x02)
#define APDS9960_LED_DRIVE_12_5MA                               (0x03)

/* LED Boost values */
#define APDS9960_LED_BOOST_100                                  (0x00)
#define APDS9960_LED_BOOST_150                                  (0x01)
#define APDS9960_LED_BOOST_200                                  (0x02)
#define APDS9960_LED_BOOST_300                                  (0x03)    

/* ALS Gain (AGAIN) values */
#define APDS9960_ALS_GAIN_1X                                    (0x00)
#define APDS9960_ALS_GAIN_4X                                    (0x01)
#define APDS9960_ALS_GAIN_16X                                   (0x02)
#define APDS9960_ALS_GAIN_64X                                   (0x03)

#define APDS9960_POWER_ON                                       (B00000001)
#define APDS9960_ALS_ENABLE                                     (B00000010)
#define APDS9960_PROXIMITY_DETECT_ENABLE                        (B00000100)
#define APSD9960_WAIT_ENABLE                                    (B00001000)
#define APSD9960_ALS_INTERRUPT_ENABLE                           (B00010000)
#define APSD9960_ALS_PROXIMITY_ENABLE                           (B00100000)
#define APSD9960_GESTURE_ENABLE                                 (B01000000)

#define APSD9960_STATUS_AVALID                                  (B00000001)
#define APSD9960_STATUS_PVALID                                  (B00000010)
#define APSD9960_STATUS_GINT                                    (B00000100)
#define APSD9960_STATUS_AINT                                    (B00010000)
#define APSD9960_STATUS_PINT                                    (B00100000)
#define APSD9960_STATUS_PGSAT                                   (B01000000)
#define APSD9960_STATUS_CPSAT                                   (B10000000)

#define APDS9960_MIN_ADC_INTEGRATION_TIME                       (3)
#define APDS9960_MAX_ADC_INTEGRATION_TIME                       (712)

#define APDS9960_DEFAULT_ADC_INTEGRATION_TIME                   (103)
#define APDS9960_DEFAULT_ALS_GAIN                               (0x04)
#define APDS9960_DEFAULT_LED_DRIVE                              (100)

#define APDS9960_DEFAULT_LED_BOOST                              (B00000001)
#define APDS9960_DEFAULT_ALS_SATURATION_INTERRUPT_ENABLE        (B00000010)
#define APDS9960_DEFAULT_PROXIMITY_SATURATION_INTERRUPT_ENABLE  (B00000100)

#define APDS9960_CONVERSION_TIMEOUT                             (1000UL)  // ms


/*****************************************************************************************************************************
*
*  Read specified metric's value of the ADPS9960 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success 
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getADPS9960Metric(SoftwareTWI* _softTWI, uint8_t _i2cAddress, uint16_t _integrationTime, uint8_t _gain, uint8_t _ledDrive, const uint8_t _metric, uint32_t* _value);
