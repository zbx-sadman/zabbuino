#ifndef _ZABBUINO_I2C_BMP_H_
#define _ZABBUINO_I2C_BMP_H_

#include "i2c_bus.h"

#define BMP180_I2C_ADDRESS                                      0x77  // I2C address of BMP085/180 sensor

#define BMP280_I2C_ADDRESS_1                                    0x76  // I2C address of BMP280/BME280 sensor
#define BMP280_I2C_ADDRESS_2                                    0x77


#define BMP_REGISTER_CHIPID                                     0xD0

#define BMP_REGISTER_RESET                                      0xE0 // for all BMP or not?
#define BMP_REGISTER_CONTROL                                    0xF4 


#define BMP085_CHIPID                                           0x55
#define BMP180_CHIPID                                           0x55
#define BMP280_CHIPID_1                                         0x56 // Only for engeneering samples (BMP180 datasheet, page 24)
#define BMP280_CHIPID_2                                         0x57 // Only for engeneering samples
#define BMP280_CHIPID_3                                         0x58 // Mass production ID

#define BME280_CHIPID                                           0x60 // 

#define BMP180_ULTRALOWPOWER                                    0x00
#define BMP180_STANDARD                                         0x01
#define BMP180_HIGHRES                                          0x02
#define BMP180_ULTRAHIGHRES                                     0x03

#define BMP180_REGISTER_CAL_AC1                                 0xAA  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC2                                 0xAC  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC3                                 0xAE  // R   Calibration data (16 bits)    
#define BMP180_REGISTER_CAL_AC4                                 0xB0  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC5                                 0xB2  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_AC6                                 0xB4  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_B1                                  0xB6  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_B2                                  0xB8  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_MB                                  0xBA  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_MC                                  0xBC  // R   Calibration data (16 bits)
#define BMP180_REGISTER_CAL_MD                                  0xBE  // R   Calibration data (16 bits)

#define BMP180_REGISTER_TEMPDATA                                0xF6
#define BMP180_REGISTER_PRESSUREDATA                            0xF6
#define BMP180_CMD_READTEMP                                     0x2E
#define BMP180_CMD_READPRESSURE                                 0x34

#define BMP180_READY_MASK                                       0x20  // Sco (register F4h <5>): Start of conversion. 
                                                                      // The value of this bit stays “1” during conversion and is reset to “0” after conversion is 
                                                                      // complete (data registers are filled). 
#define BMP180_READY_TIMEOUT                                    100 // ms

#define BMP280_REGISTER_DIG_T1                                  0x88
#define BMP280_REGISTER_DIG_T2                                  0x8A
#define BMP280_REGISTER_DIG_T3                                  0x8C

#define BMP280_REGISTER_DIG_P1                                  0x8E
#define BMP280_REGISTER_DIG_P2                                  0x90
#define BMP280_REGISTER_DIG_P3                                  0x92
#define BMP280_REGISTER_DIG_P4                                  0x94
#define BMP280_REGISTER_DIG_P5                                  0x96
#define BMP280_REGISTER_DIG_P6                                  0x98
#define BMP280_REGISTER_DIG_P7                                  0x9A
#define BMP280_REGISTER_DIG_P8                                  0x9C
#define BMP280_REGISTER_DIG_P9                                  0x9E

#define BMP280_REGISTER_CAL26                                   0xE1  // R calibration stored in 0xE1-0xF0

#define BMP280_REGISTER_STATUS                                  0xF3
#define BMP280_REGISTER_CONTROL                                 0xF4
#define BMP280_REGISTER_CONFIG                                  0xF5
#define BMP280_REGISTER_PRESSUREDATA                            0xF7
#define BMP280_REGISTER_TEMPDATA                                0xFA

#define BMP280_OVERSAMP_SKIPPED          			0x00
#define BMP280_OVERSAMP_1X               			0x01
#define BMP280_OVERSAMP_2X               			0x02
#define BMP280_OVERSAMP_4X               			0x03
#define BMP280_OVERSAMP_8X               			0x04
#define BMP280_OVERSAMP_16X              			0x05

#define BMP280_ULTRALOWPOWER            			0x00
#define BMP280_LOWPOWER	                 			0x01
#define BMP280_STANDARD      	                		0x02
#define BMP280_HIGHRES          		        	0x03
#define BMP280_ULTRAHIGHRES    			                0x04

#define BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE          	BMP280_OVERSAMP_1X
#define BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE       	BMP280_OVERSAMP_1X

#define BMP280_LOWPOWER_OVERSAMP_PRESSURE	         	BMP280_OVERSAMP_2X
#define BMP280_LOWPOWER_OVERSAMP_TEMPERATURE	         	BMP280_OVERSAMP_1X

#define BMP280_STANDARD_OVERSAMP_PRESSURE     	                BMP280_OVERSAMP_4X
#define BMP280_STANDARD_OVERSAMP_TEMPERATURE  	                BMP280_OVERSAMP_1X

#define BMP280_HIGHRES_OVERSAMP_PRESSURE         	        BMP280_OVERSAMP_8X
#define BMP280_HIGHRES_OVERSAMP_TEMPERATURE      	        BMP280_OVERSAMP_1X

#define BMP280_ULTRAHIGHRES_OVERSAMP_PRESSURE       	        BMP280_OVERSAMP_16X
#define BMP280_ULTRAHIGHRES_OVERSAMP_TEMPERATURE    	        BMP280_OVERSAMP_2X

#define BMP280_SLEEP_MODE                    			0x00
#define BMP280_FORCED_MODE                   			0x01
#define BMP280_NORMAL_MODE                   			0x03

#define BMP280_FILTER_COEFF_OFF                                 0x00
#define BMP280_FILTER_COEFF_2                                   0x01
#define BMP280_FILTER_COEFF_4                                   0x02
#define BMP280_FILTER_COEFF_8                                   0x03
#define BMP280_FILTER_COEFF_16                                  0x04

#define BMP280_STANDBY_TIME_1_MS                                0x00
#define BMP280_STANDBY_TIME_63_MS                               0x01
#define BMP280_STANDBY_TIME_125_MS                              0x02
#define BMP280_STANDBY_TIME_250_MS                              0x03
#define BMP280_STANDBY_TIME_500_MS                              0x04
#define BMP280_STANDBY_TIME_1000_MS                             0x05
#define BMP280_STANDBY_TIME_2000_MS                             0x06
#define BMP280_STANDBY_TIME_4000_MS                             0x07

#define BMP280_READY_MASK                                       0x09 // Byte 0 + Byte 3 must be equial 0 if BMP280 do not busy
#define BMP280_READY_TIMEOUT                                    100 // ms

// BME280 additional registers and constants
#define BME280_STANDARD_OVERSAMP_HUMIDITY 	                BMP280_OVERSAMP_1X
#define BME280_REGISTER_DIG_H1                                  0xA1
#define BME280_REGISTER_DIG_H2                                  0xE1
#define BME280_REGISTER_DIG_H3                                  0xE3
#define BME280_REGISTER_DIG_H4                                  0xE4
#define BME280_REGISTER_DIG_H5                                  0xE5
#define BME280_REGISTER_DIG_H6                                  0xE7

#define BME280_REGISTER_CONTROLHUMID                            0xF2
#define BME280_REGISTER_HUMIDDATA                               0xFD

#define T_INIT_MAX						20   // 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX					37   // 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX					10   // 10/16 = 0.625 ms
#define T_SETUP_HUMIDITY_MAX					10   // 10/16 = 0.625 ms



/*****************************************************************************************************************************
*
*   Call the subroutine (based on sensor ID) for obtaining a metric of sensor 
*
*   Returns: 
*     - result code of the called subroutine 
*     - DEVICE_ERROR_CONNECT on connection error
*     - RESULT_IS_FAIL if unknown chip ID found
*
*****************************************************************************************************************************/
int8_t getBMPMetric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint8_t _overSampling, const uint8_t _filterCoef, const uint8_t _metric, uint32_t* _value);
int8_t getBMPMetric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint8_t _overSampling, const uint8_t _filterCoef, const uint8_t _metric, char* _dst);

int8_t getBMPMetric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint8_t _overSampling, const uint8_t _filterCoef, const uint8_t _metric, char *_dst, int32_t* _value, const uint8_t _wantsNumber);

/*****************************************************************************************************************************
*
*  Read specified metric's value of the BMP280/BME280 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if sensor do not ready to work
*
*****************************************************************************************************************************/
int8_t getBMP280Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, const uint8_t _overSampling, uint8_t _filterCoef, const uint8_t _metric, char *_dst, int32_t* _value, const uint8_t _wantsNumber);

/*****************************************************************************************************************************
*
*   Read specified metric's value of the BMP180/BMP085 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if sensor do not ready to work
*
*****************************************************************************************************************************/
int8_t getBMP180Metric(SoftwareWire* _softTWI, uint8_t _i2cAddress, uint8_t _overSampling, const uint8_t _metric, char *_dst, int32_t* _value, const uint8_t _wantsNumber);


#endif // #ifndef _ZABBUINO_I2C_BMP_H_