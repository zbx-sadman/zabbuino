#pragma once
/*


*/
        
#define TSL2561_I2C_ADDRESS                                     (0x39)

//  https://www.mouser.com/ds/2/588/TSL2560-61_DS000110_2-00-932616.pdf
// PARTNO - Bits 7:4 
// 0000 TSL2560CS
// 0001 TSL2561CS
// 0100 TSL2560T/FN/CL
// 0101 TSL2561T/FN/CL

#define TSL2561_PARTNO_TSL2560CS                                (B00000000)
#define TSL2561_PARTNO_TSL2561CS                                (B00010000)
#define TSL2561_PARTNO_TSL2560T                                 (B01000000)
#define TSL2561_PARTNO_TSL2561T                                 (B01010000)
#define TSL2561_PARTNO_MASK                                     (B11110000)

#define TSL2561_TYPE_T                                          (0x00)
#define TSL2561_TYPE_CS                                         (0x01)

//#define TSL2561_TYPE                                            TSL2561_TYPE_T


#define TSL2561_CHANNEL_VISIBLE                                 (0x02)
#define TSL2561_CHANNEL_INFRARED                                (0x01)
#define TSL2561_CHANNEL_FULLSPECTRUM                            (0x00)

#define TSL2561_INTEGRATION_TIME_13MS                           (13)
#define TSL2561_INTEGRATION_TIME_101MS                          (101)
#define TSL2561_INTEGRATION_TIME_402MS                          (402)

#define TSL2561_INTEGRATION_TIME_13MS_DELAY                     (15)
#define TSL2561_INTEGRATION_TIME_101MS_DELAY                    (110)
#define TSL2561_INTEGRATION_TIME_402MS_DELAY                    (410)

#define TSL2561_INTEGRATION_TIME_13MS_BITS                      (B00000000)
#define TSL2561_INTEGRATION_TIME_101MS_BITS                     (B00000001)
#define TSL2561_INTEGRATION_TIME_402MS_BITS                     (B00000010)

#define TSL2561_GAIN_1X                                         (0x01)
#define TSL2561_GAIN_16X                                        (0x10)                                                        

#define TSL2561_GAIN_1X_BITS                                    (B00000000)
#define TSL2561_GAIN_16X_BITS                                   (B00010000)                                                             

#define TSL2561_REG_CONTROL                                     (0x00)
#define TSL2561_REG_TIMING                                      (0x01)
#define TSL2561_REG_INTERRUPT_CONTROL                           (0x06)
#define TSL2561_REG_ID                                          (0x0A)

#define TSL2561_ADC_CHANNEL00_LOW                               (0x0C)
#define TSL2561_ADC_CHANNEL00_HIGH                              (0x0D)
#define TSL2561_ADC_CHANNEL01_LOW                               (0x0E)
#define TSL2561_ADC_CHANNEL01_HIGH                              (0x0F)

#define TSL2561_POWEROFF                                        (0x00)
#define TSL2561_POWERON                                         (0x03)

#define TSL2561_INTERRUPT_DISABLED                              (0x00)

#define TSL2561_COMMAND_BIT                                     (0x80)    ///< Must be 1
#define TSL2561_CLEAR_BIT                                       (0x40)    ///< Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT                                        (0x20)    ///< 1 = read/write word (rather than byte)
#define TSL2561_BLOCK_BIT                                       (0x10)    ///< 1 = using block read/write
                                                              
#define TSL2561_CLIPPING_13MS                                   (4900)    ///< # Counts that trigger a change in gain/integration
#define TSL2561_CLIPPING_101MS                                  (37000)   ///< # Counts that trigger a change in gain/integration
#define TSL2561_CLIPPING_402MS                                  (65000)   ///< # Counts that trigger a change in gain/integration


#define TSL2561_LUX_SCALE                                       (14)      // scale by 2^14
#define TSL2561_RATIO_SCALE                                     (9)       // scale ratio by 2^9

//
// Integration time scaling factors
//
#define TSL2561_CH_SCALE                                        (10)      // scale channel values by 2^10
#define TSL2561_CHSCALE_TINT0                                   (0x7517)  // 322/11 * 2^CH_SCALE
#define TSL2561_CHSCALE_TINT1                                   (0x0fe7)  // 322/81 * 2^CH_SCALE

//
// T Package coefficients
//
// For Ch1/Ch0=0.00 to 0.50
// Lux/Ch0=0.0304?0.062*((Ch1/Ch0)^1.4)
// piecewise approximation
// For Ch1/Ch0=0.00 to 0.125:
// Lux/Ch0=0.0304?0.0272*(Ch1/Ch0)
//
// For Ch1/Ch0=0.125 to 0.250:
// Lux/Ch0=0.0325?0.0440*(Ch1/Ch0)
//
// For Ch1/Ch0=0.250 to 0.375:
// Lux/Ch0=0.0351?0.0544*(Ch1/Ch0)
//
// For Ch1/Ch0=0.375 to 0.50:
// Lux/Ch0=0.0381?0.0624*(Ch1/Ch0)
//
// For Ch1/Ch0=0.50 to 0.61:
// Lux/Ch0=0.0224?0.031*(Ch1/Ch0)
//
// For Ch1/Ch0=0.61 to 0.80:
// Lux/Ch0=0.0128?0.0153*(Ch1/Ch0)
//
// For Ch1/Ch0=0.80 to 1.30:
// Lux/Ch0=0.00146?0.00112*(Ch1/Ch0)
//
// For Ch1/Ch0>1.3:
// Lux/Ch0=0
//
#define TSL2561_K1T                                             (0x0040) // 0.125 * 2^RATIO_SCALE
#define TSL2561_B1T                                             (0x01f2) // 0.0304 * 2^LUX_SCALE
#define TSL2561_M1T                                             (0x01be) // 0.0272 * 2^LUX_SCALE
#define TSL2561_K2T                                             (0x0080) // 0.250 * 2^RATIO_SCALE
#define TSL2561_B2T                                             (0x0214) // 0.0325 * 2^LUX_SCALE
#define TSL2561_M2T                                             (0x02d1) // 0.0440 * 2^LUX_SCALE
#define TSL2561_K3T                                             (0x00c0) // 0.375 * 2^RATIO_SCALE
#define TSL2561_B3T                                             (0x023f) // 0.0351 * 2^LUX_SCALE
#define TSL2561_M3T                                             (0x037b) // 0.0544 * 2^LUX_SCALE
#define TSL2561_K4T                                             (0x0100) // 0.50 * 2^RATIO_SCALE
#define TSL2561_B4T                                             (0x0270) // 0.0381 * 2^LUX_SCALE
#define TSL2561_M4T                                             (0x03fe) // 0.0624 * 2^LUX_SCALE
#define TSL2561_K5T                                             (0x0138) // 0.61 * 2^RATIO_SCALE
#define TSL2561_B5T                                             (0x016f) // 0.0224 * 2^LUX_SCALE
#define TSL2561_M5T                                             (0x01fc) // 0.0310 * 2^LUX_SCALE
#define TSL2561_K6T                                             (0x019a) // 0.80 * 2^RATIO_SCALE
#define TSL2561_B6T                                             (0x00d2) // 0.0128 * 2^LUX_SCALE
#define TSL2561_M6T                                             (0x00fb) // 0.0153 * 2^LUX_SCALE
#define TSL2561_K7T                                             (0x029a) // 1.3 * 2^RATIO_SCALE
#define TSL2561_B7T                                             (0x0018) // 0.00146 * 2^LUX_SCALE
#define TSL2561_M7T                                             (0x0012) // 0.00112 * 2^LUX_SCALE
#define TSL2561_K8T                                             (0x029a) // 1.3 * 2^RATIO_SCALE
#define TSL2561_B8T                                             (0x0000) // 0.000 * 2^LUX_SCALE
#define TSL2561_M8T                                             (0x0000) // 0.000 * 2^LUX_SCALE
                           
//
// CS package coefficients
//
// For 0 <= Ch1/Ch0 <= 0.52
// Lux/Ch0 = 0.0315?0.0593*((Ch1/Ch0)^1.4)
// piecewise approximation
// For 0 <= Ch1/Ch0 <= 0.13
// Lux/Ch0 = 0.0315?0.0262*(Ch1/Ch0)
// For 0.13 <= Ch1/Ch0 <= 0.26
// Lux/Ch0 = 0.0337?0.0430*(Ch1/Ch0)
// For 0.26 <= Ch1/Ch0 <= 0.39
// Lux/Ch0 = 0.0363?0.0529*(Ch1/Ch0)
// For 0.39 <= Ch1/Ch0 <= 0.52
// Lux/Ch0 = 0.0392?0.0605*(Ch1/Ch0)
// For 0.52 < Ch1/Ch0 <= 0.65
// Lux/Ch0 = 0.0229?0.0291*(Ch1/Ch0)
// For 0.65 < Ch1/Ch0 <= 0.80
// Lux/Ch0 = 0.00157?0.00180*(Ch1/Ch0)
// For 0.80 < Ch1/Ch0 <= 1.30
// Lux/Ch0 = 0.00338?0.00260*(Ch1/Ch0)
// For Ch1/Ch0 > 1.30
// Lux = 0
//
#define TSL2561_K1C                                             (0x0043) // 0.130 * 2^RATIO_SCALE
#define TSL2561_B1C                                             (0x0204) // 0.0315 * 2^LUX_SCALE
#define TSL2561_M1C                                             (0x01ad) // 0.0262 * 2^LUX_SCALE
#define TSL2561_K2C                                             (0x0085) // 0.260 * 2^RATIO_SCALE
#define TSL2561_B2C                                             (0x0228) // 0.0337 * 2^LUX_SCALE
#define TSL2561_M2C                                             (0x02c1) // 0.0430 * 2^LUX_SCALE
#define TSL2561_K3C                                             (0x00c8) // 0.390 * 2^RATIO_SCALE
#define TSL2561_B3C                                             (0x0253) // 0.0363 * 2^LUX_SCALE
#define TSL2561_M3C                                             (0x0363) // 0.0529 * 2^LUX_SCALE
#define TSL2561_K4C                                             (0x010a) // 0.520 * 2^RATIO_SCALE
#define TSL2561_B4C                                             (0x0282) // 0.0392 * 2^LUX_SCALE
#define TSL2561_M4C                                             (0x03df) // 0.0605 * 2^LUX_SCALE
#define TSL2561_K5C                                             (0x014d) // 0.65 * 2^RATIO_SCALE
#define TSL2561_B5C                                             (0x0177) // 0.0229 * 2^LUX_SCALE
#define TSL2561_M5C                                             (0x01dd) // 0.0291 * 2^LUX_SCALE
#define TSL2561_K6C                                             (0x019a) // 0.80 * 2^RATIO_SCALE
#define TSL2561_B6C                                             (0x0101) // 0.0157 * 2^LUX_SCALE
#define TSL2561_M6C                                             (0x0127) // 0.0180 * 2^LUX_SCALE
#define TSL2561_K7C                                             (0x029a) // 1.3 * 2^RATIO_SCALE
#define TSL2561_B7C                                             (0x0037) // 0.00338 * 2^LUX_SCALE
#define TSL2561_M7C                                             (0x002b) // 0.00260 * 2^LUX_SCALE
#define TSL2561_K8C                                             (0x029a) // 1.3 * 2^RATIO_SCALE
#define TSL2561_B8C                                             (0x0000) // 0.000 * 2^LUX_SCALE
#define TSL2561_M8C                                             (0x0000) // 0.000 * 2^LUX_SCALE



/*****************************************************************************************************************************
*
*  Read specified metric's value of the TSL2561 sensor, put it to specified variable's address on success.
*
*  Returns: 
*    - RESULT_IS_UNSIGNED_VALUE    on success 
*    - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*    - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*    - DEVICE_ERROR_CONNECT        on connection error
*
*****************************************************************************************************************************/
int8_t getTSL2561Metric(SoftwareTWI*, uint8_t, const uint16_t, const uint8_t, const uint8_t, uint32_t*);
