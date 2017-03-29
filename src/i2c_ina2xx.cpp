#include "i2c_ina2xx.h"

/*****************************************************************************************************************************
*
*   Read specified metric's value of the SHT2X sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if sensor do not ready to work
*
*****************************************************************************************************************************/
uint8_t getINA219Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, uint8_t _maxVoltage, uint16_t _maxCurrent, const uint8_t _metric,  char* _dst)
{
  int16_t result, calValue, configValue;
  uint8_t value[2];
  uint8_t i2cReg, currentDivider_mA, powerDivider_mW;     

//  if (!isI2CDeviceReady(_i2cAddress)) { return DEVICE_ERROR_CONNECT; }
/*
    Configuration Register (address = 00h) [on reset = 399Fh / B0011100110011111]
      RST  ---  BRNG  PG1  PG0  BADC4  BADC3  BADC2  BADC1  SADC4  SADC3  SADC2  SADC1  MODE3  MODE2  MODE1
       0    0     1    1    1     0      0      1      1     0       0      1      1      1      1      1

    RST: Reset Bit
    Bit 15 Setting this bit to '1' generates a system reset that is the same as power-on reset. Resets all registers to default values; 
    this bit self-clears.
 
    BRNG: Bus Voltage Range
    Bit 13 0 = 16V FSR
    1 = 32V FSR (default value)

    PG: PGA (Shunt Voltage Only)
    Bits 11, 12 Sets PGA gain and range (+/- mV). Note that the PGA defaults to +/-8 (320mV range). Table 4 shows the gain and range for
    the various product gain settings.
    00 - 40 mV, 01 - 80 mV, 10 - 160 mV, 11 - 320 mV

    BADC: BADC Bus ADC Resolution/Averaging
    Bits 7–10 These bits adjust the Bus ADC resolution (9-, 10-, 11-, or 12-bit) or set the number of samples used when
    averaging results for the Bus Voltage Register (02h).

    SADC: SADC Shunt ADC Resolution/Averaging
    Bits 3–6 These bits adjust the Shunt ADC resolution (9-, 10-, 11-, or 12-bit) or set the number of samples used when
    averaging results for the Shunt Voltage Register (01h).
    
    *** BADC (Bus) and SADC (Shunt) ADC resolution/averaging and conversion time settings are shown in Table 5.

    MODE: Operating Mode
    Bits 0–2 Selects continuous, triggered, or power-down mode of operation. These bits default to continuous shunt and bus
    measurement mode. The mode settings are shown in Table 6.
    111 - Shunt and bus, continuous
*/
 
  // I use this switches to avoid calculation with float numbers using
  // But i think that need to rewrite code or add more variants
  switch (_maxVoltage) {
    case 16: //16V 
      switch (_maxCurrent) {
        // 16V, 400mA
        case 400:                    
          calValue = 8192;
          currentDivider_mA = 20;
          powerDivider_mW = 1;
          configValue = INA219_CONFIG_BVOLTAGERANGE_16V |
                        INA219_CONFIG_GAIN_1_40MV |
                        INA219_CONFIG_BADCRES_12BIT |
                        INA219_CONFIG_SADCRES_12BIT_1S_532US |
                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS; 
          break;
        // 16V, 1000mA
        case 1000:                   
          calValue = 8192;
          currentDivider_mA = 20;  
          powerDivider_mW = 1;     
          configValue = INA219_CONFIG_BVOLTAGERANGE_16V |
                        INA219_CONFIG_GAIN_8_320MV |
                        INA219_CONFIG_BADCRES_12BIT |
                        INA219_CONFIG_SADCRES_12BIT_1S_532US |
                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS; 

        // 16V, 2A
        case 2000:                 
        // 16V, 3A
        case 3000:                 
        default:                   
          calValue = 4096;
          currentDivider_mA = 10;  
          powerDivider_mW = 2;     
          configValue = INA219_CONFIG_BVOLTAGERANGE_16V |
                        INA219_CONFIG_GAIN_8_320MV |
                        INA219_CONFIG_BADCRES_12BIT |
                        INA219_CONFIG_SADCRES_12BIT_1S_532US |
                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS; 
      }
      break;
  
    case 32: // 32V 
    default: 
      switch (_maxCurrent) {
        // 32V, 1A
        case 1000:                 
          calValue = 8192;
          currentDivider_mA = 20; // Current LSB = 50uA per bit (1000/50 = 20)     
          powerDivider_mW = 1;                  
          configValue = INA219_CONFIG_BVOLTAGERANGE_32V |
                        INA219_CONFIG_GAIN_8_320MV |
                        INA219_CONFIG_BADCRES_12BIT |
                        INA219_CONFIG_SADCRES_12BIT_1S_532US |
                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS; 
          break;
        // 32V, 2A
        case 2000:                 
        // 32V, 3A
        case 3000:
        default:                  
          calValue = 4096;
          currentDivider_mA = 10;  // Current LSB = 100uA per bit (1000/100 = 10)
          powerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)
          configValue = INA219_CONFIG_BVOLTAGERANGE_32V |
                        INA219_CONFIG_GAIN_8_320MV |
                        INA219_CONFIG_BADCRES_12BIT |
                        INA219_CONFIG_SADCRES_12BIT_1S_532US |
                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
      }

      break;
   }

  writeByteToI2C(_softTWI, INA219_I2C_ADDRESS, INA219_REG_CONFIGURATION, configValue);
  writeByteToI2C(_softTWI, INA219_I2C_ADDRESS, INA219_REG_CALIBRATION, calValue);
 
  // Wait ready bit - CNVR == 1 in Bus Voltage Register
  do {
     delay(10);
     readBytesFromI2C(_softTWI, INA219_I2C_ADDRESS, INA219_REG_BUS_VOLTAGE, value, 2);
  } while (!(value[1] & B00000010)); 


  switch (_metric) {
    case SENS_READ_SHUNT_VOLTAGE:
      i2cReg = INA219_REG_SHUNT_VOLTAGE; 
      break;
    case SENS_READ_BUS_VOLTAGE:
      i2cReg = INA219_REG_BUS_VOLTAGE;
      break;
    case SENS_READ_POWER:
      i2cReg = INA219_REG_POWER;
      break;
    case SENS_READ_DC:
      i2cReg = INA219_REG_CURRENT;
      break;
   }

  readBytesFromI2C(_softTWI, INA219_I2C_ADDRESS, i2cReg, value, 2);
  result = WireToU16(value);

  switch (_metric) {
    case SENS_READ_SHUNT_VOLTAGE:
//      result = result; 
      break;
    case SENS_READ_BUS_VOLTAGE:
      result = (result >> 3) * 4;
      break;
    case SENS_READ_POWER:
      result = result * powerDivider_mW; 
      break;
    case SENS_READ_DC:
//      if bitRead(result, 15) { result = 0; }        // negative
      result = result / currentDivider_mA;          
      break;
   }

  ltoa(result, _dst, 10);
  gatherSystemMetrics(); // Measure memory consumption
  return RESULT_IN_BUFFER;  

}


