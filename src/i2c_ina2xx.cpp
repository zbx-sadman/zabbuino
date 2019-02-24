// Config & common included files
#include "sys_includes.h"

#include "SoftwareWire/SoftwareWire.h"
#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "i2c_ina2xx.h"

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getINA219Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, uint8_t _voltageRange, uint16_t _maxExpectedCurrent, const uint8_t _metric, uint32_t* _value)
{
  char stubBuffer;
  return getINA219Metric(_softTWI, _i2cAddress, _voltageRange, _maxExpectedCurrent, _metric, &stubBuffer, _value, true);

}

int8_t getINA219Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, uint8_t _voltageRange, uint16_t _maxExpectedCurrent, const uint8_t _metric, char* _dst)
{
  uint32_t stubValue = 0x00;
  return getINA219Metric(_softTWI, _i2cAddress, _voltageRange, _maxExpectedCurrent, _metric, _dst, &stubValue, false);
}


/*****************************************************************************************************************************
*
*   Read specified metric's value of the INA219 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_CONNECT on test connection error
*     - RESULT_IS_FAIL - on other fails
*
*****************************************************************************************************************************/
int8_t getINA219Metric(SoftwareWire* _softTWI, const uint8_t _i2cAddress, uint8_t _voltageRange, uint16_t _maxExpectedCurrent, const uint8_t _metric, char* _dst, uint32_t* _value, const uint8_t _wantsNumber)
{
  int8_t rc = RESULT_IS_FAIL;
  uint16_t result, calValue, configValue;
  uint8_t value[2];
  uint8_t i2cReg, currentDivider_mA, powerDivider_mW;     

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { rc = DEVICE_ERROR_CONNECT; goto finish; }

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

  // !!! Used shunt is R100 (0.1Ohm) !!!

  switch (_voltageRange) {
    case 16: //16V 
      switch (_maxExpectedCurrent) {
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
      switch (_maxExpectedCurrent) {
        // 32V, 1A
        case 1000:                 
          calValue = 8192;
//          calValue = 10240;
          currentDivider_mA = 20; // Current LSB = 50uA per bit (1000/50 = 20)     
//          currentDivider_mA = 25; // Current LSB = 50uA per bit (1000/50 = 20)     
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


  U16ToWire(value, configValue);
  if (0x02 != writeBytesToI2C(_softTWI, _i2cAddress, INA219_REG_CONFIGURATION, value, 0x02)) { goto finish; }

  U16ToWire(value, calValue);
  if (0x02 != writeBytesToI2C(_softTWI, _i2cAddress, INA219_REG_CALIBRATION, value, 0x02)) { goto finish; }

  // Wait ready bit - CNVR == 1 in Bus Voltage Register
  do {
     delay(10);
     if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, INA219_REG_BUS_VOLTAGE, value, 0x02)) { goto finish; }
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
    default:
      goto finish; 
      break;

   }


  if (0x02 != readBytesFromI2C(_softTWI, _i2cAddress, i2cReg, value, 0x02)) { goto finish; }
//  Serial.print("Register: 0x"); Serial.println(i2cReg, HEX);
//  Serial.print("uC register content: "); Serial.print("0x"); Serial.print(value[0], HEX);  Serial.print(" 0x"); Serial.print(value[1], HEX); Serial.println();
  result = WireToU16(value);

//  Serial.print("result raw: 0x"); Serial.print(result, HEX); Serial.print(" => "); Serial.println(result); 

  switch (_metric) {
    case SENS_READ_SHUNT_VOLTAGE:
      *_value = result; 
      break;
    case SENS_READ_BUS_VOLTAGE:
      *_value = (int16_t) (result >> 3) * 4;
      break;
    case SENS_READ_POWER:
      *_value = ((int16_t) result) * powerDivider_mW; 
      break;
    case SENS_READ_DC:
//      if bitRead(result, 15) { result = 0; }        // negative
      *_value = ((int16_t) result)  / currentDivider_mA;
      break;
   }


  if (!_wantsNumber) {
     ltoa(*_value, _dst, 10);
  }

  rc = RESULT_IS_BUFFERED;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  return rc;

}


