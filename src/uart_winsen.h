#pragma once
/*

Model name: ZE03-X-X-X
"Z" means module, "E" means electrochemical type sensor, "03" means version, the first X means gas type, the second X means detection range, 
the third X means output way.
Eg. "ZE03-CO-(0-1000)ppm-UART/VDC","ZE03-O2-(0-25)%VOL-UART/VDC"


ZP14 Combustible Gas Detection Module
Gas name: 0x01 is for CH4, while Unit ppm: 0x03 is for ppm. Concentration(High Byte): 
The highest bit(bit 8) is for sensor fault judgment; 
bit 7 is for sensor concentration judgment. 
Sensor fault judgment: 1 is for sensor failure, 0 is for no failure. 
Sensor concentration judgment: 1 is for concentration over alarm-point, 0 is for under alarm-point. 
Gas concentration value = The low 6 bit of High Byte*256+Low Byte
https://www.winsen-sensor.com/d/files/ZP14.pdf

ZC05                           
Gas name: 0x01 is for CH4.
Concentration (High Byte): The highest bit(bit 8) is for sensor fault judgment;
Sensor fault judgment: 1 is for sensor failure, 0 is for no failure.
Gas concentration value = The low 5 bit of High Byte*256+Low Byte
https://www.winsen-sensor.com/d/files/ZC05.pdf

ZE16B-CO (active)
Gas concentration value = High Byte*256+Low Byte
Please note that in the above calculation formula, the byte4 and byte5 means the decimalism value changed
from hexadecimal. For example: Original byte4 is 0x01 and original byte5 is 0x2C.
01 is hexadecimal and it is 1 after changing to decimalism.
2C is hexadecimal and it is 44 after changing to decimalism.
So, concentration= 1x256+44=300 ppm
https://www.winsen-sensor.com/d/files/ZE16B-CO.pdf

ZE15-CO (active+Q&A)
Gas name: 0x04 is for CO.
Concentration (High Byte): The highest bit(bit 8) is for sensor fault judgment;
Note: sensor fault judgment: 1 is for sensor failure, 0 is for no failure.
Gas concentration value = (The low 5 bit of High Byte*256+Low Byte)*0.1
https://www.winsen-sensor.com/d/files/ZE15-CO.pdf
Number of Digital 0 = 1

ZE03
Q&A only
https://www.winsen-sensor.com/d/files/PDF/Gas%20Sensor%20Module/Industrial%20Application%20Gas%20Sensor%20Module/ZE03%20Electrochemical%20Module%20V2.4.pdf

ZE03-CO (active+Q&A)
https://www.winsen-sensor.com/d/files/PDF/Gas%20Sensor%20Module/CO%20Detection%20Module/ZE03%20Electrochemical%20Module%20V2.4.pdf

ZE11 Electrochemical Detection Module For ETO (active+Q&A), Detection Gas: benzene, dimethyl benzene, ethylene oxide, chloro ethylene, ID = 0x40
https://www.winsen-sensor.com/d/files/PDF/Gas%20Sensor%20Module/Industrial%20Application%20Gas%20Sensor%20Module/ze11-electrochemical-module-manualv1_3.pdf

ZE12 Electrochemical Gas Sensor Module For Atmospheric Monitoring (active+Q&A), Detection Gas: CO, H2S, NO2, SO2, O3, ID = 0x04
https://www.winsen-sensor.com/d/files/PDF/Gas%20Sensor%20Module/Industrial%20Application%20Gas%20Sensor%20Module/ze12-electrochemical-module-manualv1_5.pdf

ZE27-O3, ZE25-O3  (active+Q&A)
https://www.winsen-sensor.com/d/files/ZE27-O3.pdf
https://www.winsen-sensor.com/d/files/ZE25-O3.pdf
Gas concentration(PPB)=(Concentration high byte*256+Concentration low byte).
Number of Digital 0 = 0

https://www.winsen-sensor.com/d/files/ZE14-O3.pdf (active)
Gas concentration value= (Concentration High Byte *256+ Concentration Low Byte)*0.1
Number of Digital 0 = 1


*/

#define WINSEN_UART_SPEED                                       (9600)
#define WINSEN_UART_DEFAULT_READ_TIMEOUT                        (2500UL)
#define WINSEN_UART_START_BYTE                                  (0xFF)
#define WINSEN_UART_PACKET_SIZE                                 (0x09)
#define WINSEN_UART_GET_COMMAND                                 (0x86)

#define WINSEN_GAS_TYPE_ID_CH4                                  (0x01)
#define WINSEN_GAS_TYPE_ID_CH2O                                 (0x17)
#define WINSEN_GAS_TYPE_ID_O3                                   (0x2A)
#define WINSEN_GAS_TYPE_ID_CO                                   (0x04)
#define WINSEN_UNIT_ID_PPM                                      (0x03)
#define WINSEN_UNIT_ID_PPB                                      (0x04)

#define WINSEN_CALCULATION_MASK_DEFAULT                         (0xFF)
#define WINSEN_SENSOR_FAILURE_MASK_DEFAULT                      (0x00)

#define WINSEN_CALCULATION_MASK_ZC05                            (0x1F)
#define WINSEN_CALCULATION_MASK_ZE15                            (0x1F)
#define WINSEN_CALCULATION_MASK_ZP14                            (0x3F)

#define WINSEN_SENSOR_FAILURE_MASK_ZC05                         (0x80)
#define WINSEN_SENSOR_FAILURE_MASK_ZE15                         (0x80)
#define WINSEN_SENSOR_FAILURE_MASK_ZP14                         (0x80)

#define WINSEN_QMODE_PACKET_BYTE_COMMAND_ID                     (0x01)
#define WINSEN_QMODE_PACKET_BYTE_PARTSPER_CONCENTRATION_HIGH    (0x02)
#define WINSEN_QMODE_PACKET_BYTE_PARTSPER_CONCENTRATION_LOW     (0x03)
#define WINSEN_QMODE_PACKET_BYTE_MASS_CONCENTRATION_HIGH        (0x06)
#define WINSEN_QMODE_PACKET_BYTE_MASS_CONCENTRATION_LOW         (0x07)

#define WINSEN_SENSOR_TYPE_ID_MH_ZXX                            (0x01)
#define WINSEN_SENSOR_TYPE_ID_ZE08_CH2O                         (0x02)
#define WINSEN_SENSOR_TYPE_ID_ZE14_O3                           (0x03)
// ZE25-O3 eq ZE27-O3
#define WINSEN_SENSOR_TYPE_ID_ZE25_O3                           (0x04)
#define WINSEN_SENSOR_TYPE_ID_ZP14                              (0x05)
#define WINSEN_SENSOR_TYPE_ID_ZE15_CO                           (0x06)



// ??? Struct must be packed to works properly on non 8-bit MCUs?
typedef struct {
  uint8_t startByte;
  uint8_t gasTypeId;
  uint8_t gasUnitId;
  uint8_t noDecimalByte;
  uint8_t concentrationHighByte, concentrationLowByte;
  uint8_t fullRangeHighByte, fullRangeLowByte;
  uint8_t crc;
} winsenSensorAModeData_t;

// !!! Q&A mode's answer frame is not equal for various type of sensors
// MH-Z14, MH-Z19 data structure
/*
typedef struct {
  uint8_t commandId;
  uint8_t concentrationMassHighByte, concentrationMassLowByte;
  uint8_t unusedNo1, unusedNo2;
  uint8_t concentrationPartsPerHighByte, concentrationPartsPerLowByte;
  uint8_t crc;
} winsenSensorQModeData_t;
*/

uint8_t winsenUartRecieve(Stream&, const uint32_t, uint8_t*, const uint8_t);
uint8_t winsenCalcCrc(uint8_t*, const uint8_t);

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the MH-Zxx sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getQModeMhZxMetric(const uint8_t, const uint8_t, const uint8_t, int32_t*);

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the ZE08-CH2O sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getAModeZe08Ch2OMetric(const uint8_t, const uint8_t, const uint8_t, int32_t*);

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the ZE14-O3/ZE25-O3/ZE27-O3 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getAModeZe14O3Metric(const uint8_t, const uint8_t, const uint8_t, int32_t*);

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the ZE15-CO/ZE16-CO sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getAModeZe15COMetric(const uint8_t, const uint8_t, const uint8_t, int32_t*);
int8_t getAModeZe16COMetric(const uint8_t, const uint8_t, const uint8_t, int32_t*);

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the ZP14 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getAModeZp14Metric(const uint8_t, const uint8_t, const uint8_t, int32_t*);

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Winsen sensors works in Active Mode, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getAModeWinsenMetric(const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t, int32_t*);

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Winsen sensors works in Query Mode, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getQModeWinsenMetric(const uint8_t, const uint8_t, const uint8_t, const uint8_t, int32_t*);
