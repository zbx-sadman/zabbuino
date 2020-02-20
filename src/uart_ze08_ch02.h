#pragma once
/*


*/

#define WINSEN_ZE08_CH2O_UART_TX_PIN                            (2)
#define WINSEN_ZE08_CH2O_UART_SPEED                             (9600)
#define WINSEN_ZE08_CH2O_UART_DEFAULT_READ_TIMEOUT              (2500UL)
#define WINSEN_ZE08_CH2O_UART_START_BYTE                        (0xFF)
#define WINSEN_ZE08_CH2O_UART_GAS_NAME                          (0x17)
#define WINSEN_ZE08_CH2O_UART_GAS_UNIT                          (0x04)

typedef struct {
  uint8_t gasName;
  uint8_t gasUnit;
  uint8_t noDecimalByte;
  uint8_t concentrationHighByte, concentrationLowByte;
  uint8_t fullRangeHighByte, fullRangeLowByte;
  uint8_t crc;
} Ze08Ch2OData_t;

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Plantower PMS sensors, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getZe08Ch2OMetric(const uint8_t, const uint8_t, const uint8_t, uint32_t*);
