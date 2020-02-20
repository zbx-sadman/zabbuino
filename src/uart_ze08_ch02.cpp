// Config & common included files
#include "sys_includes.h"

#include <SoftwareSerial.h>

#include "service.h"
#include "system.h"

#include "uart_bus.h"
#include "winsen.h"
#include "uart_ze08_ch02.h"


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

int8_t getZe08Ch2OMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, uint32_t* _value) {
  uint8_t recievedBytes, crc;
  uint16_t rc = DEVICE_ERROR_TIMEOUT;

  Ze08Ch2OData_t Ze08Ch2OData;
  uint8_t* ptrRawBuffer = (uint8_t*) &Ze08Ch2OData;

  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(WINSEN_ZE08_CH2O_UART_SPEED);

  recievedBytes = winsenUartRecieve(swSerial, WINSEN_ZE08_CH2O_UART_DEFAULT_READ_TIMEOUT, ptrRawBuffer, sizeof(Ze08Ch2OData));

  // Reading is not finished sucessfully: not all bytes recieved or wrong Gas ID / Unit ID contained in the packet
  if (recievedBytes < sizeof(Ze08Ch2OData) ||
      WINSEN_ZE08_CH2O_UART_GAS_NAME != Ze08Ch2OData.gasName ||
      WINSEN_ZE08_CH2O_UART_GAS_UNIT != Ze08Ch2OData.gasUnit) {
    goto finish;
  }
  // Calculate checksum.
  //Start byte & recieved checksum is not taken in account. The first one is dropped in the read procedure and the second one just will skipped in calculation

  // Try to crc whole buffer?
  crc = winsenCalcCrc(ptrRawBuffer, sizeof(Ze08Ch2OData) - 1);
  //Serial.print("\nRecieved / calculated checksum: 0x"); Serial.print(Ze08Ch2OData.checkSum, HEX); Serial.print(" / 0x"); Serial.println(checkSum, HEX);

  *_value = 0x00;

  if (crc != Ze08Ch2OData.crc) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }

  // rc = (Ze08Ch2OData.fullRangeHighByte << 0x08) + Ze08Ch2OData.fullRangeLowByte; Serial.print("Full range (ppb): "); Serial.println(rc);

  switch (_metric) {
     case SENS_READ_CH2O:  { *_value = (Ze08Ch2OData.concentrationHighByte << 0x08) + Ze08Ch2OData.concentrationLowByte; break; }
     default:              { rc = DEVICE_ERROR_NOT_SUPPORTED; goto finish; }
  }

  rc = RESULT_IS_UNSIGNED_VALUE;

finish:
  swSerial.~SoftwareSerial();
  return rc;
}

