// Config & common included files
#include "sys_includes.h"

//#include <SoftwareSerial.h>

#include "service.h"
#include "system.h"

#include "uart_bus.h"
#include "uart_winsen.h"

/*****************************************************************************************************************************
*
*  Winsen's CRC calculate
*  
*****************************************************************************************************************************/
uint8_t winsenCalcCrc(uint8_t* _src, const uint8_t _len) {
  uint8_t crc = 0x00;

  for (uint8_t i = 0x00; i < _len; i++) {
    crc += _src[i];
    //Serial.print(" 0x"); Serial.print(ptrRawBuffer[i], HEX);
  }
  crc = (~crc) + 0x01;
  return crc;
}


/*****************************************************************************************************************************
*
*  Take data from Winsen sensor
*  
*****************************************************************************************************************************/
uint8_t winsenUartRecieve(Stream& _stream, const uint32_t _timeout, uint8_t* _dst, const uint8_t _maxLength) {
  uint8_t headerDetected = false,
          recievedBytes = 0x00;

  uint32_t readStartTime = millis();

  while ((_maxLength > recievedBytes) && (_timeout > millis() - readStartTime)) {
    if (!_stream.available()) {
      continue;
    }
    _dst[recievedBytes] = _stream.read();
 //   Serial.print("winsen: 0x"); Serial.println(_dst[recievedBytes], HEX);

    if (!headerDetected) {
      headerDetected = (WINSEN_UART_START_BYTE == _dst[0x00]);
    } else {
      recievedBytes++;
    }
  }
  return recievedBytes;
}

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the ZE14-O3 sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getZe14O3Metric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
  return getAModeWinsenMetric(_rxPin, _txPin, _metric, WINSEN_GAS_TYPE_ID_O3, WINSEN_SENSOR_FAILURE_MASK_DEFAULT, WINSEN_CALCULATION_MASK_DEFAULT,_value);
}

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the ZE15-CO sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getZe15COMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
  return getAModeWinsenMetric(_rxPin, _txPin, _metric, WINSEN_GAS_TYPE_ID_CO, WINSEN_SENSOR_FAILURE_MASK_ZE15, WINSEN_CALCULATION_MASK_ZE15, _value);
}

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the ZE15-CO sensor, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getZe16COMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
  return getAModeWinsenMetric(_rxPin, _txPin, _metric, WINSEN_GAS_TYPE_ID_CO, WINSEN_SENSOR_FAILURE_MASK_DEFAULT, WINSEN_CALCULATION_MASK_DEFAULT, _value);
}

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
int8_t getZp14Metric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
  return getAModeWinsenMetric(_rxPin, _txPin, _metric, WINSEN_GAS_TYPE_ID_CH4, WINSEN_SENSOR_FAILURE_MASK_ZP14, WINSEN_SENSOR_FAILURE_MASK_ZP14, _value);
}

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

int8_t getZe08Ch2OMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
  return getAModeWinsenMetric(_rxPin, _txPin, _metric, WINSEN_GAS_TYPE_ID_CH2O, WINSEN_SENSOR_FAILURE_MASK_DEFAULT, WINSEN_CALCULATION_MASK_DEFAULT, _value);
}


/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Winsen sensors, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_WRONG_ANSWER if device returns unexpected gas ID 
*     - DEVICE_ERROR_FAILURE on device failure flag returned
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getAModeWinsenMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, const uint8_t _gasTypeId, const uint8_t _sensorFailureMask, const uint8_t _valueCalculatonMask, int32_t* _value) {
  // ToDo: use _metric for specify full range / concentration ?
  __SUPPRESS_WARNING_UNUSED(_metric);

  uint8_t recievedBytes, crc;
  uint16_t rc = DEVICE_ERROR_TIMEOUT;

  winsenSensorData_t winsenSensorData;
  uint8_t* ptrRawBuffer = (uint8_t*) &winsenSensorData;

  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(WINSEN_UART_SPEED);

  recievedBytes = winsenUartRecieve(swSerial, WINSEN_UART_DEFAULT_READ_TIMEOUT, ptrRawBuffer, sizeof(winsenSensorData));

  // Reading is not finished sucessfully: not all bytes recieved 
  if (recievedBytes < sizeof(winsenSensorData)) { goto finish; }

  // Calculate checksum.
  //Start byte & recieved checksum is not taken in account. The first one is dropped in the read procedure and the second one just will skipped in calculation

  // Try to crc whole buffer?
  crc = winsenCalcCrc(ptrRawBuffer, sizeof(winsenSensorData) - 0x01);

  *_value = 0x00;

  if (crc != winsenSensorData.crc) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }
  // Wrong answer: recieved Gas ID not expected 
  if (_gasTypeId != winsenSensorData.gasTypeId) { rc = DEVICE_ERROR_WRONG_ANSWER; goto finish; }

  if (_sensorFailureMask & winsenSensorData.concentrationHighByte) { rc = DEVICE_ERROR_FAILURE; goto finish; }

  winsenSensorData.concentrationHighByte &= _valueCalculatonMask;

  switch (_gasTypeId) {
     case WINSEN_GAS_TYPE_ID_CH2O:
     case WINSEN_GAS_TYPE_ID_O3:   
       { 
          *_value = (winsenSensorData.concentrationHighByte << 0x08) + winsenSensorData.concentrationLowByte; 
          break; 
       }
     default: { 
          rc = DEVICE_ERROR_NOT_SUPPORTED; 
          goto finish; 
        }
  }

  // noDecimalByte = 0x00 - final multiplier is 1
  // noDecimalByte = 0x01 - final multiplier is 0.1
  rc = (0x00 == winsenSensorData.noDecimalByte) ? RESULT_IS_UNSIGNED_VALUE : RESULT_IS_FLOAT_01_DIGIT;

finish:
  swSerial.~SoftwareSerial();
  return rc;
}
