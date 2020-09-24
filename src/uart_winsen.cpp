// Config & common included files
#include "sys_includes.h"

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
    yield();
    if (!_stream.available()) {
      continue;
    }
    _dst[recievedBytes] = _stream.read();
    //Serial.print("winsen: 0x"); Serial.println(_dst[recievedBytes], HEX);

    if (!headerDetected) {
      headerDetected = (WINSEN_UART_START_BYTE == _dst[0x00]);
    }
    if (headerDetected) {
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
int8_t getAModeZe14O3Metric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
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
int8_t getAModeZe15COMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
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
int8_t getAModeZe16COMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
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
int8_t getAModeZp14Metric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
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

int8_t getAModeZe08Ch2OMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
  return getAModeWinsenMetric(_rxPin, _txPin, _metric, WINSEN_GAS_TYPE_ID_CH2O, WINSEN_SENSOR_FAILURE_MASK_DEFAULT, WINSEN_CALCULATION_MASK_DEFAULT, _value);
}

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
int8_t getQModeMhZxMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, int32_t* _value) {
  return getQModeWinsenMetric(_rxPin, _txPin, _metric, WINSEN_SENSOR_TYPE_ID_MH_ZXX , _value);
  //return getQModeWinsenMetric(_rxPin, _txPin, _metric, WINSEN_SENSOR_TYPE_ID_ZE08_CH2O, _value);
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

  winsenSensorAModeData_t winsenSensorData;
  uint8_t* ptrRawBuffer = (uint8_t*) &winsenSensorData;

  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(WINSEN_UART_SPEED);

  recievedBytes = winsenUartRecieve(swSerial, WINSEN_UART_DEFAULT_READ_TIMEOUT, ptrRawBuffer, sizeof(winsenSensorData));

  // Reading is not finished sucessfully: not all bytes recieved 
  if (recievedBytes < sizeof(winsenSensorData)) { goto finish; }

  *_value = 0x00;

  // Calculate checksum.
  // Start byte is not taken in account
  crc = winsenCalcCrc(&ptrRawBuffer[0x01], sizeof(winsenSensorData) - 0x01);

  if (crc) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }
  // Wrong answer: recieved Gas ID not expected 
  if (_gasTypeId != winsenSensorData.gasTypeId) { rc = DEVICE_ERROR_WRONG_ANSWER; goto finish; }

  if (_sensorFailureMask & winsenSensorData.concentrationHighByte) { rc = DEVICE_ERROR_FAILURE; goto finish; }

  winsenSensorData.concentrationHighByte &= _valueCalculatonMask;

  switch (_gasTypeId) {
//     case WINSEN_GAS_TYPE_ID_CO2:
     case WINSEN_GAS_TYPE_ID_CH2O:
     case WINSEN_GAS_TYPE_ID_O3:   
       { 
          *_value = ((uint16_t) winsenSensorData.concentrationHighByte << 0x08) + winsenSensorData.concentrationLowByte; 
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
// !!! Q&A mode's answer frame is not equal for various type of sensors
int8_t getQModeWinsenMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, const uint8_t _sensorId, int32_t* _value) {
  __SUPPRESS_WARNING_UNUSED(_metric);

  uint8_t recievedBytes, crc, 
          sensorStatusByte              = 0x00,
          sensorFailureMask             = WINSEN_SENSOR_FAILURE_MASK_DEFAULT,
          valueCalculatonMask           = WINSEN_CALCULATION_MASK_DEFAULT,
          concentrationPartsPerHighByte = 0x00, 
          concentrationPartsPerLowByte  = 0x00;

  uint16_t rc = DEVICE_ERROR_TIMEOUT;

  uint8_t rawBuffer[WINSEN_UART_PACKET_SIZE] = {WINSEN_UART_START_BYTE, 0x01, WINSEN_UART_GET_COMMAND, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};;

  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(WINSEN_UART_SPEED);

  *_value = 0x00;

  serialSend(&swSerial, rawBuffer, sizeof(rawBuffer), !UART_SLOW_MODE);
  recievedBytes = winsenUartRecieve(swSerial, WINSEN_UART_DEFAULT_READ_TIMEOUT, rawBuffer, sizeof(rawBuffer));

  // Reading is not finished sucessfully: not all bytes recieved 
  if (recievedBytes < sizeof(rawBuffer)) { goto finish; }

  // Calculate checksum.
  // Start byte is not taken in account
  crc = winsenCalcCrc(&rawBuffer[0x01], sizeof(rawBuffer) - 0x01);

  if (crc) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }

  // Wrong answer: recieved Command ID not expected 
  if (WINSEN_UART_GET_COMMAND != rawBuffer[WINSEN_QMODE_PACKET_BYTE_COMMAND_ID]) { rc = DEVICE_ERROR_WRONG_ANSWER; goto finish; }

  concentrationPartsPerHighByte = rawBuffer[WINSEN_QMODE_PACKET_BYTE_PARTSPER_CONCENTRATION_HIGH];
  concentrationPartsPerLowByte  = rawBuffer[WINSEN_QMODE_PACKET_BYTE_PARTSPER_CONCENTRATION_LOW];

  switch (_sensorId) {
   case WINSEN_SENSOR_TYPE_ID_ZE08_CH2O: {
     // ZE08-CH2O place 'Parts-Per' concentration to Mass Concentration bytes
     concentrationPartsPerHighByte = rawBuffer[WINSEN_QMODE_PACKET_BYTE_MASS_CONCENTRATION_HIGH];
     concentrationPartsPerLowByte  = rawBuffer[WINSEN_QMODE_PACKET_BYTE_MASS_CONCENTRATION_LOW];
     // failure mask & calculaton mask is default
     // ...
     break;
   }

   case WINSEN_SENSOR_TYPE_ID_ZE15_CO: {
     sensorFailureMask   = WINSEN_SENSOR_FAILURE_MASK_ZE15; 
     valueCalculatonMask = WINSEN_CALCULATION_MASK_ZE15;
     break;
   }

   case WINSEN_SENSOR_TYPE_ID_ZP14: {
     sensorFailureMask   = WINSEN_SENSOR_FAILURE_MASK_ZP14; 
     valueCalculatonMask = WINSEN_CALCULATION_MASK_ZP14;
     break;
   }


   default: {
     // failure mask & calculaton mask is default
     // ...
     break;
   }
  }

  if (sensorFailureMask & sensorStatusByte) { rc = DEVICE_ERROR_FAILURE; goto finish; }

  concentrationPartsPerHighByte &= valueCalculatonMask;
  *_value = ((uint16_t) concentrationPartsPerHighByte << 0x08) + concentrationPartsPerLowByte; 

  rc = RESULT_IS_UNSIGNED_VALUE;

finish:
  swSerial.~SoftwareSerial();
  return rc;
}
