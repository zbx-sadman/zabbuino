// Config & common included files
#include "sys_includes.h"

#include <SoftwareSerial.h>

#include "service.h"
#include "system.h"

#include "uart_bus.h"
#include "uart_plantower.h"


/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getPlantowerPM25Metric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, uint32_t* _value)
{
  char stubBuffer;
  return getPlantowerPM25Metric(_rxPin, _txPin, _metric, &stubBuffer, _value, OUTPUT_NUMBER);
}

int8_t getPlantowerPM25Metric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, char* _dst)
{
  uint32_t stubValue;
  return getPlantowerPM25Metric(_rxPin, _txPin, _metric, _dst, &stubValue, OUTPUT_STRING);
}

int8_t getPlantowerPM25AllMetrics(const uint8_t _rxPin, const uint8_t _txPin, char* _dst)
{
  uint32_t stubValue;
  return getPlantowerPM25Metric(_rxPin, _txPin, SENS_READ_ALL, _dst, &stubValue, OUTPUT_JSON);
}


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
int8_t getPlantowerPM25Metric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, char* _dst, uint32_t* _value, const uint8_t _outputType) {
  int8_t rc = DEVICE_ERROR_TIMEOUT;
  uint8_t buffer[sizeof(plantowerData_t)],
          headerDetected = false,
          writePos=0x00;
  uint16_t checkCode = 0x00;
  uint32_t readStartTime; 

  uint8_t* ptrRawBuffer = (uint8_t*) buffer; // Need for skip 2 byte header
  uint16_t* ptrConvertedBuffer = (uint16_t*) buffer;
  plantowerData_t* ptrDataStructured = (plantowerData_t*) buffer;
       
  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(PLANTOWER_UART_SPEED);

  readStartTime = millis();

  // Wait for 32 byte reading until time is not out
  while ((sizeof(plantowerData_t) > writePos) && (PLANTOWER_DEFAULT_READ_TIMEOUT > millis() - readStartTime)) {

     if (!swSerial.available()) {
        continue;
      }

      buffer[writePos]=swSerial.read();

    // Header of packet: 0x42 0x4D
    // On header waiting test first two byte every time when its in buffer (writePos == 0x01)
    // if signature is detected - write real data from the start of buffer
    // if not - just move second byte to first position and continue writing to the next cell of buffer
    if (!headerDetected) {
       if (0x01 == writePos) {
          headerDetected = (0x4D == buffer[0x01] && 0x42 == buffer[0x00]);
          if (headerDetected) {
             continue;
           } else {
              buffer[0x00] = buffer[0x01];
           }
       }
    }
     writePos++;
  }

  // reading is not finished sucessfully
  if (writePos < sizeof(plantowerData_t)) {
    goto finish;
  }
 
  // Checkcode is sum of all bytes except packet's "check code" value (2 bytes)
  checkCode = 0x42 + 0x4D;
  for (uint8_t i = 0; i < (sizeof(plantowerData_t) - 2); i++) {
    checkCode += buffer[i];
  }

  // Convert bytes to uint16_t
  for (uint8_t i = 0; i < (sizeof(plantowerData_t) / 2); i++) {
    //    uint16_t tmpValue = (ptrRawBuffer[0x00] << 8) | ptrRawBuffer[0x01];
    //    *ptrConvertedBuffer = tmpValue;
    *ptrConvertedBuffer = (ptrRawBuffer[0x00] << 8) | ptrRawBuffer[0x01];
    ptrRawBuffer += 2;
    ptrConvertedBuffer += 1;
  }

  if (checkCode != ptrDataStructured->checkCode) {
     rc = DEVICE_ERROR_CHECKSUM;
     goto finish;
  }

  *_value = 0x00;

  switch (_metric) {
     case SENS_READ_STANDART_PM10:
       *_value = ptrDataStructured->standartPM10;
       break;
     case SENS_READ_STANDART_PM25:
       *_value = ptrDataStructured->standartPM25;
       break;
     case SENS_READ_STANDART_PM100:   
       *_value = ptrDataStructured->standartPM100;
       break;
     case SENS_READ_ENVIRONMENT_PM10: 
       *_value = ptrDataStructured->environmentPM10;
       break;
     case SENS_READ_ENVIRONMENT_PM25: 
       *_value = ptrDataStructured->environmentPM25;
       break;
     case SENS_READ_ENVIRONMENT_PM100:
       *_value = ptrDataStructured->environmentPM100;
       break;
     case SENS_READ_PARTICLES_03_UM:  
       *_value = ptrDataStructured->particles03um;
       break;
     case SENS_READ_PARTICLES_05_UM:  
       *_value = ptrDataStructured->particles05um;
       break;
     case SENS_READ_PARTICLES_10_UM:  
       *_value = ptrDataStructured->particles10um;
       break;
     case SENS_READ_PARTICLES_25_UM:  
       *_value = ptrDataStructured->particles25um;
       break;
     case SENS_READ_PARTICLES_50_UM:  
       *_value = ptrDataStructured->particles50um;
       break;
     case SENS_READ_PARTICLES_100_UM:
       *_value = ptrDataStructured->particles100um;
       break;
     case SENS_READ_ALL:
       break;
     default:
       rc = RESULT_IS_FAIL;
       goto finish;
       break;
  }

  switch (_outputType) {
     case OUTPUT_STRING:
       ltoa(*_value, _dst, 10);
       break;
     case OUTPUT_NUMBER:
       break;
     case OUTPUT_JSON:
       snprintf_P(_dst, 250, PSTR("{\"SPM10\":%u,\"SPM25\":%u,\"SPM100\":%u,\"EPM10\":%u,\"EPM25\":%u,\"EPM100\":%u,\"PRT03\":%u,\"PRT05\":%u,\"PRT10\":%u,\"PRT25\":%u,\"PRT50\":%u,\"PRT100\":%u}"),
                ptrDataStructured->standartPM10, ptrDataStructured->standartPM25, ptrDataStructured->standartPM100,
                ptrDataStructured->environmentPM10, ptrDataStructured->environmentPM25, ptrDataStructured->environmentPM100, 
                ptrDataStructured->particles03um, ptrDataStructured->particles05um, ptrDataStructured->particles10um, 
                ptrDataStructured->particles25um, ptrDataStructured->particles50um, ptrDataStructured->particles100um
                );
       break;
  } 

  rc = RESULT_IS_BUFFERED;

  finish:
  gatherSystemMetrics(); // Measure memory consumption
  swSerial.~SoftwareSerial(); 
  return rc;
}

