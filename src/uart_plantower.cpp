// Config & common included files
#include "sys_includes.h"

#include <SoftwareSerial.h>

#include "service.h"
#include "system.h"

#include "uart_bus.h"
#include "uart_plantower.h"


/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Plantower PMS sensors, put it to output buffer or variable's address on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED        on success and SENS_READ_ALL metric specified
*     - RESULT_IS_UNSIGNED_VALUE  on success and single metric specified
*     - DEVICE_ERROR_TIMEOUT      if device stop talking
*     - DEVICE_ERROR_CHECKSUM     on checksum error
*
*****************************************************************************************************************************/
static int8_t getPlantowerPMSMetric(const uint8_t, const uint8_t, uint8_t, const uint8_t, const uint8_t, char*, const uint16_t, uint32_t*);

/*****************************************************************************************************************************
*
*   
*
*****************************************************************************************************************************/

int8_t getPlantowerPMSOneMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const uint8_t _concentrationType, const uint8_t _particlesSize, uint32_t* _value) {
  char stubBuffer;
  return getPlantowerPMSMetric(_rxPin, _txPin, _metric, _concentrationType, _particlesSize, &stubBuffer, sizeof(stubBuffer), _value);
}

int8_t getPlantowerPMSAllMetrics(const uint8_t _rxPin, const uint8_t _txPin, char* _dst, const uint16_t _dstSize) {
  uint32_t stubValue;
  return getPlantowerPMSMetric(_rxPin, _txPin, SENS_READ_ALL, PLANTOWER_CONCENTRATION_TYPE_NONE, PLANTOWER_PARTICLES_SIZE_NONE, _dst, _dstSize, &stubValue);
}

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Plantower PMS sensors, put it to output buffer or variable's address on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED        on success and SENS_READ_ALL metric specified
*     - RESULT_IS_UNSIGNED_VALUE  on success and single metric specified
*     - DEVICE_ERROR_TIMEOUT      if device stop talking
*     - DEVICE_ERROR_CHECKSUM     on checksum error
*
*****************************************************************************************************************************/
static int8_t getPlantowerPMSMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const uint8_t _concentrationType, const uint8_t _particlesSize, char* _dst, const uint16_t _dstSize, uint32_t* _value) {
  int8_t rc = DEVICE_ERROR_TIMEOUT;
  uint8_t headerDetected = false,
          recievedBytes = 0x00;
  uint16_t crc = 0x00;
  uint32_t readStartTime; 

  plantowerData_t plantowerData;

  uint8_t* ptrRawBuffer = (uint8_t*) &plantowerData; // Need for skip 2 byte header
  uint16_t* ptrConvertedBuffer = (uint16_t*) &plantowerData;
     
  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(PLANTOWER_UART_SPEED);

  readStartTime = millis();

  // Wait for 32 byte reading until time is not out
  while ((sizeof(plantowerData_t) > recievedBytes) && (PLANTOWER_DEFAULT_READ_TIMEOUT > millis() - readStartTime)) {

     if (!swSerial.available()) { continue; }

      ptrRawBuffer[recievedBytes] = swSerial.read();

      // Header of packet: 0x42 0x4D
     if (!headerDetected && 0x01 == recievedBytes && 0x4D == ptrRawBuffer[0x01] && 0x42 == ptrRawBuffer[0x00]) {
       headerDetected = true;
       recievedBytes = 0x00;
       continue;
     }
     recievedBytes++;
  } // while

  // reading is not finished sucessfully
  if (recievedBytes < sizeof(plantowerData_t)) { goto finish; }
 
  // Checkcode is sum of all bytes except packet's "check code" value (2 bytes)
  crc = 0x42 + 0x4D;

  for (uint8_t i = 0x00; i < (sizeof(plantowerData_t) - 0x02); i++) { crc += ptrRawBuffer[i]; }

  // Convert bytes to uint16_t
  for (uint8_t i = 0; i < (sizeof(plantowerData_t) / 0x02); i++) {
    //    uint16_t tmpValue = (ptrRawBuffer[0x00] << 8) | ptrRawBuffer[0x01];
    //    *ptrConvertedBuffer = tmpValue;
    *ptrConvertedBuffer = (ptrRawBuffer[0x00] << 0x08) | ptrRawBuffer[0x01];
    ptrRawBuffer += 0x02;
    ptrConvertedBuffer += 0x01;
  }

  if (crc != plantowerData.crc) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }

  *_value = 0x00;
  rc = DEVICE_ERROR_NOT_SUPPORTED; 

  // Construct metric 
  switch (_metric) {
     case SENS_READ_ALL: { 
       uint16_t writtenBytes = snprintf_P(_dst, _dstSize, PSTR("{\"SPM10\":%u,\"SPM25\":%u,\"SPM100\":%u,\"EPM10\":%u,\"EPM25\":%u,\"EPM100\":%u,\"PRT03\":%u,\"PRT05\":%u,\"PRT10\":%u,\"PRT25\":%u,\"PRT50\":%u,\"PRT100\":%u}"),
                plantowerData.standartPM10, plantowerData.standartPM25, plantowerData.standartPM100,
                plantowerData.environmentPM10, plantowerData.environmentPM25, plantowerData.environmentPM100, 
                plantowerData.particles03um, plantowerData.particles05um, plantowerData.particles10um, 
                plantowerData.particles25um, plantowerData.particles50um, plantowerData.particles100um
                );
       _dst[writtenBytes] = CHAR_NULL;
       rc = RESULT_IS_BUFFERED;
       goto finish; 
       break; 
     }

     case SENS_READ_CONCENTRATION: { 
       switch (_concentrationType) {
         case PLANTOWER_CONCENTRATION_TYPE_FACTORY: { 
           switch (_particlesSize) {
             case PLANTOWER_PARTICLES_SIZE_010: { *_value = plantowerData.standartPM10; break; }
             case PLANTOWER_PARTICLES_SIZE_025: { *_value = plantowerData.standartPM25; break; }
             case PLANTOWER_PARTICLES_SIZE_100: { *_value = plantowerData.standartPM100; break; } 
             default: { goto finish; }
           } //switch (_particlesSize)
           break;
         } // case PLANTOWER_CONCENTRATION_TYPE_FACTORY
      
         case PLANTOWER_CONCENTRATION_TYPE_ENVIRONMENT: { 
           switch (_particlesSize) {
             case PLANTOWER_PARTICLES_SIZE_010: { *_value = plantowerData.environmentPM10; break; }
             case PLANTOWER_PARTICLES_SIZE_025: { *_value = plantowerData.environmentPM25; break; }
             case PLANTOWER_PARTICLES_SIZE_100: { *_value = plantowerData.environmentPM100; break; } 
             default: { goto finish; }
       } //switch (_particlesSize)
           break;
         } // case PLANTOWER_CONCENTRATION_TYPE_ENVIRONMENT

         case PLANTOWER_CONCENTRATION_TYPE_NONE:
         default: { goto finish; }
       } // switch (_concentrationType)
       break; 
     } // case SENS_READ_CONCENTRATION

     case SENS_READ_PARTICLES: { 
       switch (_particlesSize) {
         case PLANTOWER_PARTICLES_SIZE_003: { *_value = plantowerData.particles03um; break; }
         case PLANTOWER_PARTICLES_SIZE_005: { *_value = plantowerData.particles05um; break; }
         case PLANTOWER_PARTICLES_SIZE_010: { *_value = plantowerData.particles10um; break; }
         case PLANTOWER_PARTICLES_SIZE_025: { *_value = plantowerData.particles25um; break; }
         case PLANTOWER_PARTICLES_SIZE_050: { *_value = plantowerData.particles50um; break; }
         case PLANTOWER_PARTICLES_SIZE_100: { *_value = plantowerData.particles100um; break; } 
         case PLANTOWER_PARTICLES_SIZE_NONE:
         default: { goto finish; }
       } // switch (_particlesSize)
       break; 
     } // case SENS_READ_PARTICLES
     default:     { goto finish; }
  }

  rc = RESULT_IS_UNSIGNED_VALUE;

finish:
  gatherSystemMetrics(); // Measure memory consumption
  swSerial.~SoftwareSerial(); 
  return rc;
}

