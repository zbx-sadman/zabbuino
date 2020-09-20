// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "i2c_bus.h"
#include "uart_bus.h"
#include "uart_plantower.h"

static uint8_t processingRawData(uint8_t*, uint8_t);
static uint8_t getRawDataViaUart(const uint8_t, const uint8_t, uint8_t*, uint8_t);
static int8_t getRawDataViaI2C(SoftwareTWI*, uint8_t, uint8_t*, const uint8_t);


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
static int8_t getPlantowerWuhanPMMetricViaUart(const uint8_t, const uint8_t, uint8_t, const uint8_t, const uint8_t, char*, const uint16_t, uint32_t*);
static int8_t getPlantowerWuhanPMMetricViaI2C(SoftwareTWI*, uint8_t, uint8_t, const uint8_t, const uint8_t, char*, const uint16_t, uint32_t*);
static int8_t formPlantowerWuhanPMMetric(plantowerWuhanData_t&, uint8_t, const uint8_t, const uint8_t, char*, const uint16_t, uint32_t*);
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


int8_t getPlantowerWuhanPMOneMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const uint8_t _concentrationType, const uint8_t _particlesSize, uint32_t* _value) {
  char stubBuffer;
  return getPlantowerWuhanPMMetricViaUart(_rxPin, _txPin, _metric, _concentrationType, _particlesSize, &stubBuffer, sizeof(stubBuffer), _value);
}

int8_t getPlantowerWuhanPMAllMetrics(const uint8_t _rxPin, const uint8_t _txPin, char* _dst, const uint16_t _dstSize) {
  uint32_t stubValue;
  return getPlantowerWuhanPMMetricViaUart(_rxPin, _txPin, SENS_READ_ALL, PLANTOWER_CONCENTRATION_TYPE_NONE, PLANTOWER_PARTICLES_SIZE_NONE, _dst, _dstSize, &stubValue);
}

int8_t getPlantowerWuhanPMOneMetric(SoftwareTWI* _softTWI, uint8_t _i2cAddress, uint8_t _metric, const uint8_t _concentrationType, const uint8_t _particlesSize, uint32_t* _value) {
  char stubBuffer;
  return getPlantowerWuhanPMMetricViaI2C(_softTWI, _i2cAddress, _metric, _concentrationType, _particlesSize, &stubBuffer, sizeof(stubBuffer), _value);
}

int8_t getPlantowerWuhanPMAllMetrics(SoftwareTWI* _softTWI, uint8_t _i2cAddress, char* _dst, const uint16_t _dstSize) {
  uint32_t stubValue;
  return getPlantowerWuhanPMMetricViaI2C(_softTWI, _i2cAddress, SENS_READ_ALL, PLANTOWER_CONCENTRATION_TYPE_NONE, PLANTOWER_PARTICLES_SIZE_NONE, _dst, _dstSize, &stubValue);
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
 
  plantowerData_t plantowerData;
  uint8_t* ptrRawData = (uint8_t*) &plantowerData;


  // reading is not finished sucessfully
  rc = getRawDataViaUart(_rxPin, _txPin, ptrRawData, sizeof(plantowerData));
  if (RESULT_IS_OK != rc) {goto finish; }

  rc = processingRawData(ptrRawData, sizeof(plantowerData));
  if (RESULT_IS_OK != rc) {goto finish; }

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
//  gatherSystemMetrics(); // Measure memory consumption
  return rc;
}


// One of releases Wuhan Cubic PM2012 have protocol structure like Plantower PMSxxx with some changes: no Factory(Standart) PM falues & Status/Version fields added
static int8_t getPlantowerWuhanPMMetricViaUart(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const uint8_t _concentrationType, const uint8_t _particlesSize, char* _dst, const uint16_t _dstSize, uint32_t* _value) {
  int8_t rc = DEVICE_ERROR_TIMEOUT;
 
  plantowerWuhanData_t plantowerWuhanData;
  uint8_t* ptrRawData = (uint8_t*) &plantowerWuhanData;

  //dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]


  // reading is not finished sucessfully
  rc = getRawDataViaUart(_rxPin, _txPin, ptrRawData, sizeof(plantowerWuhanData));
  if (RESULT_IS_OK != rc) {goto finish; }

  rc = processingRawData(ptrRawData, sizeof(plantowerWuhanData));
  if (RESULT_IS_OK != rc) {goto finish; }

  rc = formPlantowerWuhanPMMetric(plantowerWuhanData, _metric, _concentrationType, _particlesSize, _dst, _dstSize, _value);

finish:
//  gatherSystemMetrics(); // Measure memory consumption
  return rc;
}

static int8_t getPlantowerWuhanPMMetricViaI2C(SoftwareTWI* _softTWI, uint8_t _i2cAddress, uint8_t _metric, const uint8_t _concentrationType, const uint8_t _particlesSize, char* _dst, const uint16_t _dstSize, uint32_t* _value) {
  int8_t rc = DEVICE_ERROR_TIMEOUT;
 
  plantowerWuhanData_t plantowerWuhanData;
  uint8_t* ptrRawData = (uint8_t*) &plantowerWuhanData;

  // reading is not finished sucessfully
  rc = getRawDataViaI2C(_softTWI,_i2cAddress, ptrRawData, sizeof(plantowerWuhanData));
  if (RESULT_IS_OK != rc) {goto finish; }

  rc = processingRawData(ptrRawData, sizeof(plantowerWuhanData));
  if (RESULT_IS_OK != rc) {goto finish; }

  rc = formPlantowerWuhanPMMetric(plantowerWuhanData, _metric, _concentrationType, _particlesSize, _dst, _dstSize, _value);

finish:
//  gatherSystemMetrics(); // Measure memory consumption
  return rc;
}

static int8_t formPlantowerWuhanPMMetric(plantowerWuhanData_t& _plantowerWuhanData, uint8_t _metric, const uint8_t _concentrationType, const uint8_t _particlesSize, char* _dst, const uint16_t _dstSize, uint32_t* _value) {
  int8_t rc = DEVICE_ERROR_NOT_SUPPORTED;
 
  *_value = 0x00;

  // Construct metric 
  switch (_metric) {
     case SENS_READ_ALL: { 
       uint16_t writtenBytes = snprintf_P(_dst, _dstSize, PSTR("{\"SPM10\":%u,\"SPM25\":%u,\"SPM100\":%u,\"EPM10\":%u,\"EPM25\":%u,\"EPM100\":%u,\"PRT03\":%u,\"PRT05\":%u,\"PRT10\":%u,\"PRT25\":%u,\"PRT50\":%u,\"PRT100\":%u,\"STATUS\":%u}"),
                _plantowerWuhanData.standartPM10, _plantowerWuhanData.standartPM25, _plantowerWuhanData.standartPM100,
                _plantowerWuhanData.environmentPM10, _plantowerWuhanData.environmentPM25, _plantowerWuhanData.environmentPM100, 
                _plantowerWuhanData.particles03um, _plantowerWuhanData.particles05um, _plantowerWuhanData.particles10um, 
                _plantowerWuhanData.particles25um, _plantowerWuhanData.particles50um, _plantowerWuhanData.particles100um,
                *((uint8_t*)&_plantowerWuhanData.status)
                );
       _dst[writtenBytes] = CHAR_NULL;
       rc = RESULT_IS_BUFFERED;
       goto finish; 
       break; 
     }

     case SENS_READ_STATUS: { 
       *_value = *((uint8_t*)&_plantowerWuhanData.status);
       break; 
     } // case SENS_READ_STATUS

     case SENS_READ_CONCENTRATION: { 
       switch (_concentrationType) {
         case PLANTOWER_CONCENTRATION_TYPE_FACTORY: { 
           switch (_particlesSize) {
             case PLANTOWER_PARTICLES_SIZE_010: { *_value = _plantowerWuhanData.standartPM10; break; }
             case PLANTOWER_PARTICLES_SIZE_025: { *_value = _plantowerWuhanData.standartPM25; break; }
             case PLANTOWER_PARTICLES_SIZE_100: { *_value = _plantowerWuhanData.standartPM100; break; } 
             default: { goto finish; }
           } //switch (_particlesSize)
           break;
         } // case PLANTOWER_CONCENTRATION_TYPE_FACTORY
      
         case PLANTOWER_CONCENTRATION_TYPE_ENVIRONMENT: { 
           switch (_particlesSize) {
             case PLANTOWER_PARTICLES_SIZE_010: { *_value = _plantowerWuhanData.environmentPM10; break; }
             case PLANTOWER_PARTICLES_SIZE_025: { *_value = _plantowerWuhanData.environmentPM25; break; }
             case PLANTOWER_PARTICLES_SIZE_100: { *_value = _plantowerWuhanData.environmentPM100; break; } 
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
         case PLANTOWER_PARTICLES_SIZE_003: { *_value = _plantowerWuhanData.particles03um; break; }
         case PLANTOWER_PARTICLES_SIZE_005: { *_value = _plantowerWuhanData.particles05um; break; }
         case PLANTOWER_PARTICLES_SIZE_010: { *_value = _plantowerWuhanData.particles10um; break; }
         case PLANTOWER_PARTICLES_SIZE_025: { *_value = _plantowerWuhanData.particles25um; break; }
         case PLANTOWER_PARTICLES_SIZE_050: { *_value = _plantowerWuhanData.particles50um; break; }
         case PLANTOWER_PARTICLES_SIZE_100: { *_value = _plantowerWuhanData.particles100um; break; } 
         case PLANTOWER_PARTICLES_SIZE_NONE:
         default: { goto finish; }
       } // switch (_particlesSize)
       break; 
     } // case SENS_READ_PARTICLES
     default:     { goto finish; }
  }

  rc = RESULT_IS_UNSIGNED_VALUE;

finish:
  return rc;
}


/***************************************************************************************************/
static uint8_t getRawDataViaUart(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _buffer, uint8_t _bufferSize) {
  uint8_t writePos = 0x00;

  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(PLANTOWER_UART_SPEED);

  uint32_t readStartTime = millis();

  // Wait for 32 byte reading until time is not out
  while ((_bufferSize > writePos) && (PLANTOWER_DEFAULT_READ_TIMEOUT > millis() - readStartTime)) {

     if (!swSerial.available()) { continue; }
     _buffer[writePos] = swSerial.read();
     // 1st byte of header must be 0x42. 2nd byte of header must be 0x4D. Buffer filling should be restarted every time until header not detected
     writePos = ((0x00 == writePos && 0x42 != _buffer[writePos]) || (0x01 == writePos && 0x4D != _buffer[writePos])) ? 0x00 : (writePos + 0x01);
  }

  // reading is not finished sucessfully
  return (_bufferSize == writePos) ? RESULT_IS_OK : DEVICE_ERROR_TIMEOUT;
   
}

static int8_t getRawDataViaI2C(SoftwareTWI* _softTWI, uint8_t _i2cAddress, uint8_t* _buffer, const uint8_t _bufferSize) {
  int8_t rc = DEVICE_ERROR_CONNECT;

  if (!_i2cAddress) { _i2cAddress = PM2012_I2C_ADDRESS; }

  if (!isI2CDeviceReady(_softTWI, _i2cAddress)) { goto finish; }
 
  if (_bufferSize != readBytesFromI2C(_softTWI, _i2cAddress, I2C_NO_REG_SPECIFIED, _buffer, _bufferSize)) { rc = DEVICE_ERROR_TIMEOUT; goto finish; }

  rc = RESULT_IS_OK;

finish:
  return rc;
}

static uint8_t processingRawData(uint8_t* _buffer, uint8_t _bufferSize) {
  uint16_t checkCode = 0x00;

  uint8_t*  ptrRawBuffer = _buffer; 
  uint16_t* ptrConvertedBuffer = (uint16_t*) _buffer;

  // Convert uint8_t array to uint16_t array thru swapping bytes
  for (uint8_t i = 0x00; i < (_bufferSize / 0x02); i++) {
    *ptrConvertedBuffer = (ptrRawBuffer[0x00] << 8) | ptrRawBuffer[0x01];
    ptrConvertedBuffer++;
    ptrRawBuffer += 2;
  }
  // Make One step backward to point CRC in the buffer
  ptrConvertedBuffer--;

  // Calc CRC
  for (uint8_t i = 0x00; i < (_bufferSize - sizeof(checkCode)); i++) { checkCode += _buffer[i]; }

  return (*ptrConvertedBuffer == checkCode) ? RESULT_IS_OK : DEVICE_ERROR_CHECKSUM;

}

