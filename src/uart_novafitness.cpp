// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "uart_bus.h"
#include "uart_novafitness.h"

static int8_t getNovaSDSMetric(const uint8_t, const uint8_t, const uint8_t, const uint8_t, char*, const uint16_t, uint32_t*);
static uint8_t getNovaSDSChecksum(uint8_t*, uint8_t);

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Nova Fitness SDS (011) sensors, put it to output buffer or variable's address on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED        on success and SENS_READ_ALL metric specified
*     - RESULT_IS_UNSIGNED_VALUE  on success and single metric specified
*     - DEVICE_ERROR_TIMEOUT      if device stop talking
*     - DEVICE_ERROR_CHECKSUM     on checksum error
*
*****************************************************************************************************************************/
int8_t getNovaSDSOneMetric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const uint8_t _particlesSize, uint32_t* _value) {
  char stubBuffer;
  return getNovaSDSMetric(_rxPin, _txPin, _metric, _particlesSize, &stubBuffer, sizeof(stubBuffer) ,_value);
}

int8_t getNovaSDSAllMetrics(const uint8_t _rxPin, const uint8_t _txPin, char* _dst, const uint16_t _dstSize) {
  uint32_t stubValue;
  return getNovaSDSMetric(_rxPin, _txPin, SENS_READ_ALL, NOVA_SDS_PARTICLES_SIZE_NONE, _dst, _dstSize, &stubValue);
}

/*****************************************************************************************************************************
*
*
*****************************************************************************************************************************/
static uint8_t getNovaSDSChecksum(uint8_t* _buffer, uint8_t _len) {
  uint16_t checksum = 0x00;
  
  while (_len) {
    checksum += *_buffer;
    _buffer++;
    _len--;
  }
  return (checksum & 0xFF);
}

/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Nova Fitness SDS (011) sensors, put it to output buffer or variable's address on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED        on success and SENS_READ_ALL metric specified
*     - RESULT_IS_UNSIGNED_VALUE  on success and single metric specified
*     - DEVICE_ERROR_TIMEOUT      if device stop talking
*     - DEVICE_ERROR_CHECKSUM     on checksum error
*
*****************************************************************************************************************************/
static int8_t getNovaSDSMetric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, const uint8_t _particlesSize, char* _dst, const uint16_t _dstSize, uint32_t* _value) {
  int8_t rc = DEVICE_ERROR_TIMEOUT;
  uint8_t checksum, len, buffer[NOVA_SDS_REQUEST_SUZE] = {0x00,};
  uint16_t concentrationPM025, concentrationPM100;
       
  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(NOVA_SDS_UART_SPEED);


  buffer[NOVA_SDS_FIELD_REQUEST_HEAD]        = 0xAA;
  buffer[NOVA_SDS_FIELD_REQUEST_COMMAND_ID]  = 0xB4;
  buffer[NOVA_SDS_FIELD_REQUEST_DATABYTE_01] = 0x04;
  buffer[NOVA_SDS_FIELD_REQUEST_DATABYTE_14] = 0xFF;
  buffer[NOVA_SDS_FIELD_REQUEST_DATABYTE_15] = 0xFF;
  buffer[NOVA_SDS_FIELD_REQUEST_CHECKSUM]    = getNovaSDSChecksum(&buffer[NOVA_SDS_FIELD_REQUEST_DATABYTE_01], NOVA_SDS_FIELD_REQUEST_DATABYTE_15 - NOVA_SDS_FIELD_REQUEST_DATABYTE_01 + 0x01);
  buffer[NOVA_SDS_FIELD_REQUEST_TAIL]        = 0xAB;

  serialSend(&swSerial, buffer, sizeof(buffer), false);

  len = serialRecive(&swSerial, buffer, 0x0A, NOVA_SDS_DEFAULT_READ_TIMEOUT, !UART_STOP_ON_CHAR, '\r', !UART_SLOW_MODE);

  if (len < 0x0A) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value

  if ( 0xAA != buffer[NOVA_SDS_FIELD_RESPONSE_HEAD] || 0xC0 != buffer[NOVA_SDS_FIELD_RESPONSE_COMMAND_ID] || 0xAB != buffer[NOVA_SDS_FIELD_RESPONSE_TAIL]) { goto finish; } // rc inited with DEVICE_ERROR_TIMEOUT value

  checksum = getNovaSDSChecksum(&buffer[NOVA_SDS_FIELD_RESPONSE_DATABYTE_01], NOVA_SDS_FIELD_RESPONSE_DATABYTE_06 - NOVA_SDS_FIELD_RESPONSE_DATABYTE_01 + 0x01);

  if ( checksum != buffer[NOVA_SDS_FIELD_RESPONSE_CHECKSUM]) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }

  // divide to 10 with uint16_t is so ugly :(
  concentrationPM025 = (((uint16_t) buffer[NOVA_SDS_FIELD_RESPONSE_DATABYTE_02] << 8) | buffer[NOVA_SDS_FIELD_RESPONSE_DATABYTE_01]) / 10; 
  concentrationPM100 = (((uint16_t) buffer[NOVA_SDS_FIELD_RESPONSE_DATABYTE_04] << 8) | buffer[NOVA_SDS_FIELD_RESPONSE_DATABYTE_03]) / 10;

  rc = DEVICE_ERROR_NOT_SUPPORTED; 

  switch (_metric) {
     case SENS_READ_ALL: {
       uint16_t writtenBytes = snprintf_P(_dst, _dstSize, PSTR("{\"EPM25\":%u,\"EPM100\":%u}"), concentrationPM025, concentrationPM100);
       _dst[writtenBytes] = CHAR_NULL;
       rc = RESULT_IS_BUFFERED;
       goto finish;
       break;
     } // case SENS_READ_ALL

     case SENS_READ_CONCENTRATION: { 
       switch (_particlesSize) {
         case NOVA_SDS_PARTICLES_SIZE_025:
           *_value = concentrationPM025;
           break;

         case NOVA_SDS_PARTICLES_SIZE_100:
           *_value = concentrationPM100;
           break;

         default:
           goto finish;
       } // switch (_particlesSize)
     } // case SENS_READ_CONCENTRATION
  } // switch (_metric)

  rc = RESULT_IS_UNSIGNED_VALUE;

  finish:
//  gatherSystemMetrics(); // Measure memory consumption
  swSerial.~SoftwareSerial(); 
  return rc;
}

