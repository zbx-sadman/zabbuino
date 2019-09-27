/*

Based on: https://github.com/RobTillaart/Arduino/tree/master/libraries/DHTstable
version 0.1.13 is used

*/

// Config & common included files
#include "sys_includes.h"

#include "service.h"
#include "system.h"

#include "dht.h"

/*****************************************************************************************************************************
*
*  Read specified metric's value of the AM/DHT sensor, put it to specified variable's address on success.
*
*  Returns: 
*     - RESULT_IS_FLOAT_01_DIGIT    on success
*     - DEVICE_ERROR_NOT_SUPPORTED  on wrong params specified
*     - DEVICE_ERROR_TIMEOUT        on sensor stops answer to the request
*     - DEVICE_ERROR_CHECKSUM       on detect data corruption
*     - DEVICE_ERROR_ACK_L
*     - DEVICE_ERROR_ACK_H
*
*****************************************************************************************************************************/
int8_t getDHTMetric(const uint8_t _pin, const uint8_t _sensorModel, const uint8_t _metric, int32_t* _value) {
  int8_t   rc = DEVICE_ERROR_TIMEOUT;
  // INIT BUFFERVAR TO RECEIVE DATA
  uint8_t  mask = 128,
           bit, port, i,
           idx = 0x00, 
           crc = 0x00,
           wakeupDelay,
           bits[5] = {0,0,0,0,0};      // buffer to receive data
  uint16_t loopCount = 0x00;
  uint32_t humidity, 
           waitTime,
           // To avoid reading eror on sensor:
           //   - two readtake minimum time interval be 1 sec;
           //   - _pin must be HIGH before operating at least 1 s*. 
           //
           //   * DHT11, DHT22 datasheets: "When power is supplied to sensor, don't send any instruction to the sensor within one second to pass unstable status"
           //   Note: on AM2301 power 2sec crossed the unstable state 
           readingInterval = 1000UL, 
           pinUpDelay = 1000UL;       
  int32_t  temperature;

  static uint32_t lastReadTime = 0x00;
  /*
  Need to use lastpin/lastresult static array to return data immedately

  static int8_t lastPin = -1;
  if ((_pin != lastPin) || (0 == waitTime)) {}
  */


  bit = digitalPinToBitMask(_pin);
  port = digitalPinToPort(_pin);
  volatile uint8_t *PIR = portInputRegister(port);

   switch (_sensorModel) {
    case DHT11_ID:
      wakeupDelay = DHTLIB_DHT11_WAKEUP; // T-be - Host the start signal down time
      break;
    case DHT21_ID:
      // To avoid reading eror on AM2301:
      //   - two readtake minimum time interval for be 2sec
      //   - _pin must be HIGH before operating ~2 sec (experimental value).
      readingInterval = pinUpDelay = 2000UL; 
      //break;
    case DHT22_ID:
    case DHT33_ID:
    case DHT44_ID:
      wakeupDelay = DHTLIB_DHT_WAKEUP;   // T-be
      break;

    default:
      rc = DEVICE_ERROR_NOT_SUPPORTED;
      goto finish;
  }

  // DHT sensor have limit for taking samples frequency
  waitTime = millis() - lastReadTime;
  waitTime = (waitTime < readingInterval) ? (readingInterval - waitTime) : 0;

  // Sensor won't to go ready state if no HIGH level exist on pin for a few time (on first reading). 
  // This delay can be skipped by setting pin to OUTPUT+HIGH with port_mode[] & port_pullup[] arrays (cfg_tune.h)
  //  if (digitalRead(_pin) == LOW ) {
  if (LOW == (*PIR & bit)) {
     if (waitTime < pinUpDelay) { waitTime = pinUpDelay; }
  }

  stopTimerOne(); 
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);

  // Wait for sensor waked up
  delay(waitTime); 

  // REQUEST SAMPLE
  digitalWrite(_pin, LOW);
  delay(wakeupDelay);
  digitalWrite(_pin, HIGH);
  delayMicroseconds(40);
  pinMode(_pin, INPUT_PULLUP);

  // data exchange with DHT sensor must not be interrupted

  // GET ACKNOWLEDGE or TIMEOUT
  loopCount = DHTLIB_TIMEOUT;
//  while (digitalRead(_pin) == LOW) {
  while (LOW == (*PIR & bit)) { if (--loopCount == 0) { rc = DEVICE_ERROR_ACK_L; goto finish; } }

  loopCount = DHTLIB_TIMEOUT;
//  while (digitalRead(_pin) == HIGH) {
  while (LOW != (*PIR & bit) ) { if (--loopCount == 0) { rc = DEVICE_ERROR_ACK_H; goto finish; } }

  // READ THE OUTPUT - 40 BITS => 5 BYTES
  for (i = 40; i != 0; i--) {
      loopCount = DHTLIB_TIMEOUT;
      //  while (digitalRead(_pin) == LOW) {
      while(LOW == (*PIR & bit)) { if (--loopCount == 0) { rc = DEVICE_ERROR_TIMEOUT; goto finish; } }

      uint32_t t = micros();
      loopCount = DHTLIB_TIMEOUT;
      //  while (digitalRead(_pin) == HIGH) {
      while(LOW != (*PIR & bit)) { if (--loopCount == 0) { rc = DEVICE_ERROR_TIMEOUT; goto finish; } }

      if ((micros() - t) > 40) { bits[idx] |= mask; }
      mask >>= 1;
      // next byte?
      if (mask == 0x00) { mask = 128; idx++; }
  }

  // Store time to use it to use on the next call of sub
  lastReadTime = millis();
     
  switch (_sensorModel) {
    case DHT11_ID:
      // original code part:
      //        these bits are always zero, masking them reduces errors.
      //        bits[0] &= 0x7F;
      //        bits[2] &= 0x7F;
      crc = bits[0] + bits[2];
      humidity    = bits[0] * 10;  // bits[1] == 0;
      temperature = bits[2] * 10;  // bits[3] == 0;
      break;
    case DHT21_ID:
    case DHT22_ID:
    case DHT33_ID:
    case DHT44_ID:
    default:
      // original code part:
      //        these bits are always zero, masking them reduces errors.
      //        bits[0] &= 0x03;
      //        bits[2] &= 0x83;
      //        humidity = (bits[0]*256 + bits[1]) * 0.1;  <== * 0.1 processed by ltoaf()
      crc = bits[0] + bits[1] + bits[2] + bits[3];
      humidity = (bits[0] * 256 + bits[1]);
      temperature = ((bits[2] & 0x7F) * 256 + bits[3]);// * 0.1;
      // negative temperature
      if (bits[2] & 0x80) { temperature = -temperature; }
  }
  // TEST CHECKSUM
  if (bits[4] != crc) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }

  *_value = (SENS_READ_HUMD == _metric) ? humidity : temperature;

/*
  switch (_metric) {
     case SENS_READ_HUMD:
       *_value = humidity;
       break;

     case SENS_READ_TEMP:
       *_value = temperature;
       break;
     case SENS_READ_ALL:
       snprintf_P(_dst, 250, PSTR("{\"t\":%u,\"SPM25\":%u,\"SPM100\":%u,\"EPM10\":%u,\"EPM25\":%u,\"EPM100\":%u,\"PRT03\":%u,\"PRT05\":%u,\"PRT10\":%u,\"PRT25\":%u,\"PRT50\":%u,\"PRT100\":%u}"),
                ptrDataStructured->standartPM10, ptrDataStructured->standartPM25, ptrDataStructured->standartPM100,
                ptrDataStructured->environmentPM10, ptrDataStructured->environmentPM25, ptrDataStructured->environmentPM100, 
                ptrDataStructured->particles03um, ptrDataStructured->particles05um, ptrDataStructured->particles10um, 
                ptrDataStructured->particles25um, ptrDataStructured->particles50um, ptrDataStructured->particles100um
                );
       break;
   }
*/

  rc = RESULT_IS_FLOAT_01_DIGIT;

finish:
  startTimerOne(); 
  gatherSystemMetrics(); 
  return rc;
}


/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
/*
int8_t getDHTMetric(const uint8_t _pin, const uint8_t _sensorModel, const uint8_t _metric, int32_t* _value)
{
  //char stubBuffer;
  return getDHTMetric(_pin, _sensorModel, _metric, &stubBuffer, _value);

}
int8_t getDHTMetric(const uint8_t _pin, const uint8_t _sensorModel, const uint8_t _metric, char* _dst)
{
  int32_t stubValue;
  return getDHTMetric(_pin, _sensorModel, _metric, _dst, &stubValue);
}
*/