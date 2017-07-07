/*

Based on: https://github.com/RobTillaart/Arduino/tree/master/libraries/DHTstable
version 0.1.13 is used

*/

#include "dht.h"

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getDHTMetric(const uint8_t _pin, const uint8_t _sensorModel, const uint8_t _metric, int32_t* _value)
{
  char stubBuffer;
  return getDHTMetric(_pin, _sensorModel, _metric, &stubBuffer, _value, true);

}

int8_t getDHTMetric(const uint8_t _pin, const uint8_t _sensorModel, const uint8_t _metric, char* _dst)
{
  int32_t stubValue;
  return getDHTMetric(_pin, _sensorModel, _metric, _dst, &stubValue, false);
}

/*****************************************************************************************************************************
*
*  Read specified metric's value of the AM/DHT sensor, put it to output buffer on success. 
*
*  Returns: 
*    - RESULT_IN_BUFFER on success
*    - DEVICE_ERROR_CONNECT on connection error
*    - DEVICE_ERROR_ACK_L
*    - DEVICE_ERROR_ACK_H
*    - DEVICE_ERROR_TIMEOUT if sensor stops answer to the request
*
*****************************************************************************************************************************/
int8_t getDHTMetric(const uint8_t _pin, const uint8_t _sensorModel, const uint8_t _metric, char *_dst, int32_t* _value, const uint8_t _wantsNumber)
{
  int8_t rc = DEVICE_ERROR_TIMEOUT;
  // INIT BUFFERVAR TO RECEIVE DATA
  uint8_t mask = 128,
          bit, port, i,
          idx = 0, 
          crc = 0,
          wakeupDelay,
          bits[5] = {0,0,0,0,0};      // buffer to receive data
  uint16_t loopCount = 0;
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

  static uint32_t lastReadTime = 0;

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
    default:
      wakeupDelay = DHTLIB_DHT_WAKEUP;   // T-be
  }


  // DHT sensor have limit for taking samples frequency
  waitTime = millis() - lastReadTime;
  waitTime = (waitTime < readingInterval) ? (readingInterval - waitTime) : 0;

  // Sensor won't to go ready state if no HIGH level exist on pin for a few time (on first reading). 
  // This delay can be skipped by setting pin to OUTPUT+HIGH with port_mode[] & port_pullup[] arrays (tune.h)
  //  if (digitalRead(_pin) == LOW ) {
  if ((*PIR & bit) == LOW ) {
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
  while ((*PIR & bit) == LOW) {
    if (--loopCount == 0) { rc = DEVICE_ERROR_ACK_L; goto finish; }
  }

  loopCount = DHTLIB_TIMEOUT;
//  while (digitalRead(_pin) == HIGH) {
  while ((*PIR & bit) != LOW) {
    if (--loopCount == 0) { rc = DEVICE_ERROR_ACK_H; goto finish; }
  }

    // READ THE OUTPUT - 40 BITS => 5 BYTES
    for (i = 40; i != 0; i--)
    {
        loopCount = DHTLIB_TIMEOUT;
        //  while (digitalRead(_pin) == LOW) {
        while((*PIR & bit) == LOW) {
          if (--loopCount == 0) { rc = DEVICE_ERROR_TIMEOUT; goto finish; }  
        }

        uint32_t t = micros();

        loopCount = DHTLIB_TIMEOUT;
        //  while (digitalRead(_pin) == HIGH) {
        while((*PIR & bit) != LOW) {
          if (--loopCount == 0) { rc = DEVICE_ERROR_TIMEOUT; goto finish; }  
        }

        if ((micros() - t) > 40) { 
            bits[idx] |= mask;
        }
        mask >>= 1;
        if (mask == 0)   // next byte?
        {
            mask = 128;
            idx++;
        }
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
      humidity = (bits[0]*256 + bits[1]);
      temperature = ((bits[2] & 0x7F) * 256 + bits[3]);// * 0.1;
      // negative temperature
      if (bits[2] & 0x80) {
         temperature = -temperature;
      }
  }
  // TEST CHECKSUM
  if (bits[4] != crc) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }

  *_value = (SENS_READ_HUMD == _metric) ? humidity : temperature;
  if (!_wantsNumber) {
     ltoaf(*_value, _dst, 1);
  }

  rc = RESULT_IN_BUFFER;

  finish:
  startTimerOne(); 
  gatherSystemMetrics(); 
  return rc;
}


