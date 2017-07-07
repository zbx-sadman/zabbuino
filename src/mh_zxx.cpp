#include "mh_zxx.h"

/*****************************************************************************************************************************
*
*   Calculate CRC of Winsen MH-Zxx CO2 sensor data packet
*
*   Returns: 
*     - CRC
*
*****************************************************************************************************************************/
static uint8_t crcMHZxx(uint8_t* _data) {
    uint8_t crc = 0;
    for(uint8_t i = 1; i < (MH_ZXX_PACKET_SIZE-1); i++) { crc += _data[i]; }
    crc = 0xFF - crc;
    crc += 1;  
    return crc;
}

/*****************************************************************************************************************************
*
*   Overloads of main subroutines. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getMHZxxMetricUART(const uint8_t _rxPin, const uint8_t _txPin, int32_t* _value) {
  uint8_t stubBuffer;
  return getMHZxxMetricUART(_rxPin, _txPin, &stubBuffer, _value, true);
}


int8_t getMHZxxMetricUART(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _dst) {
  int32_t stubValue;
  return getMHZxxMetricUART(_rxPin, _txPin, _dst, &stubValue, false);
}


int8_t getMHZxxMetricPWM(const uint8_t _pin, const uint16_t _range, int32_t* _value) {
  uint8_t stubBuffer;
  return getMHZxxMetricPWM(_pin, _range, &stubBuffer, _value, true);
}

int8_t getMHZxxMetricPWM(const uint8_t _pin, const uint16_t _range, uint8_t* _dst) {
  int32_t stubValue;
  return getMHZxxMetricPWM(_pin, _range, _dst, &stubValue, false);
}

/*****************************************************************************************************************************
*
*   Read specified metric's value of the Winsen MH-Zxx CO2 sensor via UART, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IN_BUFFER on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*
*****************************************************************************************************************************/
int8_t getMHZxxMetricUART(const uint8_t _rxPin, const uint8_t _txPin, uint8_t* _dst, int32_t* _value, const uint8_t _wantsNumber) {
  uint8_t len, rc = DEVICE_ERROR_TIMEOUT;
  
  SoftwareSerial swSerial(_rxPin, _txPin);
  swSerial.begin(MH_ZXX_UART_SPEED);

  _dst[MH_ZXX_STARTING_BYTE] = 0xFF;                           // Starting byte
  _dst[MH_ZXX_SENSOR_NUMBER] = 0x01;                           // Sensor No.
  _dst[MH_ZXX_CMD] = MH_ZXX_CMD_GAS_CONCENTRATION;             // Command
  _dst[3] = _dst[4] = _dst[5] = _dst[6] = _dst[7] = 0x00;      // Stub bytes
  _dst[MH_ZXX_CRC] = 0x79;                                     // Check value

  // The serial stream can get out of sync. The response starts with 0xff, try to resync : https://github.com/jehy/arduino-esp8266-mh-z19-serial/blob/master/arduino-esp8266-mhz-19-serial.ino
  //  Send command to MH-Zxx
  serialSend(&swSerial, _dst, MH_ZXX_PACKET_SIZE, !UART_SLOW_MODE);

  //  Recieve from MH-Zxx
  //  It actually do not use '\r', '\n', '\0' to terminate string
  len = serialRecive(&swSerial, _dst, MH_ZXX_PACKET_SIZE, MH_ZXX_DEFAULT_READ_TIMEOUT, !UART_STOP_ON_CHAR, '\r', !UART_SLOW_MODE);
 
  // Connection timeout occurs
  if (len < MH_ZXX_PACKET_SIZE) { rc = DEVICE_ERROR_TIMEOUT; goto finish; }

  // Wrong answer. buffer[0] must contain 0xFF
  if (0xFF != _dst[MH_ZXX_STARTING_BYTE]) { rc = DEVICE_ERROR_WRONG_ANSWER; goto finish; }

  // Bad CRC
  // CRC calculate for bytes #1..#9 (byte #0 excluded)
  if (_dst[MH_ZXX_CRC] != crcMHZxx(_dst)) { rc = DEVICE_ERROR_CHECKSUM; goto finish; }

  *_value = 256 * _dst[MH_ZXX_GAS_CONCENTRATION_HIGH_BYTE];
  *_value += _dst[MH_ZXX_GAS_CONCENTRATION_LOW_BYTE];

  if (!_wantsNumber) {
     ultoa(*_value, (char*) _dst, 10);
  }

  rc = RESULT_IN_BUFFER;


  finish:
  gatherSystemMetrics(); // Measure memory consumption
  swSerial.~SoftwareSerial(); 
  return rc;

}


/*****************************************************************************************************************************
*
*  Read specified metric's value of the Winsen MH-Zxx CO2 sensor via PWM, put it to output buffer on success. 
*
*  Returns: 
*    - RESULT_IN_BUFFER on success
*    - DEVICE_ERROR_ACK_L
*    - DEVICE_ERROR_ACK_H
*    - DEVICE_ERROR_TIMEOUT if sensor stops answer to the request
*
*****************************************************************************************************************************/
int8_t getMHZxxMetricPWM(uint8_t _pin, uint16_t _range, uint8_t* _dst, int32_t* _value, const uint8_t _wantsNumber) {

//  volatile uint8_t *PIR;
  uint8_t stage = MH_ZXX_STAGE_WAIT_FOR_LOW, 
          pinState, // pinBit, pinPort,
          rc = DEVICE_ERROR_ACK_L;
          
  uint32_t startTime, highTime, lowTime, nowTime;

  
  /* 
  pinBit = digitalPinToBitMask(_pin);
  pinPort = digitalPinToPort(_pin);
  *PIR = portInputRegister(pinPort);
*/
  pinMode(_pin, INPUT_PULLUP);

  stopTimerOne(); 
  startTime = millis();
  
  do {
     nowTime = millis();
     pinState = digitalRead(_pin);
//     pinState = *PIR & pinBit;

     switch (stage) {
       case MH_ZXX_STAGE_WAIT_FOR_LOW:  
         if (LOW == pinState) { 
             rc = DEVICE_ERROR_ACK_H;
             stage = MH_ZXX_STAGE_WAIT_FOR_HIGH; 
         }
         break;

       case MH_ZXX_STAGE_WAIT_FOR_HIGH:  
         if (HIGH == pinState) { 
            rc = DEVICE_ERROR_TIMEOUT;
            stage = MH_ZXX_STAGE_COUNT_FOR_HIGH; 
            highTime = nowTime;
         }
         break;

       case MH_ZXX_STAGE_COUNT_FOR_HIGH:
         if (LOW == pinState) { 
            highTime = nowTime - highTime;
            lowTime = nowTime;
            stage = MH_ZXX_STAGE_COUNT_FOR_LOW; 
         }
         break;

       case MH_ZXX_STAGE_COUNT_FOR_LOW:  
         if (HIGH == pinState) { 
            lowTime = nowTime - lowTime;
            stage = MH_ZXX_STAGE_CYCLE_FINISHED; 
            goto finish; 
         }
         break;       

     }   
  } while ( nowTime - startTime < MH_ZXX_CYCLE_TIME);
  
  finish:
  if (MH_ZXX_STAGE_CYCLE_FINISHED == stage) {
/* 
 Serial.print("high = ");
  Serial.print(highTime);
  Serial.print(" ; low = ");
  Serial.print(lowTime);
  Serial.println();
*/
 
     *_value = _range * (highTime - 2) / (highTime + lowTime - 4);
     if (!_wantsNumber) {
        ultoa(*_value, (char*) _dst, 10);
     }
     rc = RESULT_IN_BUFFER;
  } 
  
  startTimerOne(); 
  gatherSystemMetrics(); 

  return rc;   


}
