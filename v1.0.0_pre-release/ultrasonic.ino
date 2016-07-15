// Based on:
//          Ultrasonic - Library for HR-SC04 Ultrasonic Ranging Module.
//          GitHub: https://github.com/JRodrigoTech/Ultrasonic-HC-SR04


uint32_t ultrasonicRanging(const uint8_t _triggerPin, const uint8_t _echoPin)
{
  uint32_t result;
  // timeout in microseconds. 38010 * 100 / 58 => 65534. It is out of distance range (too close or too far).
  uint16_t timeOut =  38010;  

  // Between pings must be silence period for 50ms or more
  // delay(50);
  pinMode(_triggerPin, OUTPUT);
  pinMode(_echoPin, INPUT);
  
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);
  
  // Using IO trigger for at least 10us high level signal, 
  digitalWrite(_triggerPin, HIGH);
  delayMicroseconds(10);
  
  digitalWrite(_triggerPin, LOW);
    
  // how much is the result if no obstacle exists ?
  result = pulseIn(_echoPin, HIGH, timeOut);
  
  if ( 0 == result) { result = timeOut; }
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  // 29 * 2 = 58

  // Resolution : 0.3 cm => 3mm. 
  // Return result in mm
  return (result * 100  / 58 ) ;
}
