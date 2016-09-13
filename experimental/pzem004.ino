/*
Based on: https://github.com/olehs/PZEM004T
version 1.0 is used

*/

#define PZEM_VOLTAGE (uint8_t)0xB0
#define PZEM_CURRENT (uint8_t)0xB1
#define PZEM_POWER   (uint8_t)0xB2
#define PZEM_ENERGY  (uint8_t)0xB3
#define PZEM_PACKET_SIZE      0x07
#define DEFAULT_READ_TIMEOUT 1000

uint8_t crcPZEM004(uint8_t *_data, uint8_t _size) {
    uint16_t crc = 0;
    for(uint8_t i=0; i < _size; i++) { crc += *_data; _data++;}
    //while (_size) { crc += *_data; _data++; _size--; }
    return (uint8_t)(crc & 0xFF);
}

int32_t getPZEM004Metric(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _metric, const char* _ip, char* _outBuffer) {
  uint8_t buffer[PZEM_PACKET_SIZE],
          command;
  int32_t result;
  SoftwareSerial *swSerial;

  //if (NULL == swSerial) {
     swSerial = new SoftwareSerial(_rxPin, _txPin); 
     swSerial->begin(9600);
  //}

   switch (_metric) {
     case SENS_READ_AC:
       command = PZEM_CURRENT; 
       //Serial.println("Taking Voltage");
       break;
     case SENS_READ_VOLTAGE:
       command = PZEM_VOLTAGE; 
       //Serial.println("Taking Voltage");
       break;
     case SENS_READ_POWER:
       command = PZEM_POWER; 
       //Serial.println("Taking Power");
       break;
     case SENS_READ_ENERGY:
       command = PZEM_ENERGY; 
       //Serial.println("Taking Energy");
       break;
   }

   /*  Send to PZEM004 */
//    Serial.println("Send commmand...");
    
    // Make packet for PZEM
    // 1-th byte in the packet - metric (command)
    buffer[0] = command; 
    // 2..5 bytes - ip address. Convert its from _ip or use default (192.168.1.1) if _ip is invalid
    result = hstoba((uint8_t*) &buffer[1], _ip, 4);
    if (!result) {
       buffer[1] = 0xC0;  // 192
       buffer[2] = 0xA8;  // 168
       buffer[3] = 0x01;  // 1
       buffer[4] = 0x01;  // 1
    } 

    // 6-th byte - used to provide the value of the alarm threshold (in kW), 00 else
    buffer[5] = 0x00; 
    // 7-th byte - CRC
    buffer[6] = crcPZEM004(buffer, sizeof(buffer) - 1); 

    //for(int i=0; i < sizeof(buffer); i++) { Serial.print("Byte# "); Serial.print(i); Serial.print(" => "); Serial.println(buffer[i], HEX);  }

    if (swSerial) {
       // cleaning PZEM's outgoing buffer
       while(swSerial->available()) { swSerial->read(); }
       // Send request
       if (! swSerial->write(buffer, sizeof(buffer))) { return DEVICE_ERROR_TIMEOUT; };
    }

   /*  Recieve from PZEM004 */
    //Serial.println("Recieve answer...");
    
    unsigned long startTime = millis();
    uint8_t len = 0;
    
    while((len < PZEM_PACKET_SIZE) && (millis() - startTime < DEFAULT_READ_TIMEOUT)) {
        if(swSerial) {
            if(swSerial->available() > 0) {
                uint8_t c = (uint8_t) swSerial->read();
                if(!c && !len) {
                    continue; // skip 0 at startup
                }
                buffer[len++] = c;
                //Serial.print("Byte: "); Serial.println(c, HEX);
            }
        }
    }

    // Serial.println("Destroy current SoftwareSerial instance");
    swSerial->~SoftwareSerial();// 

    // Connection timeout occurs
    if (len != PZEM_PACKET_SIZE) { return DEVICE_ERROR_TIMEOUT; }
    // Wrong answer. buffer[0] must contain command - 0x10 (command B1 -> reply A1)
    //command = command - 0x10;
    //Serial.print("Header expected: "); Serial.println(command, HEX);
    if (buffer[0] != (command - 0x10)) { return DEVICE_ERROR_WRONG_ANSWER; }
    // Bad CRC
    if (buffer[6] != crcPZEM004(buffer, len - 1)) { return DEVICE_ERROR_CHECKSUM; }
    
//    Serial.println("Calculating...");
   // data is placed in buffer from 2-th byte, because 1-th byte is Header
   switch (_metric) {
     case SENS_READ_AC:
       result = ((buffer[1] << 8) + buffer[2]) * 100 + buffer[3];
       ltoaf(result, _outBuffer, 2);
       break;
     case SENS_READ_VOLTAGE:
       result = ((buffer[1] << 8) + buffer[2]) * 10 + buffer[3]; 
       ltoaf(result, _outBuffer, 1);
       break;
     case SENS_READ_POWER:
       result = (buffer[1] << 8) + buffer[2];
       ltoa(result, _outBuffer, 10);
       break;
     case SENS_READ_ENERGY:
       result = ((uint32_t) buffer[1] << 16) + ((uint16_t) buffer[2] << 8) + buffer[3];
       ltoa(result, _outBuffer, 10);
       break;
   }

  return RESULT_IN_BUFFER;
}

