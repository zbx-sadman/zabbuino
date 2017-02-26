#include "RS485.h"

// Create internal SoftSerial object) on init
RS485Class::RS485Class() {
}
 
// Create internal SoftSerial object) on init
uint8_t RS485Class::begin(uint16_t _baudRate, uint8_t _rxPin, uint8_t _txPin, uint8_t _busModePin) {
   return begin(_baudRate, _rxPin, _txPin, _busModePin, false);
}

uint8_t RS485Class::begin(uint16_t _baudRate, uint8_t _rxPin, uint8_t _txPin, uint8_t _busModePin, uint8_t _inverseLogic) {
  UART = new SoftwareSerial(_rxPin, _txPin, _inverseLogic);
  UART->begin(_baudRate);
  // One character send time (in uS)
  //waitAfterSendmS = ((1000000UL / (_baudRate / 10)) * 2) + 1;
  waitAfterSendmS = (10000 / _baudRate) + 1;
  busModePin = _busModePin;
  pinMode(busModePin , OUTPUT);
  switchBusToRX();
  init();
  return true;
}
 
RS485Class::~RS485Class() {
  // Destroy SoftwareSerial object
  UART->~SoftwareSerial();
}
/*
void RS485Class::begin(uint32_t _address) {
  localAddress = _address;
  // init the pointers to the buffer's read/write positions
  init();
}

void RS485Class::begin(uint8_t _octet0, uint8_t _octet1, uint8_t _octet2, uint8_t _octet3) {
  localAddress[0] = _octet0;
  localAddress[1] = _octet1;
  localAddress[2] = _octet2;
  localAddress[3] = _octet3;
  init();
}
*/

void RS485Class::init() {
  buffReadPos =  buffWritePos = 0;
  dataAvailable = false;    
  pktRecieved   = false;
  pktType       = RS485_PKTTYPE_NONE;
  remoteAddress = RS485_INADDR_NONE;
}

int RS485Class::maintain() {
  // Do some system work
  tick();
  // 0: nothing happened
  return 0;
}

uint8_t RS485Class::connect(const NetworkAddress _dst) {
    // No real connect maded, just fill destination address
    remoteAddress = _dst;
    // fake 'socket' number is always 1
    return 1;
}


int32_t RS485Class::ping(const NetworkAddress _dstAddress) {
  uint32_t startTime;
  DTSM( Serial.print("[*] Sending PING packet to 0x"); Serial.println((uint32_t) _dstAddress, HEX); )
  startTime = millis();
  // Connect to destination address & send PING message to it
  // connect(_dstAddress);
  //uint16_t messageSize = createPacket((uint8_t) RS485_PKTTYPE_PING);
  send(buffer, createPacket(_dstAddress, RS485_PKTTYPE_PING));
  init();
  // Wait for PONG & return reply time on success or -1 on fail
  return ((-1 == waitForPacket(_dstAddress, RS485_PKTTYPE_PONG)) ? -1 : (millis() - startTime));
};



void RS485Class::send(uint8_t* _src, uint16_t _size) {
  switchBusToTX();
  DTSM( Serial.print("[*] Sending data: "); )
  while (_size) {
    DTSD( Serial.print(" 0x"); Serial.print(*_src, HEX); )
    UART->write(*_src);
    _src++;
    _size--;
  }
  DTSM( Serial.print("\n[*] Sending finished\n"); )
  delay(waitAfterSendmS);
  switchBusToRX();
}


/**********************************************************************************************************************/
void RS485Class::tick() {
  uint8_t rc, testAndFlushPacket;
  int16_t incomingData;
  uint16_t calculatedCRC;

  testAndFlushPacket  = false;
  
//  incomingData = UART->peek();
//  DTSD( Serial.print("Peek: 0x"); Serial.println(incomingData, HEX); )

  // Do nothing with buffer if not all data is readed from buffer
  // May be need to do UART.read() to flush incoming data?
  if (dataAvailable) { 
      UART->read();
      return; 
  }

  // .read() can replace .available(), because returns -1 if no data in buffer or SoftwareSerial instance do not listening and byte (unit8_t) if data exists in buffer
  if (!UART->available()) { return; } 
  
  incomingData = UART->read();
  //if (-1 == incomingData) { return; } 
  incomingData = (uint8_t) incomingData;
  DTSD( Serial.print("Rcv: 0x"); Serial.println(incomingData, HEX); )
  // Analyze incoming byte and assembly the packet
  rc = assemblePacket(incomingData);
  
  // going to next round if packet not recieved
  if (true != rc) { return; }
  
  // Just look to part of buffer thru "packet_header_t" glasses
  packetHeader = (packet_header_t*) &buffer[RS485_PREAMBLE_SIZE];

  // Calculate CRC, check it and do nothing if CRC is wrong (packet is corrupted)
  calculatedCRC = crc16(&buffer[RS485_CRC_CALC_FROM], (buffWritePos-RS485_CRC_CALC_FROM));
  DTSD( Serial.print("[*] Recieved CRC: "); Serial.print(packetHeader->CRC, HEX); Serial.print(", calculated CRC: "); Serial.println(calculatedCRC, HEX); )
  if (calculatedCRC != packetHeader->CRC) { return; }

  // Convert to "Network to Host Long" byte order
  packetHeader->srcAddr = htonl(packetHeader->srcAddr);
  packetHeader->dstAddr = htonl(packetHeader->dstAddr);
  DTSD( Serial.print("[*] Dst addr: "); Serial.println((uint32_t) packetHeader->dstAddr, HEX); )

  // Wrong destination address, stop processing 
  // TODO: What about broadcast?
  if (packetHeader->dstAddr != localAddress) { return; }
  // Packet from me to me, ANY or NONE address? No, it is spoof. Stop processing.
  if (localAddress      == packetHeader->srcAddr ||
      RS485_INADDR_NONE == packetHeader->srcAddr ||
      RS485_INADDR_ANY  == packetHeader->srcAddr) { return; }

  // Packet is recieved and validated
  DTSD( Serial.print("[*] Packet validated\n"); )
  DTSD( Serial.print("[*] Src addr: "); Serial.println((uint32_t) packetHeader->srcAddr, HEX); )
  // Imitate session connection to remote address to make able sending outgoing packets until session stop()'ed
  //remoteAddress = packetHeader->srcAddr;
  connect(packetHeader->srcAddr);
  
  DTSM( Serial.print("[*] Recieved packet from "); Serial.print((uint32_t) packetHeader->srcAddr, HEX); 
         Serial.print(" to me ("); Serial.print((uint32_t) packetHeader->dstAddr, HEX); 
         Serial.print("), CRC OK ("); Serial.print(calculatedCRC, HEX); 
         Serial.println(")"); 
      )
  
  pktRecieved = true;
  pktType = packetHeader->type;
  DTSM( Serial.print("[*] Packet type: "); )
      switch (pktType) {
        case RS485_PKTTYPE_PING:
          // PING can be be sended by any host and PONG must be returned back
          DTSM( Serial.print("PING\n[*] Send back PONG\n"); )
          send(buffer, createPacket(remoteAddress, RS485_PKTTYPE_PONG));
          init();
          break;
        case RS485_PKTTYPE_PONG:
          // no additional action
          DTSM( Serial.print("PONG\n"); )
          testAndFlushPacket = true;
          break;
        case RS485_PKTTYPE_ACK:
          // Flush unexpected packet
          testAndFlushPacket = true;
          DTSM( Serial.print("ACK\n"); )
          break;
        case RS485_PKTTYPE_DATA:
          // DATA can be be sended by any host and ACK must be returned back
          DTSM( Serial.print("DATA\n[*] Send back ACK\n"); )
          send(buffer, createPacket(remoteAddress, RS485_PKTTYPE_ACK));
          // Now data available in the buffer oficially (but preamble & header data must be masked)
          dataAvailable = true;
          buffReadPos = RS485_PREAMBLE_SIZE + RS485_HEADER_SIZE;
          break;
        default:
          DTSD( Serial.print("UNKNOWN\n"); )
          break;
      }   

      
      if (testAndFlushPacket && !(pktExpectedType == pktType && pktExpectedAddress == remoteAddress)) {
            pktExpectedType = -1;
            pktExpectedAddress = RS485_INADDR_NONE;
      }

}

/**********************************************************************************************************************/
uint8_t RS485Class::assemblePacket(const uint8_t _src) {
  static stage_t stage = ST_NOP;
  static uint8_t storeData, preambleCnt, rc;
  static uint16_t dataSize;

  // Always ready to catch preamble/sync block 
  preambleCnt = (RS485_CONTROL_PREAMBLE == _src) ? (preambleCnt + 1) : 0;
  // Real preable must be larger that the protective preable, because first byte can be lost on UART interrupt waking-up
  if (RS485_ENOUGHT_PREAMBLE_SIZE <= preambleCnt) {  
      preambleCnt = 0; 
      DTSD( puts("[X] Preamble stage\n"); )
      // SOH control byte must follow the preamble
      stage = ST_SOH; 
  }

  // Store data in the buffer only on several stages of assembling
  if (storeData) {
     if (RS485_BUFFER_SIZE > buffWritePos) { 
        buffer[buffWritePos] = _src;
     } else {
        // buffer overflow, just drop all data
        stage = ST_INIT; 
     }
  } 

  // Assembly stages
  rc = false;
  switch (stage) {
     // NOP stage 
     // Just wait for preamble
     case ST_NOP:
       // return false;
       break; 
       
     // INIT stage
     // All variables take initial value
     case ST_INIT:
       storeData = false;
       DTSD( puts("[X] INIT stage\n"); )
       stage = ST_NOP;
       break;

     // SOH wait stage
     // Skip all preamble control bytes until SOH detected.
     case ST_SOH:
       switch (_src) {
          case RS485_CONTROL_PREAMBLE:
            break;
          case RS485_CONTROL_SOH:
            // Later we just look to part of buffer thru "packet_header_t" glasses
            packetHeader = (packet_header_t*) &buffer[RS485_PREAMBLE_SIZE];
            // Reserve space for preamble & SOH byte
            // buffWritePos var will be increased at end of subroutine, and SOH size (+1 byte) will be take in account
            buffWritePos = RS485_PREAMBLE_SIZE;
            storeData = true;
            DTSD( puts("[X] SOH stage\n"); )
            // Jump to ST_HEADER recieving stage if SOH detected
            stage = ST_HEADER; 
            break;
          // Unexpected char detected - it's break all data structure, assembling must be canceled
          default:
           // Packet have wrong start sequence
           DTSD( puts("[!] Wrong structure\n"); )
           stage = ST_INIT; 
           break;
       }
       break;

     // HEADER store stage
     // Write all bytes to the buffer
     case ST_HEADER:
       // -1 is SOH byte that already counted on ST_SOH stage
       if ((RS485_HEADER_SIZE + RS485_PREAMBLE_SIZE - 1) <= buffWritePos) { 
          //DTSD( printf("Datasize: %u\n", packetHeader->dataSize); )
          DTSD( puts("[X] HEADER stage\n"); )
          // Service packets have not user data. Packet have enough data to analyzing.
          if (0 >= packetHeader->dataSize) {
             //buffWritePos++;
             stage = ST_INIT; 
             // Finished
             rc = true;
          } else {
             dataSize = buffWritePos + packetHeader->dataSize;
             stage = ST_DATA;
          }
       }
       break;

     // DATA store stage
     // Still write incoming bytes to the buffer
     case ST_DATA:
       // Stop when read number of bytes
       if (dataSize <= buffWritePos) { 
          DTSD( puts("[X] DATA stage\n"); )
          stage = ST_INIT; 
          // Finished
          rc = true;
       }
       break;
    default:
      // Wrong stage, internal error
      break;
  }
  
   //if (storeData) { buffWritePos++; }
   buffWritePos++; 
  return rc;
}


uint16_t RS485Class::createPacket(const NetworkAddress _dstAddress, const uint8_t _pktType) {
  return createPacket(_dstAddress, _pktType, NULL); 
}

uint16_t RS485Class::createPacket(const NetworkAddress _dstAddress, const uint8_t _pktType, const uint8_t* _src) { 
  char preamble[] = "\xFF\xFF\xFF\xFF\xFF\xFF";
  uint16_t buffWriteLocalPos;
  
  //  Make outgoing packet:
  //    1. Copy preamble to buffer
  strcpy((char*) &buffer, (char*) &preamble);
  //    ...-1 mean 'drop trailing \0' 
  buffWriteLocalPos = sizeof(preamble) - 1;
     
  //    2. Put service data to packet header  
  // Just look to part of buffer thru "packet_header_t" glasses
  packetHeader = (packet_header_t*) &buffer[buffWriteLocalPos];
  buffWriteLocalPos += sizeof(packet_header_t);
   
  // SOH control byte
  packetHeader->SOH=0x01;
  // RS control byte
  packetHeader->RS=0x1E;
  // Need send addresses in Big Endian byte order
 // packetHeader->dstAddr = _dstAddress;
  //packetHeader->srcAddr = localAddress;
  // Convert to "Host to Network Long" byte order
  packetHeader->dstAddr = htonl(_dstAddress);
  packetHeader->srcAddr = htonl(localAddress);
  packetHeader->version=0x01;
  packetHeader->type=_pktType;
  packetHeader->dataSize = 0x00; 
  
  //    3. Copy key to buffer if need
  if (NULL != _src) {
     packetHeader->dataSize = strlen((char*) _src); 
     strcpy((char*) &buffer[buffWriteLocalPos ], (char*) _src);
     buffWriteLocalPos  += packetHeader->dataSize;
     // Add trailing '\n' 
     buffer[buffWriteLocalPos++] = '\n';
     packetHeader->dataSize++;
  }
  //     4. Calculate CRC and store it
  packetHeader->CRC = crc16(&buffer[RS485_CRC_CALC_FROM], buffWriteLocalPos-RS485_CRC_CALC_FROM);
   
   //for (int i=0; i < 21 ; i++) { printf(" 0x%02X", buffer[i]); }
       
   return buffWriteLocalPos;
}


int32_t RS485Class::waitForPacket(const NetworkAddress _fromAddress, const uint8_t _pktType) {
  uint32_t startTime, currTime;
  int32_t rc = -1;
  startTime = millis();
  // Wait for incoming packet
  pktExpectedType = _pktType;
  pktExpectedAddress = _fromAddress;
  init();
  do {
     tick();
     currTime = millis();
     // Return number of ms if correct packet from specified address recieved
     if (pktRecieved && _pktType == pktType && _fromAddress == remoteAddress) { rc = currTime - startTime; goto finish; }
  } while (commTimeout > (currTime - startTime));
   // return -1 if timeout occured
  
  finish:
  pktExpectedType = -1;
  pktExpectedAddress = RS485_INADDR_NONE;
  return rc;
}


// Calculate 16-bit CRC-CCITT with using _crc_ccitt_update() from AVRLib
uint16_t RS485Class::crc16 (const uint8_t *_src, uint16_t _len) { 
  uint8_t inbyte;
  uint16_t crc = 0xFFFF;
  //Serial.print("CRC calculating...\n");
  while (_len) {
    _len--;
    inbyte = *_src++;
    //Serial.print("inbyte: 0x"); Serial.println(inbyte, HEX);
    crc = _crc_ccitt_update(crc, inbyte);
    }  // end of while
  return crc;
  //Serial.print("\n");
}  // end of crc16


void RS485Class::switchBusToRX() {
 DTSH( Serial.println("Bus RX"); )
 digitalWrite(busModePin, RS485_RX_STATE);   
}

void RS485Class::switchBusToTX() {
 DTSH( Serial.println("Bus TX"); )
 digitalWrite(busModePin, RS485_TX_STATE);
}


IPAddress RS485Class::localIP() {
  // Return address in Network byte order because IPAddress class want it
  //return IPAddress(htonl((uint32_t) localAddress));
  return IPAddress((uint32_t) localAddress);
}

IPAddress RS485Class::subnetMask() {
  return IPAddress(0x00UL);
}

IPAddress RS485Class::gatewayIP() {
  return IPAddress(0x00UL);
}

IPAddress RS485Class::dnsServerIP() {
  return IPAddress(0x00UL);
}

