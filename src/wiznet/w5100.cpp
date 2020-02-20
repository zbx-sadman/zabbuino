#include "../net_platforms.h"
#ifdef NETWORK_ETHERNET_WIZNET

/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
//#include <stdio.h>
//#include <string.h>

#include "w5100.h"


uint8_t  W5100Class::ss_pin = SS_PIN_DEFAULT;
// pointers and bitmasks for optimized SS pin
volatile uint8_t * W5100Class::ss_pin_reg;

// W5100 controller instance
W5100Class W5100;

uint8_t W5100Class::ss_pin_mask;

W5100Class::W5100Class(){
  SPI.begin();
  initSS();
}

uint8_t W5100Class::init(void) {
  uint8_t rc = false;
  // hardwareStatus() testing need because softReset() can return 0x00 if chip disconnected
  if (!softReset() || !hardwareStatus()) { goto finish; }
 
// !!!!
#if defined(NETWORK_ETHERNET_W5100)
  writeTMSR(WIZNET_SOCKET_MEM_SIZE_REG_VALUE);
  writeRMSR(WIZNET_SOCKET_MEM_SIZE_REG_VALUE);
  rc = true;

// !!!!
#elif defined(NETWORK_ETHERNET_W5500)
  for (uint8_t i = 0x00; i < WIZNET_SOCKETS_ONBOARD; i++) {
       // Configure used sockets...
       if (i < maxSocketNumber) {
          writeSnRX_SIZE(i, WIZNET_SOCKET_MEM_SIZE_REG_VALUE);
          writeSnTX_SIZE(i, WIZNET_SOCKET_MEM_SIZE_REG_VALUE);
       // and unused (other)
       } else {
          writeSnRX_SIZE(i, 0x00);
          writeSnTX_SIZE(i, 0x00);
       }
  }
  rc = true;
#endif
// !!!!

finish:
  return rc;

}

uint8_t W5100Class::hardwareStatus() {
  uint8_t rc = 0x00;
#if defined(NETWORK_ETHERNET_W5100)
  // Store Mode Register
  uint8_t oldMR = readMR();  
  // toggle 0x10 (ping block)
  uint8_t testMR = oldMR ^ 0x10;
  // Write test value to Mode Register
  writeMR(testMR);
  // Test value by reading. If we get the same - chip is works (probaly)
  if (readMR() == testMR) { rc = 51; }
  // Restore Mode Register
  writeMR(oldMR);
#elif defined(NETWORK_ETHERNET_W5500)
  // VERSIONR always indicates the W5500 version as 0x04.
  if (0x04 == readVERSIONR()) { rc = 55; }
#endif
  return rc;
}

uint8_t W5100Class::linkStatus() {
  uint8_t rc = 0x00;
/*
#if defined(NETWORK_ETHERNET_W5100)
  if (WIZNET_SOCKET_MEM_SIZE_REG_VALUE == readTMSR()) { rc = 51; }
#elif defined(NETWORK_ETHERNET_W5500)
  // VERSIONR always indicates the W5500 version as 0x04.
  if (0x04 == readVERSIONR()) { rc = 55; }
#endif
*/
  return rc;
}


// Soft reset the Wiznet chip, by writing to its MR register reset bit
uint8_t W5100Class::softReset(void) {
   uint16_t count = 0x00;
   // write to reset bit
   writeMR(WIZNET_SOFT_RESET_COMMAND);
   // then wait for soft reset to complete
   do {
      	uint8_t mr = readMR();
   	if (mr == 0x00) { return true; }
   	delay(1);
   } while (++count < 20);
   return false;
}
   
uint16_t W5100Class::getTXFreeSize(socket_t s) {
    uint16_t val=0, val1=0;
    // !!! timeout, wdt
    do {
        val1 = readSnTX_FSR(s);
        if (val1 != 0)
            val = readSnTX_FSR(s);
    } 
    while (val != val1);
    return val;
}

uint16_t W5100Class::getRXReceivedSize(socket_t s) {
  uint16_t val=0,val1=0;
    // !!! timeout, wdt
  do {
    val1 = readSnRX_RSR(s);
    if (val1 != 0)
      val = readSnRX_RSR(s);
  } 
  while (val != val1);
  return val;
}


void W5100Class::send_data_processing(socket_t s, const uint8_t *data, uint16_t len) {
  // This is same as having no offset in a call to send_data_processing_offset
  send_data_processing_offset(s, 0, data, len);
}

void W5100Class::send_data_processing_offset(socket_t s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
// !!!!
#if defined(NETWORK_ETHERNET_W5100)

  uint16_t ptr = readSnTX_WR(s);
  ptr += data_offset;
  uint16_t offset = ptr & socketMask;
  uint16_t dstAddr = offset + getSocketTxBaseAddr(s);

  if (offset + len > socketSize) {
    // Wrap around circular buffer
    uint16_t size = socketSize - offset;
    write(dstAddr, data, size);
    write(getSocketTxBaseAddr(s), data + size, len - size);
  } else {
    write(dstAddr, data, len);
  }
  ptr += len;
  writeSnTX_WR(s, ptr);

// !!!!
#elif defined(NETWORK_ETHERNET_W5500)

    uint16_t ptr = readSnTX_WR(s);
    uint8_t cntl_byte = (0x14 + (s << 0x05) );
    ptr += data_offset;
    write(ptr, cntl_byte, data, len);
    ptr += len;
    writeSnTX_WR(s, ptr);

#endif
// !!!!

}


void W5100Class::recv_data_processing(socket_t s, uint8_t* data, uint16_t len, uint8_t peek) {
  uint16_t ptr;
  ptr = readSnRX_RD(s);
  read_data(s, ptr, data, len);
  if (!peek)
  {
    ptr += len;
    writeSnRX_RD(s, ptr);
  }
}

//void W5100Class::read_data(socket_t s, volatile uint16_t src, volatile uint8_t *dst, uint16_t len)
void W5100Class::read_data(socket_t s, uint16_t src, uint8_t* dst, uint16_t len) {
// !!!!
#if defined(NETWORK_ETHERNET_W5100)
  uint16_t size;
  uint16_t src_mask;
  uint16_t src_ptr;
  
  src_mask = (uint16_t)src & socketMask;
  src_ptr = getSocketRxBaseAddr(s) + src_mask;
  
  if (hasOffsetAddressMapping() || src_mask + len <= socketSize) {
     read(src_ptr, dst, len);
  } else {
     size = socketSize - src_mask;
     read(src_ptr, dst, size);
     dst += size;
     read(getSocketRxBaseAddr(s), dst, len - size);
  }
// !!!!
#elif defined(NETWORK_ETHERNET_W5500)
    uint8_t cntl_byte = (0x18 + (s << 0x05));
    read((uint16_t)src , cntl_byte, (uint8_t*) dst, len);
#endif
// !!!!

}


// !!!!
#if defined(NETWORK_ETHERNET_W5100)
uint8_t W5100Class::write(uint16_t _addr, uint8_t _data) {
  setSS();  
  SPI.transfer(0xF0);
  SPI.transfer(_addr >> 0x08);
  SPI.transfer(_addr &  0xFF);
  SPI.transfer(_data);
  resetSS();

  return 1;
}
// !!!!
#elif defined(NETWORK_ETHERNET_W5500)

uint8_t W5100Class::write(uint16_t _addr, uint8_t _controlByte, uint8_t _data) {
    setSS();  
    SPI.transfer(_addr >> 0x08);
    SPI.transfer(_addr &  0xFF);
    SPI.transfer(_controlByte);
    SPI.transfer(_data);
    resetSS();
    return 1;
}
#endif
// !!!!



// !!!!
#if defined(NETWORK_ETHERNET_W5100)
uint16_t W5100Class::write(uint16_t _addr, const uint8_t* _buf, uint16_t _len) {
  for (uint16_t i = 0x00; i < _len; i++)
  {
    setSS();    
    SPI.transfer(0xF0);
    SPI.transfer(_addr >> 0x08);
    SPI.transfer(_addr &  0xFF);
    _addr++;
    SPI.transfer(_buf[i]);
    resetSS();
  }
  return _len;
}
// !!!!
#elif defined(NETWORK_ETHERNET_W5500)
uint16_t W5100Class::write(uint16_t _addr, uint8_t _controlByte, const uint8_t* _buf, uint16_t _len)
{
    setSS();
    SPI.transfer(_addr >> 0x08);
    SPI.transfer(_addr &  0xFF);
    SPI.transfer(_controlByte);
    for (uint16_t i = 0x00; i < _len; i++) { SPI.transfer(_buf[i]); }
    resetSS();
    return _len;
}

#endif
// !!!!



// !!!!
#if defined(NETWORK_ETHERNET_W5100)
uint8_t W5100Class::read(uint16_t _addr)
{
  setSS();  
  SPI.transfer(0x0F);
  SPI.transfer(_addr >> 0x08);
  SPI.transfer(_addr &  0xFF);
  uint8_t _data = SPI.transfer(0x00);
  resetSS();
  return _data;
}
// !!!!
#elif defined(NETWORK_ETHERNET_W5500)
uint8_t W5100Class::read(uint16_t _addr, uint8_t _controlByte)
{
    setSS();
    SPI.transfer(_addr >> 0x08);
    SPI.transfer(_addr &  0xFF);
    SPI.transfer(_controlByte);
    uint8_t _data = SPI.transfer(0x00);
    resetSS();
    return _data;
}
#endif
// !!!!


// !!!!
#if defined(NETWORK_ETHERNET_W5100)
uint16_t W5100Class::read(uint16_t _addr, uint8_t *_buf, uint16_t _len)
{
  for (uint16_t i=0; i<_len; i++)
  {
    setSS();
    SPI.transfer(0x0F);
    SPI.transfer(_addr >> 0x08);
    SPI.transfer(_addr &  0xFF);
    _addr++;
    _buf[i] = SPI.transfer(0x00);
    resetSS();
  }
  return _len;
}
// !!!!
#elif defined(NETWORK_ETHERNET_W5500)

uint16_t W5100Class::read(uint16_t _addr, uint8_t _controlByte, uint8_t *_buf, uint16_t _len)
{ 
    setSS();
    SPI.transfer(_addr >> 0x08);
    SPI.transfer(_addr &  0xFF);
    SPI.transfer(_controlByte);
    for (uint16_t i = 0x00; i < _len; i++){
        _buf[i] = SPI.transfer(0x00);
    }
    resetSS();
    return _len;
}
#endif
// !!!!


void W5100Class::execCmdSn(socket_t s, SockCMD _cmd) {
  // Send command to socket
  writeSnCR(s, _cmd);
  // Wait for command to complete
  // !!! timeout, wdt
  while (readSnCR(s))
    ;
}



#endif // NETWORK_ETHERNET_WIZNET