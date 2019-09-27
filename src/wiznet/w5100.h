#pragma once
#include "../net_platforms.h"
#ifdef NETWORK_ETHERNET_WIZNET


#include <SPI.h>

typedef uint8_t socket_t;

#define WIZNET_SOFT_RESET_COMMAND 0x80

//#define WIZNET_LARGE_BUFFERS_USED
//#define WIZNET_SOCKETS_USED                      0x01  // Use it if WIZNET_LARGE_BUFFERS_USED defied


/***************************************************/
/**            Default SS pin setting             **/
/***************************************************/
// If variant.h or other headers specifically define the
// default SS pin for ethernet, use it.
#if defined(PIN_SPI_SS_ETHERNET_LIB)
#define SS_PIN_DEFAULT  PIN_SPI_SS_ETHERNET_LIB

// For boards using AVR, assume shields with SS on pin 10
// will be used.  This allows for Arduino Mega (where
// SS is pin 53) and Arduino Leonardo (where SS is pin 17)
// to work by default with Arduino Ethernet Shield R2 & R3.
#elif defined(__AVR__)
#define SS_PIN_DEFAULT  10

// If variant.h or other headers define these names
// use them if none of the other cases match
#elif defined(PIN_SPI_SS)
#define SS_PIN_DEFAULT  PIN_SPI_SS
#elif defined(CORE_SS0_PIN)
#define SS_PIN_DEFAULT  CORE_SS0_PIN

// As a final fallback, use pin 10
#else
#define SS_PIN_DEFAULT  10
#endif

/***************************************************/
/**            Default socket setting             **/
/***************************************************/
// Define some values for settng socket size in init()
#if defined(NETWORK_ETHERNET_W5100)
//#warning "***** W5100 *****"

    #define WIZNET_SOCKETS_ONBOARD                   0x04

    // Only for W5100
    #define WIZNET_SOCKET_REGISTER_BASE_ADDRESS      0x0400
    #define WIZNET_SOCKET_REGISTER_SIZE              0x0100

    // Base for any socket size is equal
    #define WIZNET_TX_MEMORY_START_ADDRESS           0x4000
    #define WIZNET_RX_MEMORY_START_ADDRESS           0x6000

    #ifdef WIZNET_LARGE_BUFFERS_USED

       #if (WIZNET_SOCKETS_USED <= 1)
           #define WIZNET_SOCKET_SIZE                8192
           #define WIZNET_SOCKET_MEM_SIZE_REG_VALUE  0x03

       #elif (WIZNET_SOCKETS_USED <= 2)
           #define WIZNET_SOCKET_SIZE                4096
           #define WIZNET_SOCKET_MEM_SIZE_REG_VALUE  0x0A

       #else // #elif (WIZNET_SOCKETS_USED <= 2)
           #define WIZNET_SOCKET_SIZE                2048
           #define WIZNET_SOCKET_MEM_SIZE_REG_VALUE  0x55

       #endif // #elif (WIZNET_SOCKETS_USED <= 2) ... else
    #else // #ifdef WIZNET_LARGE_BUFFERS_USED
       #define WIZNET_SOCKET_SIZE                    2048
       #define WIZNET_SOCKET_MEM_SIZE_REG_VALUE      0x55
       #define WIZNET_SOCKETS_USED                   0x04
#endif // #ifdef WIZNET_LARGE_BUFFERS_USED


#elif defined(NETWORK_ETHERNET_W5500)
//#warning "***** W5500 *****"

    #define WIZNET_SOCKETS_ONBOARD                   0x08

    // Base for any socket size is equal
    #define WIZNET_TX_MEMORY_START_ADDRESS 0x8000
    #define WIZNET_RX_MEMORY_START_ADDRESS 0xC000

    #ifdef WIZNET_LARGE_BUFFERS_USED
       #if (WIZNET_SOCKETS_USED <= 1)
           #define WIZNET_SOCKET_SIZE                16384

       #elif (WIZNET_SOCKETS_USED <= 2)
           #define WIZNET_SOCKET_SIZE                8192

       #else // #elif (WIZNET_SOCKETS_USED <= 2)
           #define WIZNET_SOCKET_SIZE                4096

       #endif // #elif (WIZNET_SOCKETS_USED <= 2) ... else
    #else // #ifdef WIZNET_LARGE_BUFFERS_USED
       #define WIZNET_SOCKET_SIZE                    2048
       #define WIZNET_SOCKETS_USED                   0x08
#endif // #ifdef WIZNET_LARGE_BUFFERS_USED

       #define WIZNET_SOCKET_MEM_SIZE_REG_VALUE      (WIZNET_SOCKET_SIZE >> 10) // 16 for 16384, 8 for 8192...
       

#endif


class SnMR {
public:
  static const uint8_t CLOSE  = 0x00;
  static const uint8_t TCP    = 0x01;
  static const uint8_t UDP    = 0x02;
  static const uint8_t IPRAW  = 0x03;
  static const uint8_t MACRAW = 0x04;
  static const uint8_t PPPOE  = 0x05;
  static const uint8_t ND     = 0x20;
  static const uint8_t MULTI  = 0x80;
};

enum SockCMD {
  Sock_OPEN      = 0x01,
  Sock_LISTEN    = 0x02,
  Sock_CONNECT   = 0x04,
  Sock_DISCON    = 0x08,
  Sock_CLOSE     = 0x10,
  Sock_SEND      = 0x20,
  Sock_SEND_MAC  = 0x21,
  Sock_SEND_KEEP = 0x22,
  Sock_RECV      = 0x40
};

class SnIR {
public:
  static const uint8_t SEND_OK = 0x10;
  static const uint8_t TIMEOUT = 0x08;
  static const uint8_t RECV    = 0x04;
  static const uint8_t DISCON  = 0x02;
  static const uint8_t CON     = 0x01;
};

class SnSR {
public:
  static const uint8_t CLOSED      = 0x00;
  static const uint8_t INIT        = 0x13;
  static const uint8_t LISTEN      = 0x14;
  static const uint8_t SYNSENT     = 0x15;
  static const uint8_t SYNRECV     = 0x16;
  static const uint8_t ESTABLISHED = 0x17;
  static const uint8_t FIN_WAIT    = 0x18;
  static const uint8_t CLOSING     = 0x1A;
  static const uint8_t TIME_WAIT   = 0x1B;
  static const uint8_t CLOSE_WAIT  = 0x1C;
  static const uint8_t LAST_ACK    = 0x1D;
  static const uint8_t UDP         = 0x22;
  static const uint8_t IPRAW       = 0x32;
  static const uint8_t MACRAW      = 0x42;
  static const uint8_t PPPOE       = 0x5F;
};

/*
class IPPROTO {
public:
  static const uint8_t IP   = 0;
  static const uint8_t ICMP = 1;
  static const uint8_t IGMP = 2;
  static const uint8_t GGP  = 3;
  static const uint8_t TCP  = 6;
  static const uint8_t PUP  = 12;
  static const uint8_t UDP  = 17;
  static const uint8_t IDP  = 22;
  static const uint8_t ND   = 77;
  static const uint8_t RAW  = 255;
};

*/
class W5100Class {

public:
  W5100Class(void);
  uint8_t init();
//  void read_data(socket_t s, volatile uint16_t src, volatile uint8_t * dst, uint16_t len);
  void read_data(socket_t s, uint16_t src, uint8_t * dst, uint16_t len);
  void send_data_processing(socket_t s, const uint8_t *data, uint16_t len);
  void send_data_processing_offset(socket_t s, uint16_t data_offset, const uint8_t *data, uint16_t len);

  void recv_data_processing(socket_t s, uint8_t *data, uint16_t len, uint8_t peek = 0);

  inline void getGatewayIp(uint8_t *_addr)  { readGAR(_addr);   }
  inline void setGatewayIp(uint8_t *_addr)  { writeGAR(_addr);  }

  inline void getSubnetMask(uint8_t *_addr) { readSUBR(_addr);  }
  inline void setSubnetMask(uint8_t *_addr) { writeSUBR(_addr); }

  inline void getMACAddress(uint8_t *_addr) { readSHAR(_addr);  }
  inline void setMACAddress(uint8_t *_addr) { writeSHAR(_addr); }

  inline void getIPAddress(uint8_t *_addr)  { readSIPR(_addr);  }
  inline void setIPAddress(uint8_t *_addr)  { writeSIPR(_addr); }

  inline uint16_t getRetransmissionTime()   { return readRTR(); }
  inline uint8_t  getRetransmissionCount()  { return readRCR(); }

  inline void setRetransmissionTime(uint16_t _timeout) { writeRTR(_timeout); }
  inline void setRetransmissionCount(uint8_t _retry)   { writeRCR(_retry);   }
  
  inline uint8_t  getPHYCFGR()              { return 0; }

  uint8_t hardwareStatus();
  uint8_t linkStatus();

  // added to stock lib by sadman
  inline uint8_t getMaxSocketNumber() { return maxSocketNumber; }
  uint8_t softReset(void);

  // 



  void execCmdSn(socket_t s, SockCMD _cmd);
  
  uint16_t getTXFreeSize(socket_t s);
  uint16_t getRXReceivedSize(socket_t s);
  

  // W5100 Registers
  // ---------------
private:
// !!!!
#if defined(NETWORK_ETHERNET_W5100)

  static uint8_t write(uint16_t _addr, uint8_t _data);
  static uint16_t write(uint16_t addr, const uint8_t *buf, uint16_t len);
  static uint8_t read(uint16_t addr);
  static uint16_t read(uint16_t addr, uint8_t *buf, uint16_t len);

#define __GP_REGISTER8(name, address)             \
  static inline void write##name(uint8_t _data) { \
    write(address, _data);                        \
  }                                               \
  static inline uint8_t read##name() {            \
    return read(address);                         \
  }

#define __GP_REGISTER16(name, address)            \
  static void write##name(uint16_t _data) {       \
    write(address,   _data >> 8);                 \
    write(address+1, _data & 0xFF);               \
  }                                               \
  static uint16_t read##name() {                  \
    uint16_t res = read(address);                 \
    res = (res << 8) + read(address + 1);         \
    return res;                                   \
  }

#define __GP_REGISTER_N(name, address, size)      \
  static uint16_t write##name(uint8_t *_buff) {   \
    return write(address, _buff, size);           \
  }                                               \
  static uint16_t read##name(uint8_t *_buff) {    \
    return read(address, _buff, size);            \
  }

// !!!!
#elif defined(NETWORK_ETHERNET_W5500)

  static uint8_t  write(uint16_t _addr, uint8_t _cb, uint8_t _data);
  static uint16_t write(uint16_t _addr, uint8_t _cb, const uint8_t *buf, uint16_t len);
  static uint8_t  read(uint16_t _addr, uint8_t _cb );
  static uint16_t read(uint16_t _addr, uint8_t _cb, uint8_t *buf, uint16_t len);

#define __GP_REGISTER8(name, address)             \
  static inline void write##name(uint8_t _data) { \
    write(address, 0x04, _data);                  \
  }                                               \
  static inline uint8_t read##name() {            \
    return read(address, 0x00);                   \
  }

#define __GP_REGISTER16(name, address)            \
  static void write##name(uint16_t _data) {       \
    write(address,  0x04, _data >> 8);            \
    write(address+1, 0x04, _data & 0xFF);         \
  }                                               \
  static uint16_t read##name() {                  \
    uint16_t res = read(address, 0x00);           \
    res = (res << 8) + read(address + 1, 0x00);   \
    return res;                                   \
  }

#define __GP_REGISTER_N(name, address, size)      \
  static uint16_t write##name(uint8_t *_buff) {   \
    return write(address, 0x04, _buff, size);     \
  }                                               \
  static uint16_t read##name(uint8_t *_buff) {    \
    return read(address, 0x00, _buff, size);      \
  }

#endif
// !!!!

public:

  __GP_REGISTER8 (MR,     0x0000);    // Mode
  __GP_REGISTER_N(GAR,    0x0001, 4); // Gateway IP address
  __GP_REGISTER_N(SUBR,   0x0005, 4); // Subnet mask address
  __GP_REGISTER_N(SHAR,   0x0009, 6); // Source MAC address
  __GP_REGISTER_N(SIPR,   0x000F, 4); // Source IP address
  __GP_REGISTER8 (IR,     0x0015);    // Interrupt
  __GP_REGISTER8 (IMR,    0x0016);    // Interrupt Mask

// !!!!
#if defined(NETWORK_ETHERNET_W5100)
  __GP_REGISTER16(RTR,    0x0017);    // Timeout address
  __GP_REGISTER8 (RCR,    0x0019);    // Retry count
  __GP_REGISTER8 (RMSR,   0x001A);    // Receive memory size (W5100 only)
  __GP_REGISTER8 (TMSR,   0x001B);    // Transmit memory size (W5100 only)
  __GP_REGISTER8 (PATR,   0x001C);    // Authentication type address in PPPoE mode
  __GP_REGISTER8 (PTIMER, 0x0028);    // PPP LCP Request Timer
  __GP_REGISTER8 (PMAGIC, 0x0029);    // PPP LCP Magic Number
  __GP_REGISTER_N(UIPR,   0x002A, 4); // Unreachable IP address in UDP mode (W5100 only)
  __GP_REGISTER16(UPORT,  0x002E);    // Unreachable Port address in UDP mode (W5100 only)

#elif defined(NETWORK_ETHERNET_W5500)
  __GP_REGISTER16(RTR,    0x0019);    // Timeout address
  __GP_REGISTER8 (RCR,    0x001B);    // Retry count
  __GP_REGISTER8 (PTIMER, 0x001C);    // PPP LCP Request Timer
  __GP_REGISTER8 (PMAGIC, 0x001D);    // PPP LCP Magic Number
  __GP_REGISTER_N(PHAR,   0x001E, 6); // PPP Destination MAC Address
  __GP_REGISTER16(PSID,   0x0024);    // PPP Session Identification
  __GP_REGISTER16(PMRU,   0x0026);    // PPP Maximum Segment Size
  __GP_REGISTER_N(UIPR,   0x0028, 4); // Unreachable IP address in UDP mode
  __GP_REGISTER16(UPORT,  0x002C);    // Unreachable Port address in UDP mode
  __GP_REGISTER8 (PHYCFGR, 0x002E);   // PHY Configuration register, default value: 0b 1011 1xxx
  __GP_REGISTER8 (VERSIONR,0x0039);   // Chip Version Register (W5500 only)
#endif
// !!!!
  
#undef __GP_REGISTER8
#undef __GP_REGISTER16
#undef __GP_REGISTER_N



  // W5100 Socket registers
  // ----------------------
private:

  static inline uint8_t readSn(socket_t _s, uint16_t _addr);
  static inline uint8_t writeSn(socket_t _s, uint16_t _addr, uint8_t _data);
  static inline uint16_t readSn(socket_t _s, uint16_t _addr, uint8_t *_buf, uint16_t len);
  static inline uint16_t writeSn(socket_t _s, uint16_t _addr, uint8_t *_buf, uint16_t len);

#define __SOCKET_REGISTER8(name, address)                    \
  static inline void write##name(socket_t _s, uint8_t _data) { \
    writeSn(_s, address, _data);                             \
  }                                                          \
  static inline uint8_t read##name(socket_t _s) {            \
    return readSn(_s, address);                              \
  }

  // buffered writing (like W5500 or Ethernet 2.0.0) add more fat to code with no profit 
  // because W5100 does not support stream write and make many transfers to chip anyway
#define __SOCKET_REGISTER16(name, address)                   \
  static void write##name(socket_t _s, uint16_t _data) {     \
    writeSn(_s, address,     _data >> 8);                    \
    writeSn(_s, address + 1, _data & 0xFF);                  \
  }                                                          \
  static uint16_t read##name(socket_t _s) {                  \
    uint16_t data1 = readSn(_s, address);                    \
    uint16_t data2 = readSn(_s, address + 1);                \
    data1 = data1 << 8;                                      \
    data2 = data2 & 0xFF;                                    \
    return (data1 | data2);                                  \
  }

#define __SOCKET_REGISTER_N(name, address, size)             \
  static uint16_t write##name(socket_t _s, uint8_t *_buff) { \
    return writeSn(_s, address, _buff, size);                \
  }                                                          \
  static uint16_t read##name(socket_t _s, uint8_t *_buff) {  \
    return readSn(_s, address, _buff, size);                 \
  }

// !!!!
#if defined(NETWORK_ETHERNET_W5100)

// !!!!
#elif defined(NETWORK_ETHERNET_W5500)

  //static const uint16_t CH_BASE = 0x0000;
  //static const uint16_t CH_SIZE = 0x0000;
#endif
// !!!!

 
public:
  __SOCKET_REGISTER8(SnMR,        0x0000)        // Mode
  __SOCKET_REGISTER8(SnCR,        0x0001)        // Command
  __SOCKET_REGISTER8(SnIR,        0x0002)        // Interrupt
  __SOCKET_REGISTER8(SnSR,        0x0003)        // Status
  __SOCKET_REGISTER16(SnPORT,     0x0004)        // Source Port
  __SOCKET_REGISTER_N(SnDHAR,     0x0006, 6)     // Destination Hardw Addr
  __SOCKET_REGISTER_N(SnDIPR,     0x000C, 4)     // Destination IP Addr
  __SOCKET_REGISTER16(SnDPORT,    0x0010)        // Destination Port
  __SOCKET_REGISTER16(SnMSSR,     0x0012)        // Max Segment Size
  __SOCKET_REGISTER8(SnPROTO,     0x0014)        // Protocol in IP RAW Mode
  __SOCKET_REGISTER8(SnTOS,       0x0015)        // IP TOS
  __SOCKET_REGISTER8(SnTTL,       0x0016)        // IP TTL
  __SOCKET_REGISTER8(SnRX_SIZE,   0x001E)        // RX Memory Size (W5500 only)
  __SOCKET_REGISTER8(SnTX_SIZE,   0x001F)        // RX Memory Size (W5500 only)
  __SOCKET_REGISTER16(SnTX_FSR,   0x0020)        // TX Free Size
  __SOCKET_REGISTER16(SnTX_RD,    0x0022)        // TX Read Pointer
  __SOCKET_REGISTER16(SnTX_WR,    0x0024)        // TX Write Pointer
  __SOCKET_REGISTER16(SnRX_RSR,   0x0026)        // RX Free Size
  __SOCKET_REGISTER16(SnRX_RD,    0x0028)        // RX Read Pointer
  __SOCKET_REGISTER16(SnRX_WR,    0x002A)        // RX Write Pointer (supported?)
  
#undef __SOCKET_REGISTER8
#undef __SOCKET_REGISTER16
#undef __SOCKET_REGISTER_N


private:
//  static const uint16_t RSIZE = 0x0800; // Max Rx buffer size
//  static uint8_t initDone;
  static uint8_t ss_pin;
  static void setSS(uint8_t pin) { ss_pin = pin; }

  static const uint8_t maxSocketNumber = WIZNET_SOCKETS_USED;
  static const uint16_t socketMask = WIZNET_SOCKET_SIZE - 0x01;


public:
  static const uint16_t socketSize = WIZNET_SOCKET_SIZE;

  static uint16_t getSocketTxBaseAddr(uint8_t _socketNum) { return _socketNum * socketSize + WIZNET_TX_MEMORY_START_ADDRESS; }
  static uint16_t getSocketRxBaseAddr(uint8_t _socketNum) { return _socketNum * socketSize + WIZNET_RX_MEMORY_START_ADDRESS; }

#if defined(NETWORK_ETHERNET_W5500)
  static bool hasOffsetAddressMapping(void) { return true; }
#else 
  static bool hasOffsetAddressMapping(void) { return false; }
#endif

   static volatile uint8_t *ss_pin_reg;
   static uint8_t ss_pin_mask;

   inline static void initSS() { 
   	ss_pin_reg = portOutputRegister(digitalPinToPort(ss_pin));
   	ss_pin_mask = digitalPinToBitMask(ss_pin);
   	pinMode(ss_pin, OUTPUT);
   }
   inline static void setSS() {
   	*(ss_pin_reg) &= ~ss_pin_mask;
   }
   inline static void resetSS() {
   	*(ss_pin_reg) |= ss_pin_mask;
   }
};

extern W5100Class W5100;


// !!!!
#if defined(NETWORK_ETHERNET_W5100)

uint8_t  W5100Class::readSn(socket_t _s, uint16_t _addr) { return read(WIZNET_SOCKET_REGISTER_BASE_ADDRESS + _s * WIZNET_SOCKET_REGISTER_SIZE + _addr); }
uint8_t  W5100Class::writeSn(socket_t _s, uint16_t _addr, uint8_t _data) { return write(WIZNET_SOCKET_REGISTER_BASE_ADDRESS + _s * WIZNET_SOCKET_REGISTER_SIZE + _addr, _data); }
uint16_t W5100Class::readSn(socket_t _s, uint16_t _addr, uint8_t *_buf, uint16_t _len)  { return read(WIZNET_SOCKET_REGISTER_BASE_ADDRESS + _s * WIZNET_SOCKET_REGISTER_SIZE + _addr, _buf, _len); }
uint16_t W5100Class::writeSn(socket_t _s, uint16_t _addr, uint8_t *_buf, uint16_t _len) { return write(WIZNET_SOCKET_REGISTER_BASE_ADDRESS + _s * WIZNET_SOCKET_REGISTER_SIZE + _addr, _buf, _len); }

// !!!!
#elif defined(NETWORK_ETHERNET_W5500)

uint8_t W5100Class::readSn(socket_t _s, uint16_t _addr) {
    // 2.2.2 Control Phase
    // 0x05 - Select Mask , 0x08 - TX Buffer
    uint8_t controlByte = (_s << 0x05) + 0x08;
    return read(_addr, controlByte);
}

uint8_t W5100Class::writeSn(socket_t _s, uint16_t _addr, uint8_t _data) {
    // 2.2.2 Control Phase
    // 0x05 - Select Mask , 0x0C - RX Buffer (why not TX buff?)
    uint8_t controlByte = (_s << 0x05) + 0x0C;
    return write(_addr, controlByte, _data);
}

uint16_t W5100Class::readSn(socket_t _s, uint16_t _addr, uint8_t *_buf, uint16_t _len) {
    uint8_t controlByte = (_s << 0x05) + 0x08;
    return read(_addr, controlByte, _buf, _len );
}

uint16_t W5100Class::writeSn(socket_t _s, uint16_t _addr, uint8_t *_buf, uint16_t _len) {
    uint8_t controlByte = (_s << 0x05) + 0x0C;
    return write(_addr, controlByte, _buf, _len);
}

/*
void W5100Class::setPHYCFGR(uint8_t _val) {
  writePHYCFGR(_val);
}
*/

#endif
// !!!!

#endif // NETWORK_ETHERNET_WIZNET
