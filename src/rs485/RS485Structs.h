#ifndef RS485STRUCTS_H
#define RS485STRUCTS_H

#include "..\NetworkAddress.h"

// <***> Do not use 0xFF to avoid wrong preamble catching
#define RS485_PKTVER              0x01 // Do not use 0xFF to avoid wrong preamble catching

#define RS485_CONTROL_SOH         0x01 // SOH control byte
#define RS485_CONTROL_RS          0x1E // RS

#define RS485_PKTTYPE_NONE        0x00 // 
#define RS485_PKTTYPE_DATA        0x02 // STX
#define RS485_PKTTYPE_ACK         0x06 // ACK 
#define RS485_PKTTYPE_ENQ         0x05 // ENQ 
#define RS485_PKTTYPE_NACK        0x15 // NACK
#define RS485_PKTTYPE_PING        0x0E // SO, use as PING packet ID
#define RS485_PKTTYPE_PONG        0x0F // SI, use as PONG packet ID
// </***>


#define RS485_CONTROL_PREAMBLE    0xFF // Preamble chars to start incoming data processing
#define RS485_PREAMBLE_SIZE       0x06
#define RS485_ENOUGHT_PREAMBLE_SIZE (RS485_PREAMBLE_SIZE - 1)
#define RS485_HEADER_SIZE         16

// RS485_SOH_OFFSET is (RS485_SOH_OFFSET+1) item of array, because arrays starts from 0 item.
#define RS485_SOH_OFFSET          (RS485_PREAMBLE_SIZE + 0)
#define RS485_CRC_OFFSET          (RS485_PREAMBLE_SIZE + 1)
#define RS485_DATASIZE_OFFSET     (RS485_PREAMBLE_SIZE + 3)
#define RS485_RS_OFFSET           (RS485_PREAMBLE_SIZE + 5)
#define RS485_DSTADDR_OFFSET      (RS485_PREAMBLE_SIZE + 6)
#define RS485_PKTVERS_OFFSET      (RS485_PREAMBLE_SIZE + 10)
#define RS485_PKTTYPE_OFFSET      (RS485_PREAMBLE_SIZE + 11)
#define RS485_SRCADDR_OFFSET      (RS485_PREAMBLE_SIZE + 12)

#define RS485_CRC_CALC_FROM       RS485_DATASIZE_OFFSET

#define RS485_TX_STATE            HIGH
#define RS485_RX_STATE            LOW

#define RS485_INADDR_NONE         ((uint32_t) 0xFFFFFFFFUL)  /* 255.255.255.255 */
#define RS485_INADDR_ANY          ((uint32_t) 0x00000000UL)  /* 0.0.0.0 */
#define RS485_INADDR_BROADCAST    ((uint32_t) 0xFFFFFFFFUL)  /* 255.255.255.255 */

typedef enum {
  ST_NOP,
  ST_INIT,
  ST_SOH,
  ST_HEADER,
  ST_DATA,
}  stage_t;

#pragma pack(push, 1)
typedef struct {                                  
  uint8_t         SOH;
  uint16_t        CRC;
  uint16_t        dataSize;
  uint8_t         RS;
  NetworkAddress  dstAddr;
  uint8_t         version;
  uint8_t         type;
  NetworkAddress  srcAddr;
} packet_header_t;
#pragma pack(pop)

#endif
