#include "../net_platforms.h"
#ifdef NETWORK_ETH_WIZNET

#pragma once

#include "w5100.h"

extern uint8_t socket(socket_t s, uint8_t protocol, uint16_t port, uint8_t flag); // Opens a socket(TCP or UDP or IP_RAW mode)
extern void close(socket_t s); // Close socket
extern uint8_t connect(socket_t s, uint8_t * addr, uint16_t port); // Establish TCP connection (Active connection)
extern void disconnect(socket_t s); // disconnect the connection
extern uint8_t listen(socket_t s);	// Establish TCP connection (Passive connection)
extern uint16_t send(socket_t s, const uint8_t * buf, uint16_t len); // Send data (TCP)
extern int16_t recv(socket_t s, uint8_t * buf, int16_t len);	// Receive data (TCP)
extern uint16_t peek(socket_t s, uint8_t *buf);
extern uint16_t sendto(socket_t s, const uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t port); // Send data (UDP/IP RAW)
extern uint16_t recvfrom(socket_t s, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t *port); // Receive data (UDP/IP RAW)
extern void flush(socket_t s); // Wait for transmission to complete

extern uint16_t igmpsend(socket_t s, const uint8_t * buf, uint16_t len);

// Functions to allow buffered UDP send (i.e. where the UDP datagram is built up over a
// number of calls before being sent
/*
  @brief This function sets up a UDP datagram, the data for which will be provided by one
  or more calls to bufferData and then finally sent with sendUDP.
  @return 1 if the datagram was successfully set up, or 0 if there was an error
*/
extern int startUDP(socket_t s, uint8_t* addr, uint16_t port);
/*
  @brief This function copies up to len bytes of data from buf into a UDP datagram to be
  sent later by sendUDP.  Allows datagrams to be built up from a series of bufferData calls.
  @return Number of bytes successfully buffered
*/
uint16_t bufferData(socket_t s, uint16_t offset, const uint8_t* buf, uint16_t len);
/*
  @brief Send a UDP datagram built up from a sequence of startUDP followed by one or more
  calls to bufferData.
  @return 1 if the datagram was successfully sent, or 0 if there was an error
*/
int sendUDP(socket_t s);

#endif // NETWORK_ETH_WIZNET