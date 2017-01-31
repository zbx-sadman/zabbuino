#include "../transport_hlp.h"
#ifdef TRANSPORT_ETH_ENC28J60

#ifndef _UIP_UIP_DEBUG_H_
#define _UIP_UIP_DEBUG_H_

//#include <inttypes.h>
//#include <Arduino.h>
#include <HardwareSerial.h>
#include "uip.h"

class UIPDebug {

public:

  static void uip_debug_printconns();
  static bool uip_debug_printcon(struct uip_conn *lhs,struct uip_conn *rhs);
  static void uip_debug_printbytes(const uint8_t *data, uint8_t len);

};


#endif // _UIP_UIP_DEBUG_H_
#endif // TRANSPORT_ETH_ENC28J60
