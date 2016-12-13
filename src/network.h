#ifndef _ZABBUINO_NETWORK_H_
#define _ZABBUINO_NETWORK_H_

#include "../basic.h"
#include "tune.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                               NETWORK SECTION 
*/

// Include headers for an network module
#if defined (W5100_ETHERNET_SHIELD)
   #define NET_MODULE_NAME                                   "W5100"
   #include <Ethernet.h> 
   #include <SPI.h>
#elif defined(W5200_ETHERNET_SHIELD)
   #define NET_MODULE_NAME                                   "W5200"
   #include <Ethernet.h>
   #include <SPI.h>
#elif defined(W5500_ETHERNET_SHIELD)
   #define NET_MODULE_NAME                                   "W5500"
   #include <Ethernet.h>
   #include <SPI.h>
#elif defined(ENC28J60_ETHERNET_SHIELD)
   #define NET_MODULE_NAME                                   "ENC28J60"
   #include <UIPEthernet.h>
#endif

#endif // _ZABBUINO_NETWORK_H_

