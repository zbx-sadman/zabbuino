/*

   network_hlp.h : helper header file which just bring physical network type to source code 

*/
#ifndef _ZABBUINO_NETWORK_HELPER_H_
#define _ZABBUINO_NETWORK_HELPER_H_

#include "../basic.h"
//#include "tune.h"

#if defined(W5100_ETHERNET_SHIELD)
   #define NETWORK_ETH_WIZNET
   #define NET_MODULE_NAME                                   "W5100"
#elif defined(W5200_ETHERNET_SHIELD)
   #define NETWORK_ETH_WIZNET
   #define NET_MODULE_NAME                                   "W5200"
#elif defined(W5500_ETHERNET_SHIELD)
   #define NETWORK_ETH_WIZNET
   #define NET_MODULE_NAME                                   "W5500"
#elif defined(ENC28J60_ETHERNET_SHIELD)
   #define NETWORK_ETH_ENC28J60
   #define NET_MODULE_NAME                                   "ENC28J60"
#elif defined(RS485_SHIELD)
   #define NETWORK_RS485
   #define NET_MODULE_NAME                                   "RS485"
#endif

#endif // _ZABBUINO_NETWORK_HELPER_H_

