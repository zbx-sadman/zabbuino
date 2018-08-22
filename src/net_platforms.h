#pragma once
/*

   net_platforms.h : helper header file which just bring physical network type to source code 

*/

#include "../cfg_basic.h"
//#include "cfg_tune.h"

#if defined(W5100_ETHERNET_SHIELD)
   #define NETWORK_ETH_WIZNET
   #define PHY_MODULE_NAME                                   "W5100"
#elif defined(W5200_ETHERNET_SHIELD)
   #define NETWORK_ETH_WIZNET
   #define PHY_MODULE_NAME                                   "W5200"
#elif defined(W5500_ETHERNET_SHIELD)
   #define NETWORK_ETH_WIZNET
   #define PHY_MODULE_NAME                                   "W5500"
#elif defined(ENC28J60_ETHERNET_SHIELD)
   #define NETWORK_ETH_ENC28J60
   #define PHY_MODULE_NAME                                   "ENC28J60"
/*
#elif defined(ESP8266_WIFI_SHIELD)
   #define NETWORK_ESP8266
   #define PHY_MODULE_NAME                                   "ESP8266"
#elif defined(RS485_SHIELD)
   #define NETWORK_RS485
   #define PHY_MODULE_NAME                                   "RS485"
*/
#endif

