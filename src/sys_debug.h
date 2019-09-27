#pragma once

#if (FEATURE_DEBUG_MESSAGING_LEVEL > 0)
#define DEBUG_MESSAGING_LEVEL_LOW

#if (FEATURE_DEBUG_MESSAGING_LEVEL > 1)
#define DEBUG_MESSAGING_LEVEL_MIDDLE

#if (FEATURE_DEBUG_MESSAGING_LEVEL > 2)
#define DEBUG_MESSAGING_LEVEL_HIGH
#endif
#endif
#endif


#if defined(DEBUG_MESSAGING_LEVEL_HIGH)
#define __DMLH(_code) _code
#else
#define __DMLH(_code) /* blank */
#endif

#if defined(DEBUG_MESSAGING_LEVEL_MIDDLE)
#define __DMLM(_code) _code
#else
#define __DMLM(_code) /* blank */
#endif

#if defined(DEBUG_MESSAGING_LEVEL_LOW)
#define __DMLL(_code) _code
#else
#define __DMLL(_code) /* blank */
#endif



#ifdef FEATURE_DEBUG_TO_SERIAL_DEV
 #define __DMLD(x) x
#else
 #define __DMLD(X) /* blank */
#endif 


#ifdef FEATURE_NET_DEBUG_TO_SERIAL
 #define NDTS(x) x
#else
 #define NDTS(X) /* blank */
#endif 


