#pragma once

#ifdef FEATURE_DEBUG_TO_SERIAL_HIGH
    #define DTSL(x) x
    #define DTSM(x) x
    #define DTSH(x) x
#else 
    #ifdef FEATURE_DEBUG_TO_SERIAL_MIDDLE
        #define DTSL(x) x
        #define DTSM(x) x
        #define DTSH(X) /* blank */
    #else 
        #ifdef FEATURE_DEBUG_TO_SERIAL_LOW
            #define DTSL(x) x
            #define DTSM(X) /* blank */
            #define DTSH(X) /* blank */
        #else 
            #define DTSL(X) /* blank */
            #define DTSM(X) /* blank */
            #define DTSH(X) /* blank */
        #endif 
    #endif 
#endif 


#ifdef FEATURE_DEBUG_TO_SERIAL_DEV
 #define DTSD(x) x
#else
 #define DTSD(X) /* blank */
#endif 


#ifdef FEATURE_NET_DEBUG_TO_SERIAL
 #define NDTS(x) x
#else
 #define NDTS(X) /* blank */
#endif 

