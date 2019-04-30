#pragma once


#define COMPILER_WARNING_UNUSED_VAR_SUPRESSION

#ifdef COMPILER_WARNING_UNUSED_VAR_SUPRESSION
#define __SUPPRESS_WARNING_UNUSED(_code) (void)(_code)
#else
#define __SUPPRESS_WARNING_UNUSED(_code) (void)0x00
#endif


#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)
                         

