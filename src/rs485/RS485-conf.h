#ifndef RS485CONF_H
#define RS485CONF_H

// Need to choose size as 2^X to correct work of circular buffer
// note that buffer position pointers is uint8_t - RS485_BUFFER_SIZE must be <= 256
#define RS485_BUFFER_SIZE           128

#define RS485_DEFAULT_SPEED         9600 // Bps
#define RS485_DEFAULT_RX_PIN        12   // D12
#define RS485_DEFAULT_TX_PIN        11   // D11
#define RS485_DEFAULT_BUSMODE_PIN   10   // D11


#define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)


#endif
