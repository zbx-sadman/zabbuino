#pragma once

#define SPI_TYPE_HARDWARE                  0x00
#define SPI_TYPE_SOFTWARE                  0x01

/*****************************************************************************************************************************
*
*   Read one byte from SPI compatible bus
*
*****************************************************************************************************************************/
uint8_t spiReadByte(volatile uint8_t*, const uint8_t, volatile uint8_t*, const uint8_t);

/*****************************************************************************************************************************
*
*   Write one byte to SPI compatible bus
*
*****************************************************************************************************************************/
void spiWriteByte(const uint8_t, volatile uint8_t*, const uint8_t, volatile uint8_t*, const uint8_t, const uint8_t);