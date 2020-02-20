#pragma once

#define WINSEN_UART_START_BYTE                    (0xFF)


uint8_t winsenUartRecieve(Stream&, const uint32_t, uint8_t*, const uint8_t);
uint8_t winsenCalcCrc(uint8_t*, const uint8_t);
