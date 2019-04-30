#pragma once

/* 

The original code was written for the Wiring board by:
 * Nicholas Zambetti and Dave Mellis /Interaction Design Institute Ivrea /Dec 2004
 * http://www.potemkin.org/uploads/Wiring/MAX7219.txt
 
First modification by:
 * Marcus Hannerstig/  K3, malmö högskola /2006
 * http://www.xlab.se | http://arduino.berlios.de
 
Second modification is by:
 * tomek ness /FH-Potsdam / Feb 2007
 * http://design.fh-potsdam.de/

*/

// Uncomment to save Progmem stace, but loose ASCII support
//#define  NO_ASCII_SUPPORT

#define MAX7219_REGISTER_DECODE_MODE                                     (0x09)
#define MAX7219_REGISTER_INTENSITY                                       (0x0A)
#define MAX7219_REGISTER_SCANLIMIT                                       (0x0B)
#define MAX7219_REGISTER_SHUTDOWN                                        (0x0C)
#define MAX7219_REGISTER_DISPLAYTEST                                     (0x0F)

#ifndef NO_ASCII_SUPPORT
typedef struct {
  char sign;
  uint8_t draw;
} letterDrawing_t;

//
//  ASCII processing
//
// currByte '1111110' =>  LED SEG 'ABCDEFG' , if DP must be fired up - currByte |= 0x80
//    AAAA
//   F    B
//   F    B    
//    GGGG
//   E    C
//   E    C
//    DDDD   DP
//

const letterDrawing_t letterDrawings [] PROGMEM = {
      { ' ', B0000000 },  
      { '0', B1111110 },
      { 'O', B1111110 },
      { '1', B0110000 },
      { '2', B1101101 },
      { '3', B1111001 },
      { '4', B0110011 },
      { '5', B1011011 },
      { '6', B1011111 },
      { '7', B1110000 },
      { '8', B1111111 },
      { '9', B1111011 },
      { '-', B0000001 },
      { '_', B0001000 },
      { 'A', B1110111 },
      { 'b', B0011111 },
      { 'C', B1001110 },
      { 'c', B0001101 },
      { 'd', B0111101 },
      { 'H', B0110111 },
      { 'h', B0010111 },
      { 'E', B1001111 },
      { 'L', B0001110 },
      { 'l', B0000110 },
      { 'P', B1100111 },
      { 'n', B0010101 },
      { 'o', B0011101 },
      { 'r', B0000101 },
      { 't', B0001111 },
      { 'u', B0011100 },
      { 'U', B0111110 },
};
#endif

/*****************************************************************************************************************************
*
*  Print incoming data on MAX7219 based digital led indicator or draw on led matrix.
*
*  Returns: 
*    - none
*
*****************************************************************************************************************************/
void writeToMAX7219(const uint8_t, const uint8_t, const uint8_t, const uint8_t, char*);

