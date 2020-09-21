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
// currByte '1111110' =>  LED SEG 'ABCDEFG' , if DP must be fired up - just set last bit (.. |= 0x80)
//    AAAA
//   F    B
//   F    B    
//    GGGG
//   E    C
//   E    C
//    DDDD   DP
//

const letterDrawing_t letterDrawings [] PROGMEM = {
      { ' ', B00000000 },  
      { '0', B01111110 },
      { 'O', B01111110 },
      { '1', B00110000 },
      { '2', B01101101 },
      { '3', B01111001 },
      { '4', B00110011 },
      { '5', B01011011 },
      { '6', B01011111 },
      { '7', B01110000 },
      { '8', B01111111 },
      { '9', B01111011 },
      { '-', B00000001 },
      { '_', B00001000 },
      { 'A', B01110111 },
      { 'b', B00011111 },
      { 'C', B01001110 },
      { 'c', B00001101 },
      { 'd', B00111101 },
      { 'H', B00110111 },
      { 'h', B00010111 },
      { 'E', B01001111 },
      { 'L', B00001110 },
      { 'l', B00000110 },
      { 'P', B01100111 },
      { 'n', B00010101 },
      { 'o', B00011101 },
      { 'r', B00000101 },
      { 't', B00001111 },
      { 'u', B00011100 },
      { 'U', B00111110 },
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
void writeToMAX7219(const uint8_t, const uint8_t, const uint8_t, const uint8_t, uint8_t*);

