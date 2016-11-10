#ifndef ZabbuinoBUSMICROWIRE_h
#define ZabbuinoBUSMICROWIRE_h

#include <Arduino.h>
#include "defaults.h"
#include "../zabbuino.h"

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

#define MAX7219_REGISTER_DECODE_MODE                                     0x09
#define MAX7219_REGISTER_INTENSITY                                       0x0A
#define MAX7219_REGISTER_SCANLIMIT                                       0x0B
#define MAX7219_REGISTER_SHUTDOWN                                        0x0C
#define MAX7219_REGISTER_DISPLAYTEST                                     0x0F

/*****************************************************************************************************************************
*
*  Send one byte to MAX7219 controller
*
*  Returns: 
*    - none
*
*****************************************************************************************************************************/
void writeByteToMAX7219(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _data);

/*****************************************************************************************************************************
*
*  Push one byte of data to MAX7219 controller
*
*  Returns: 
*    - none
*
*****************************************************************************************************************************/
void pushDataToMAX7219(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _loadPin, const uint8_t _register, const uint8_t _data);

/*****************************************************************************************************************************
*
*  Print incoming data on MAX7219 based digital led indicator or draw on led matrix.
*
*  Returns: 
*    - none
*
*****************************************************************************************************************************/
void printToMAX7219(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _loadPin, const uint8_t _intensity, char* _data);


#endif