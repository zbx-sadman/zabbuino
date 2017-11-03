#ifndef _ZABBUINO_STRUCTS_H_
#define _ZABBUINO_STRUCTS_H_

#include "NetworkAddress.h"

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                         PROGRAMM STRUCTURES SECTION 
*/

// Note: netconfig_t size must be no more ___uint8_t___ bytes, because readConfig()'s read cycle use uint8_t counter. 
// Change the index's variable type if bigger size need
#pragma pack(push,1)
typedef struct {
  uint8_t CRC;                                    // 1 byte, CRC stores first, due it's EEPROM cell rewrites everytime on config saving if it changed. 
                                                  //         When the cell was broken, we just shift start config store address to next cell.
  // struct {
  // uint8_t useProtection: 1;                    // 1 bit
  // uint8_t useDHCP: 1;         	 	  // 1 bit
  // }
  uint8_t useProtection;                          // 1 byte
  uint8_t useDHCP;         	 		  // 1 byte
  uint8_t macAddress[6];                          // 6 byte 
  NetworkAddress ipAddress;     		  // 4 byte (uint8_t[])
  NetworkAddress ipNetmask;     		  // 4 byte (uint8_t[])
  NetworkAddress ipGateway;     	          // 4 byte (uint8_t[])
  char hostname[constAgentHostnameMaxLength+1];     // +1 for '\0', 255 - (1 + 1 + 1 + 6 + 4*4 + 2 + 1) = 227 bytes max
  // #ifdef ... #elif ... #endif does not work with struct operator
  uint32_t password;                              // 4 byte
  int16_t tzOffset;                               // 2 byte
} netconfig_t ;


typedef struct {
  uint32_t sysCmdCount;                           // Number of executed commands 
  uint8_t  sysCmdLast;                            // Index of last executed command
  uint32_t sysCmdLastExecTime;                    // End time of last executed command
  uint32_t sysCmdTimeMax;                         // Maximum spend time for command execution
  uint8_t  sysCmdTimeMaxN;                        // Index of the slowest command
//  uint16_t sysVCC;
  uint16_t sysVCCMin;                             // Maximum VCC (in mV) from MCU powering on
  uint16_t sysVCCMax;                             // Minimum VCC (in mV) from MCU powering on
  uint32_t sysLCDLastUsedTime;                    // Last time, when System LCD was used 
  uint32_t sysRamFree;                            // "Current" free memory (in bytes).
  uint32_t sysRamFreeMin;                         // Minimum free memory (in bytes) from MCU powering on
  uint32_t netPHYReinits;                         // PHY reinits number (restarts of network module)
} sysmetrics_t;


typedef struct {                                  // 9 bytes: 
  uint32_t value;        			  
  uint8_t owner;                                  // 1 byte 
  // mode == -1 => Interrupt is not attached
  int8_t mode;                                    // 1 byte 
  volatile uint8_t *encTerminalAPIR;              // 1 byte
  volatile uint8_t *encTerminalBPIR;              // 1 byte
  uint8_t encTerminalAPinBit;                     // 1 byte
  uint8_t encTerminalBPinBit;                     // 1 byte
} extInterrupt_t ;
#pragma pack(pop) 

#endif //_ZABBUINO_STRUCTS_H_
