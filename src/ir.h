/*

Based on: https://github.com/z3t0/Arduino-IRremote
version 2.2.1 is used

 !!! BEWARE !!! 
 Not all Arduino IRremote code is implemented to Zabbuino. Compilation may stops or hangs.
 #define's for Timer2 & Timer4  are included only to this .ino file. 

*/

#ifndef _ZABBUINO_IR_H_
#define _ZABBUINO_IR_H_

#include "../basic.h"
#include "tune.h"
#include "structs.h"
#include "service.h"

#define IR_UNKNOWN      -0x01
#define IR_UNUSED       0x00
#define IR_RC5          0x01
#define IR_RC6          0x02
#define IR_NEC          0x03
#define IR_SONY         0x04
#define IR_PANASONIC    0x05
#define IR_JVC          0x06
#define IR_SAMSUNG      0x07
#define IR_WHYNTER      0x08
#define IR_AIWA_RC_T501 0x09
#define IR_LG           0x0A
#define IR_SANYO        0x0B
#define IR_MITSUBISHI   0x0C
#define IR_DISH         0x0D
#define IR_SHARP        0x0E
#define IR_DENON        0x0F
#define IR_PRONTO       0x10
#define IR_LEGO_PF      0x11

//------------------------------------------------------------------------------
// CPU Frequency
//
#ifdef F_CPU
#	define SYSCLOCK  F_CPU     // main Arduino clock
#else
#	define SYSCLOCK  16000000  // main Arduino clock
#endif



//------------------------------------------------------------------------------
// Define which timer to use
//
// Uncomment the timer you wish to use on your board.
// If you are using another library which uses timer2, you have options to
//   switch IRremote to use a different timer.
//

// Arduino Mega
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	//#define IR_USE_TIMER1   // tx = pin 11
	#define IR_USE_TIMER2     // tx = pin 9
	//#define IR_USE_TIMER3   // tx = pin 5
	//#define IR_USE_TIMER4   // tx = pin 6
	//#define IR_USE_TIMER5   // tx = pin 46

// Arduino Duemilanove, Diecimila, LilyPad, Mini, Fio, Nano, etc
#else
	//#define IR_USE_TIMER1   // tx = pin 9
	#define IR_USE_TIMER2     // tx = pin 3

#endif

//------------------------------------------------------------------------------
// Defines for Timer

//---------------------------------------------------------
// Timer2 (8 bits)
//
#if defined(IR_USE_TIMER2)
#define TIMER_RESET
#define TIMER_ENABLE_PWM    (TCCR2A |= _BV(COM2B1))
#define TIMER_DISABLE_PWM   (TCCR2A &= ~(_BV(COM2B1)))
#define TIMER_ENABLE_INTR   (TIMSK2 = _BV(OCIE2A))
#define TIMER_DISABLE_INTR  (TIMSK2 = 0)
#define TIMER_INTR_NAME     TIMER2_COMPA_vect

#define TIMER_CONFIG_KHZ(val) ({ \
	const uint8_t pwmval = SYSCLOCK / 2000 / (val); \
	TCCR2A               = _BV(WGM20); \
	TCCR2B               = _BV(WGM22) | _BV(CS20); \
	OCR2A                = pwmval; \
	OCR2B                = pwmval / 3; \
})

#define TIMER_COUNT_TOP  (SYSCLOCK * USECPERTICK / 1000000)

//-----------------
#if (TIMER_COUNT_TOP < 256)
#	define TIMER_CONFIG_NORMAL() ({ \
		TCCR2A = _BV(WGM21); \
		TCCR2B = _BV(CS20); \
		OCR2A  = TIMER_COUNT_TOP; \
		TCNT2  = 0; \
	})
#else
#	define TIMER_CONFIG_NORMAL() ({ \
		TCCR2A = _BV(WGM21); \
		TCCR2B = _BV(CS21); \
		OCR2A  = TIMER_COUNT_TOP / 8; \
		TCNT2  = 0; \
	})
#endif
//-----------------
#if defined(CORE_OC2B_PIN)
#	define TIMER_PWM_PIN  CORE_OC2B_PIN  // Teensy
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#	define TIMER_PWM_PIN  9              // Arduino Mega
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
#	define TIMER_PWM_PIN  14             // Sanguino
#else
#	define TIMER_PWM_PIN  3              // Arduino Duemilanove, Diecimila, LilyPad, etc
#endif
#endif


uint8_t sendCommandByIR(const uint8_t _irPacketType, const uint8_t _nbits, const uint32_t _data, const uint8_t _repeat, const uint32_t _address);
uint8_t sendRawByIR(const uint16_t _frequency, unsigned int _nBits, const char* _data);
static void mark (unsigned int time);
static void space (unsigned int time);
void enableIROut (int khz);
static void custom_delay_usec(unsigned long uSecs);
#ifdef SUPPORT_IR_RC5
#define MIN_RC5_SAMPLES     11

#define RC5_T1             889
  #define RC5_RPT_LENGTH   46000
  void  sendRC5 (unsigned long data,  int nbits);
#endif

#ifdef SUPPORT_IR_RC6
  #define MIN_RC6_SAMPLES      1
  #define RC6_HDR_MARK      2666
  #define RC6_HDR_SPACE      889
  #define RC6_T1             444
  #define RC6_RPT_LENGTH   46000
  void  sendRC6 (unsigned long data,  int nbits);
#endif

#ifdef SUPPORT_IR_NEC
  #define NEC_BITS          32
  #define NEC_HDR_MARK    9000
  #define NEC_HDR_SPACE   4500
  #define NEC_BIT_MARK     560
  #define NEC_ONE_SPACE   1690
  #define NEC_ZERO_SPACE   560
  #define NEC_RPT_SPACE   2250
  void  sendNEC (unsigned long data,  int nbits);
#endif

#ifdef SUPPORT_IR_SONY
  #define SONY_BITS                   12
  #define SONY_HDR_MARK             2400
  #define SONY_HDR_SPACE             600
  #define SONY_ONE_MARK             1200
  #define SONY_ZERO_MARK             600
  #define SONY_RPT_LENGTH          45000
  #define SONY_DOUBLE_SPACE_USECS    500  // usually ssee 713 - not using ticks as get number wrapround
  void  sendSony (unsigned long data,  int nbits);
#endif

#ifdef SUPPORT_IR_SAMSUNG
  #define SAMSUNG_BITS          32
  #define SAMSUNG_HDR_MARK    5000
  #define SAMSUNG_HDR_SPACE   5000
  #define SAMSUNG_BIT_MARK     560
  #define SAMSUNG_ONE_SPACE   1600
  #define SAMSUNG_ZERO_SPACE   560
  #define SAMSUNG_RPT_SPACE   2250
  void  sendSAMSUNG (unsigned long data,  int nbits);
#endif


#ifdef SUPPORT_IR_WHYNTER
  #define WHYNTER_BITS          32
  #define WHYNTER_HDR_MARK    2850
  #define WHYNTER_HDR_SPACE   2850
  #define WHYNTER_BIT_MARK     750
  #define WHYNTER_ONE_MARK     750
  #define WHYNTER_ONE_SPACE   2150
  #define WHYNTER_ZERO_MARK    750
  #define WHYNTER_ZERO_SPACE   750
  void  sendWhynter (unsigned long data,  int nbits);
#endif


#ifdef SUPPORT_IR_LG
  #define LG_BITS 28
  #define LG_HDR_MARK 8000
  #define LG_HDR_SPACE 4000
  #define LG_BIT_MARK 600
  #define LG_ONE_SPACE 1600
  #define LG_ZERO_SPACE 550
  #define LG_RPT_LENGTH 60000
  void  sendLG (unsigned long data,  int nbits);
#endif

#ifdef SUPPORT_IR_DISH
  #define DISH_BITS          16
  #define DISH_HDR_MARK     400
  #define DISH_HDR_SPACE   6100
  #define DISH_BIT_MARK     400
  #define DISH_ONE_SPACE   1700
  #define DISH_ZERO_SPACE  2800
  #define DISH_RPT_SPACE   6200
  void  sendDISH (unsigned long data,  int nbits);
#endif

#ifdef SUPPORT_IR_SHARP
  #define SHARP_BITS             15
  #define SHARP_BIT_MARK        245
  #define SHARP_ONE_SPACE      1805
  #define SHARP_ZERO_SPACE      795
  #define SHARP_GAP          600000
  #define SHARP_RPT_SPACE      3000
  #define SHARP_TOGGLE_MASK  0x3FF
  void sendSharpRaw (unsigned long data,  int nbits);
  void sendSharp (unsigned int address,  unsigned int command);
#endif



#ifdef SUPPORT_IR_DENON
  #define BITS          14  // The number of bits in the command
  #define HDR_MARK     300  // The length of the Header:Mark
  #define HDR_SPACE    750  // The lenght of the Header:Space
  #define BIT_MARK     300  // The length of a Bit:Mark
  #define ONE_SPACE   1800  // The length of a Bit:Space for 1's
  #define ZERO_SPACE   750  // The length of a Bit:Space for 0's
  void sendDenon (unsigned long data,  int nbits);
#endif


//+=============================================================================
#ifdef SUPPORT_IR_PANASONIC
  #define PANASONIC_BITS          48
  #define PANASONIC_HDR_MARK    3502
  #define PANASONIC_HDR_SPACE   1750
  #define PANASONIC_BIT_MARK     502
  #define PANASONIC_ONE_SPACE   1244
  #define PANASONIC_ZERO_SPACE   400
  void sendPanasonic(unsigned int address,  unsigned long data);
#endif


#ifdef SUPPORT_IR_JVC
  #define JVC_BITS           16
  #define JVC_HDR_MARK     8000
  #define JVC_HDR_SPACE    4000
  #define JVC_BIT_MARK      600
  #define JVC_ONE_SPACE    1600
  #define JVC_ZERO_SPACE    550
  #define JVC_RPT_LENGTH  60000
  void sendJVC (unsigned long data,  int nbits,  bool repeat);
#endif



#endif // #ifndef _ZABBUINO_IR_H_