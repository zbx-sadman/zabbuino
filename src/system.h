#pragma once

// How long the ID of MCU (in bytes)
#if defined(ARDUINO_ARCH_AVR)
  // variable type used with portOutputRegister/portInputRegister/portModeRegister 
  #define ioRegister_t uint8_t
  const uint8_t constMcuIdSize                                    = 0x0A;
  const uint8_t constMcuIdStartAddress                            = 0x0E;
#elif defined(ARDUINO_ARCH_ESP8266)
  // variable type used with portOutputRegister/portInputRegister/portModeRegister 
  #define ioRegister_t uint32_t
  #define _delay_ms(ms) delayMicroseconds((ms) * 1000UL)
  const uint8_t constMcuIdSize                                    = 0x04;
#elif defined(ARDUINO_ARCH_ESP32)
  // variable type used with portOutputRegister/portInputRegister/portModeRegister 
  #define ioRegister_t uint32_t
  #define _delay_ms(ms) delayMicroseconds((ms) * 1000UL)
  const uint8_t constMcuIdSize                                    = 0x06;
#endif //#if defined(ARDUINO_ARCH_AVR)

/*****************************************************************************************************************************
*
*  Reset the system
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void systemReboot();

/*****************************************************************************************************************************
*
*  Read bytes from the MCU's Signature Row and put its to array
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void getMcuId(uint8_t* _dst);
void getMcuModel(uint8_t* _dst);
int32_t getMcuFreq();
int32_t getMcuVoltage();
int8_t getSystemAllInfo(char*, const uint16_t);

/*****************************************************************************************************************************
*
*  Init Timer1 
*
*   Returns: 
*     - always true at this time
*
*****************************************************************************************************************************/
uint8_t initTimerOne(const uint32_t);

/*****************************************************************************************************************************
*
*  Start Timer1
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void startTimerOne(void);


/*****************************************************************************************************************************
*
*  Handle Timer1 interrupt 
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
//ISR(TIMER1_COMPA_vect);

/*****************************************************************************************************************************
*
*  Stop Timer1
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void stopTimerOne(void);

/*****************************************************************************************************************************
*
*  Gather internal metrics and save it to global variable
*
*   Returns: 
*     - none
*
*****************************************************************************************************************************/
void gatherSystemMetrics(void);

/* ****************************************************************************************************************************
*
*  Return "Free" memory size
*
**************************************************************************************************************************** */
//inline __attribute__((always_inline)) uint32_t getRamFree(void) {
inline uint32_t getRamFree(void) {
  uint32_t result = 0x00;
#if defined(ARDUINO_ARCH_AVR)
  extern uint16_t __heap_start, *__brkval;
  uint16_t v;
  result = (uint32_t) (&v - (__brkval == 0 ? (uint32_t) &__heap_start : (uint32_t) __brkval));
#elif (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))
  result = ESP.getFreeHeap();
#endif
  return result;
}

/* ****************************************************************************************************************************
*
*   Atomic reading functions for ESP boards
*
**************************************************************************************************************************** */
#if defined(ARDUINO_ARCH_ESP8266)

#ifndef __STRINGIFY
    #define __STRINGIFY(a) #a
#endif

#ifndef xt_rsil
    #define xt_rsil(level) (__extension__({uint32_t state; __asm__ __volatile__("rsil %0," __STRINGIFY(level) : "=a" (state)); state;}))
#endif

#ifndef xt_wsr_ps
    #define xt_wsr_ps(state)  __asm__ __volatile__("wsr %0,ps; isync" :: "a" (state) : "memory")
#endif

static __inline__ void SA_iRestore(const  uint32_t *__s) { xt_wsr_ps(*__s); }

// Note value can be 0-15, 0 = Enable all interrupts, 15 = no interrupts
#define SA_ATOMIC_RESTORESTATE uint32_t _sa_saved __attribute__((__cleanup__(SA_iRestore))) = xt_rsil(15)

#endif // #if defined(ARDUINO_ARCH_ESP8266)

#if defined(ARDUINO_ARCH_ESP32)

static __inline__ void SA_iRestore(const  uint32_t *__s) {
    XTOS_RESTORE_INTLEVEL(*__s);
}

// Note value can be 0-15, 0 = Enable all interrupts, 15 = no interrupts
#define SA_ATOMIC_RESTORESTATE uint32_t _sa_saved __attribute__((__cleanup__(SA_iRestore))) = XTOS_DISABLE_LOWPRI_INTERRUPTS

#endif // #if defined(ARDUINO_ARCH_ESP32)


/*************** MACRO **********************/

#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
  #define ATOMIC() for ( SA_ATOMIC_RESTORESTATE, _sa_done =  1; _sa_done; _sa_done = 0 )
#endif // defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
 