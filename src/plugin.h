#pragma once
#include <stdint.h>

/*****************************************************************************************************************************

   Subroutine calls on start of Zabbuino

*****************************************************************************************************************************/
extern "C" void initStageUserFunction(uint8_t*);
//void __attribute__((weak)) initStageUserFunction(uint8_t*);

/*****************************************************************************************************************************

   Subroutine calls when config is already loaded, but network not started yet

*****************************************************************************************************************************/
extern "C" void netPrepareStageUserFunction(uint8_t*);
//void __attribute__((weak)) netPrepareStageUserFunction(uint8_t*);

/*****************************************************************************************************************************

   Subroutine calls before processing loop entering

*****************************************************************************************************************************/
extern "C" void preLoopStageUserFunction(uint8_t*);
//void __attribute__((weak)) preLoopStageUserFunction(uint8_t*);

/*****************************************************************************************************************************

   Subroutine calls if alarm occured

*****************************************************************************************************************************/
extern "C" void alarmStageUserFunction(uint8_t*, uint8_t);
//void __attribute__((weak)) alarmStageUserFunction(uint8_t*, uint8_t);

/*****************************************************************************************************************************

   Subroutine calls on every loop if no active network session exist

*****************************************************************************************************************************/
extern "C" void loopStageUserFunction(uint8_t*);
//void __attribute__((weak)) loopStageUserFunction(uint8_t*);

/*****************************************************************************************************************************

   Subroutine calls when user.run[] command catched

*****************************************************************************************************************************/
extern "C" int8_t executeCommandUserFunction(uint8_t*, char**, int32_t*, int32_t*);
//int8_t __attribute__((weak)) executeCommandUserFunction(uint8_t*, char**, int32_t*, int32_t*);
