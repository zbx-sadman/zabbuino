#include <Arduino.h>
#include <stdint.h>
//#include "plugin.h"
#include "sys_includes.h"

/*****************************************************************************************************************************

   Subroutine calls on start of Zabbuino

*****************************************************************************************************************************/
extern "C" {
  void initStageUserFunction(uint8_t*) __attribute__ ((weak, alias("__initStageUserFunction")));
  static void __initStageUserFunction(uint8_t*) {}
}

//void __attribute__((weak)) initStageUserFunction(uint8_t*){;}


/*****************************************************************************************************************************

   Subroutine calls when config is already loaded, but network not started yet

*****************************************************************************************************************************/
extern "C" {
  void netPrepareStageUserFunction(uint8_t*) __attribute__ ((weak, alias("__netPrepareStageUserFunction")));
  static void __netPrepareStageUserFunction(uint8_t*) {}
}
  //void __attribute__((weak)) netPrepareStageUserFunction(uint8_t*){;}
/*****************************************************************************************************************************

   Subroutine calls before processing loop entering

*****************************************************************************************************************************/
extern "C" {
  void preLoopStageUserFunction(uint8_t*) __attribute__ ((weak, alias("__preLoopStageUserFunction")));
  static void __preLoopStageUserFunction(uint8_t*) {}
}

//void __attribute__((weak)) preLoopStageUserFunction(uint8_t*){;}

/*****************************************************************************************************************************

   Subroutine calls if alarm occured

*****************************************************************************************************************************/
extern "C" {
  void alarmStageUserFunction(uint8_t*, uint8_t) __attribute__ ((weak, alias("__alarmStageUserFunction")));
  static void __alarmStageUserFunction(uint8_t*, uint8_t) {}
}
//void __attribute__((weak)) alarmStageUserFunction(uint8_t*, uint8_t){;}

/*****************************************************************************************************************************

   Subroutine calls on every loop if no active network session exist

*****************************************************************************************************************************/
extern "C" {
  void loopStageUserFunction(uint8_t*) __attribute__ ((weak, alias("__loopStageUserFunction")));
  static void __loopStageUserFunction(uint8_t*) {}
}
//void __attribute__((weak)) loopStageUserFunction(uint8_t*){;}

/*****************************************************************************************************************************

   Subroutine calls when user.run[] command catched

*****************************************************************************************************************************/
extern "C" {
  void executeCommandUserFunction(uint8_t*, char**, int32_t*, int32_t*) __attribute__ ((weak, alias("__executeCommandUserFunction")));
  static int8_t __executeCommandUserFunction(uint8_t*, char**, int32_t*, int32_t*);
}

static int8_t __executeCommandUserFunction(uint8_t*, char**, int32_t*, int32_t*) {
  return RESULT_IS_OK; 
}


/*
int8_t __attribute__((weak)) executeCommandUserFunction(uint8_t*, char**, int32_t*, int32_t*){
  return RESULT_IS_OK; 
}

*/