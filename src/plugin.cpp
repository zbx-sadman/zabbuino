#include "plugin.h"
#include "sys_includes.h"
#include <stdint.h>
#include "Arduino.h"

void __attribute__((weak)) initStageUserFunction(char*){;}
void __attribute__((weak)) netPrepareStageUserFunction(char*){;}

void __attribute__((weak)) alarmStageUserFunction(char*, uint8_t){;}
void __attribute__((weak)) loopStageUserFunction(char*){;}
void __attribute__((weak)) preLoopStageUserFunction(char*){;}

int8_t __attribute__((weak)) executeCommandUserFunction(char* _dst, char** _optarg, int32_t* _argv, int32_t* value){
  return RESULT_IS_OK; 
}

