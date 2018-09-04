#pragma once
#include <stdint.h>

void __attribute__((weak)) initStageUserFunction(char*);
void __attribute__((weak)) netPrepareStageUserFunction(char*);
void __attribute__((weak)) alarmStageUserFunction(char*, uint8_t);
void __attribute__((weak)) loopStageUserFunction(char*);

int8_t __attribute__((weak)) executeCommandUserFunction(char*, char**, int32_t*, int32_t*);
