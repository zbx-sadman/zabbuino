#pragma once
#include <stdint.h>

void initStageUserFunction(char*) __attribute__((weak));
void netPrepareStageUserFunction(char*) __attribute__((weak));
int8_t executeCommandUserFunction(char*, char**, int32_t*) __attribute__((weak));
void alarmStageUserFunction(char*, uint8_t) __attribute__((weak));
void loopStageUserFunction(char*) __attribute__((weak));

