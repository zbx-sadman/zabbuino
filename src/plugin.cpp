#include "plugin.h"
#include "sys_includes.h"
#include <stdint.h>

void initStageUserFunction(char*){;}
void netPrepareStageUserFunction(char*){;}
int8_t executeCommandUserFunction(char*, char**, int32_t*){return RESULT_IS_OK; }

void alarmStageUserFunction(char*, uint8_t){;}
void loopStageUserFunction(char*){;}

