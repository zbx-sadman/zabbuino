#ifndef ZabbuinoEEPROM_h
#define ZabbuinoEEPROM_h

#include <Arduino.h>
#include <IPAddress.h>
#include "../zabbuino.h"
#include "service.h"
#include "defaults.h"


uint8_t saveConfigToEEPROM(const netconfig_t* _configStruct);
uint8_t loadConfigFromEEPROM(netconfig_t* _configStruct);

#endif