#ifndef ZabbuinoEEPROM_h
#define ZabbuinoEEPROM_h

#include <Arduino.h>
#include <IPAddress.h>
#include <EEPROM.h>
#include "defaults.h"
#include "../zabbuino.h"
#include "service.h"

uint8_t saveConfigToEEPROM(const netconfig_t* _configStruct);
uint8_t loadConfigFromEEPROM(netconfig_t* _configStruct);

#endif