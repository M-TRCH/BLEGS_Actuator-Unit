#ifndef EEPROM_UTILS_H
#define EEPROM_UTILS_H

#include <Arduino.h>
#include <EEPROM.h>
#include "system.h"

// EEPROM memory addresses
#define ADDR_ROTOR_OFFSET_CW      0
#define ADDR_ROTOR_OFFSET_CCW     4
#define ADDR_ROTOR_OFFSET_ABS     8

void saveFloatToEEPROM(int addr, float value);
float readFloatFromEEPROM(int addr);
void saveMotorDataToEEPROM(float cwOffset, float ccwOffset, float absOffset);
void loadMotorDataFromEEPROM(float &cwOffset, float &ccwOffset, float &absOffset, bool debug = false);

#endif // EEPROM_UTILS_H
