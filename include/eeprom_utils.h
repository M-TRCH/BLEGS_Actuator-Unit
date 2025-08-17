
#ifndef EEPROM_UTILS_H
#define EEPROM_UTILS_H

#include <Arduino.h>
#include <EEPROM.h>

void saveFloatToEEPROM(int addr, float value);
float readFloatFromEEPROM(int addr);

#endif // EEPROM_UTILS_H
