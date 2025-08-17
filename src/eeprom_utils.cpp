
#include "eeprom_utils.h"

void saveFloatToEEPROM(int addr, float value) 
{
    EEPROM.put(addr, value);
}

float readFloatFromEEPROM(int addr) 
{
    float value;
    EEPROM.get(addr, value);
    return value;
}
