
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

/**
 * @brief Saves the motor data (offsets) to EEPROM
 * @param cwOffset The clockwise offset
 * @param ccwOffset The counter-clockwise offset
 * @param absOffset The absolute offset
 */
void saveMotorDataToEEPROM(float cwOffset, float ccwOffset, float absOffset) 
{
    saveFloatToEEPROM(ADDR_ROTOR_OFFSET_CW, cwOffset);
    saveFloatToEEPROM(ADDR_ROTOR_OFFSET_CCW, ccwOffset);
    saveFloatToEEPROM(ADDR_ROTOR_OFFSET_ABS, absOffset);
}

/**
 * @brief Loads the motor data (offsets) from EEPROM
 * @param cwOffset The clockwise offset
 * @param ccwOffset The counter-clockwise offset
 * @param absOffset The absolute offset
 */
void loadMotorDataFromEEPROM(float &cwOffset, float &ccwOffset, float &absOffset, bool debug) 
{
    cwOffset = readFloatFromEEPROM(ADDR_ROTOR_OFFSET_CW);
    ccwOffset = readFloatFromEEPROM(ADDR_ROTOR_OFFSET_CCW);
    absOffset = readFloatFromEEPROM(ADDR_ROTOR_OFFSET_ABS);
    if (debug)
    {
        Serial3.print(F("CW Offset: "));
        Serial3.println(cwOffset, SERIAL3_DECIMAL_PLACES);
        Serial3.print(F("CCW Offset: "));
        Serial3.println(ccwOffset, SERIAL3_DECIMAL_PLACES);
        Serial3.print(F("Absolute Offset: "));
        Serial3.println(absOffset, SERIAL3_DECIMAL_PLACES);
    }
}
