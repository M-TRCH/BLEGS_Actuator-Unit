#ifndef EEPROM_UTILS_H
#define EEPROM_UTILS_H

#include <Arduino.h>
#include <EEPROM.h>
#include "system.h"

// EEPROM Configuration Structure (Version 1.0 - Basic Motor Data)
typedef struct {
    // Header for validation and versioning
    uint32_t magic_number;              // Magic number for validation (0xBEEF1234)
    uint16_t version;                   // Structure version (0x0100 for v1.0)
    uint16_t data_size;                 // Size of this structure
    
    // Motor calibration data (primary data for v1.0)
    float rotor_offset_cw;              // Rotor offset clockwise (degrees)
    float rotor_offset_ccw;             // Rotor offset counter-clockwise (degrees)
    float rotor_offset_absolute;        // Absolute rotor offset (degrees)
    
    // Status flags
    bool is_calibrated;                 // Motor calibration status
    bool is_first_boot;                 // First boot flag
    
    // Reserved space for future expansion (56 bytes)
    uint8_t reserved[56];               // Reserved for future features
    
    // Data integrity
    uint32_t checksum;                  // CRC32 checksum for validation
} EepromMotorConfig_t;

// Global EEPROM configuration instance
extern EepromMotorConfig_t eepromMotorConfig;

// EEPROM Constants
#define EEPROM_MAGIC_NUMBER         0xBEEF1234
#define EEPROM_VERSION              0x0100          // Version 1.0
#define EEPROM_CONFIG_ADDRESS       0               // Start address
#define EEPROM_CONFIG_SIZE          sizeof(EepromMotorConfig_t)

// Modern EEPROM Functions
void initEEPROM();
bool loadEEPROMConfig();
bool saveEEPROMConfig();
void resetEEPROMToDefaults();
bool validateEEPROMData();
uint32_t calculateEEPROMChecksum(EepromMotorConfig_t* config);
void printEEPROMConfig();

// Convenient access macros
#define GET_ROTOR_OFFSET_CW()       (eepromMotorConfig.rotor_offset_cw)
#define GET_ROTOR_OFFSET_CCW()      (eepromMotorConfig.rotor_offset_ccw)
#define GET_ROTOR_OFFSET_ABS()      (eepromMotorConfig.rotor_offset_absolute)
#define SET_ROTOR_OFFSET_CW(val)    (eepromMotorConfig.rotor_offset_cw = (val))
#define SET_ROTOR_OFFSET_CCW(val)   (eepromMotorConfig.rotor_offset_ccw = (val))
#define SET_ROTOR_OFFSET_ABS(val)   (eepromMotorConfig.rotor_offset_absolute = (val))
#define IS_MOTOR_CALIBRATED()       (eepromMotorConfig.is_calibrated)
#define SET_MOTOR_CALIBRATED(val)   (eepromMotorConfig.is_calibrated = (val))

// Legacy compatibility functions (kept for backward compatibility)
void saveMotorDataToEEPROM(float cwOffset, float ccwOffset, float absOffset);
void loadMotorDataFromEEPROM(float &cwOffset, float &ccwOffset, float &absOffset, bool debug = false);

#endif // EEPROM_UTILS_H
