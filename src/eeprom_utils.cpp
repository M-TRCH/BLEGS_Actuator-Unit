
#include "eeprom_utils.h"

// Global EEPROM configuration instances
EepromMotorConfig_t eepromMotorConfig;
EepromMotorConfig_t eepromMotorConfig_default;

/**
 * @brief Setup default configuration values
 */
void setupDefaultConfig() 
{
    // Clear structure
    memset(&eepromMotorConfig_default, 0, sizeof(EepromMotorConfig_t));
    
    // Header information
    eepromMotorConfig_default.magic_number = EEPROM_MAGIC_NUMBER;
    eepromMotorConfig_default.version = EEPROM_VERSION;
    eepromMotorConfig_default.data_size = sizeof(EepromMotorConfig_t);
    
    // Default motor calibration values
    eepromMotorConfig_default.rotor_offset_cw = 0.0f;
    eepromMotorConfig_default.rotor_offset_ccw = 0.0f;
    eepromMotorConfig_default.rotor_offset_absolute = 0.0f;
    
    // Default status flags
    eepromMotorConfig_default.is_calibrated = false;
    eepromMotorConfig_default.is_first_boot = true;
    
    // Calculate checksum
    eepromMotorConfig_default.checksum = calculateEEPROMChecksum(&eepromMotorConfig_default);
}

/**
 * @brief Initialize EEPROM system and load configuration
 */
void initEEPROM() 
{
    SystemSerial->println("=== Initializing EEPROM ===");
    
    // Setup default configuration
    setupDefaultConfig();
    
    // Try to load existing configuration
    if (!loadEEPROMConfig()) {
        SystemSerial->println("No valid EEPROM data found. Using defaults.");
        
        // Copy default to current config
        memcpy(&eepromMotorConfig, &eepromMotorConfig_default, sizeof(EepromMotorConfig_t));
        
        // Save default configuration
        if (saveEEPROMConfig()) {
            SystemSerial->println("Default configuration saved to EEPROM");
        } else {
            SystemSerial->println("ERROR: Failed to save default configuration");
        }
    } else {
        SystemSerial->println("Configuration loaded from EEPROM");
    }
    
    // Print current configuration
    printEEPROMConfig();
}

/**
 * @brief Load configuration from EEPROM
 * @return true if successful, false if data is invalid
 */
bool loadEEPROMConfig() 
{
    EepromMotorConfig_t tempConfig;
    
    // Read configuration from EEPROM
    EEPROM.get(EEPROM_CONFIG_ADDRESS, tempConfig);
    
    // Validate magic number
    if (tempConfig.magic_number != EEPROM_MAGIC_NUMBER) {
        SystemSerial->println("Invalid magic number in EEPROM");
        return false;
    }
    
    // Validate version
    if (tempConfig.version != EEPROM_VERSION) {
        SystemSerial->print("EEPROM version mismatch. Found: 0x");
        SystemSerial->print(tempConfig.version, HEX);
        SystemSerial->print(", Expected: 0x");
        SystemSerial->println(EEPROM_VERSION, HEX);
        return false;
    }
    
    // Validate checksum
    uint32_t stored_checksum = tempConfig.checksum;
    tempConfig.checksum = 0;  // Clear for calculation
    uint32_t calculated_checksum = calculateEEPROMChecksum(&tempConfig);
    
    if (stored_checksum != calculated_checksum) {
        SystemSerial->println("EEPROM checksum validation failed");
        return false;
    }
    
    // Restore checksum and copy to current config
    tempConfig.checksum = stored_checksum;
    memcpy(&eepromMotorConfig, &tempConfig, sizeof(EepromMotorConfig_t));
    
    return true;
}

/**
 * @brief Save current configuration to EEPROM
 * @return true if successful, false if failed
 */
bool saveEEPROMConfig() 
{
    // Update header information
    eepromMotorConfig.magic_number = EEPROM_MAGIC_NUMBER;
    eepromMotorConfig.version = EEPROM_VERSION;
    eepromMotorConfig.data_size = sizeof(EepromMotorConfig_t);
    
    // Calculate and set checksum
    eepromMotorConfig.checksum = 0;  // Clear old checksum
    eepromMotorConfig.checksum = calculateEEPROMChecksum(&eepromMotorConfig);
    
    // Write to EEPROM
    EEPROM.put(EEPROM_CONFIG_ADDRESS, eepromMotorConfig);
    
    // Verify by reading back
    EepromMotorConfig_t verifyConfig;
    EEPROM.get(EEPROM_CONFIG_ADDRESS, verifyConfig);
    
    // Compare the data
    return (memcmp(&eepromMotorConfig, &verifyConfig, sizeof(EepromMotorConfig_t)) == 0);
}

/**
 * @brief Reset EEPROM to default values
 */
void resetEEPROMToDefaults() 
{
    SystemSerial->println("Resetting EEPROM to default values...");
    
    // Copy default configuration
    memcpy(&eepromMotorConfig, &eepromMotorConfig_default, sizeof(EepromMotorConfig_t));
    
    // Save to EEPROM
    if (saveEEPROMConfig()) {
        SystemSerial->println("EEPROM reset successful");
    } else {
        SystemSerial->println("ERROR: EEPROM reset failed");
    }
}

/**
 * @brief Validate EEPROM data integrity
 * @return true if data is valid, false otherwise
 */
bool validateEEPROMData() 
{
    // Check magic number
    if (eepromMotorConfig.magic_number != EEPROM_MAGIC_NUMBER) {
        return false;
    }
    
    // Check version
    if (eepromMotorConfig.version != EEPROM_VERSION) {
        return false;
    }
    
    // Check checksum
    uint32_t stored_checksum = eepromMotorConfig.checksum;
    eepromMotorConfig.checksum = 0;
    uint32_t calculated_checksum = calculateEEPROMChecksum(&eepromMotorConfig);
    eepromMotorConfig.checksum = stored_checksum;
    
    return (stored_checksum == calculated_checksum);
}

/**
 * @brief Calculate CRC32 checksum for EEPROM data
 * @param config Pointer to configuration structure
 * @return Calculated checksum
 */
uint32_t calculateEEPROMChecksum(EepromMotorConfig_t* config) 
{
    uint32_t crc = 0xFFFFFFFF;
    uint8_t* data = (uint8_t*)config;
    size_t length = sizeof(EepromMotorConfig_t) - sizeof(uint32_t); // Exclude checksum field
    
    // CRC32 calculation
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return ~crc;
}

/**
 * @brief Print current EEPROM configuration
 */
void printEEPROMConfig() 
{
    SystemSerial->println("=== EEPROM Configuration ===");
    SystemSerial->print("Magic Number: 0x");
    SystemSerial->println(eepromMotorConfig.magic_number, HEX);
    SystemSerial->print("Version: 0x");
    SystemSerial->println(eepromMotorConfig.version, HEX);
    SystemSerial->print("Data Size: ");
    SystemSerial->println(eepromMotorConfig.data_size);
    
    SystemSerial->println("--- Motor Offsets ---");
    SystemSerial->print("CW Offset: ");
    SystemSerial->println(eepromMotorConfig.rotor_offset_cw, 4);
    SystemSerial->print("CCW Offset: ");
    SystemSerial->println(eepromMotorConfig.rotor_offset_ccw, 4);
    SystemSerial->print("Absolute Offset: ");
    SystemSerial->println(eepromMotorConfig.rotor_offset_absolute, 4);
    
    SystemSerial->println("--- Status ---");
    SystemSerial->print("Calibrated: ");
    SystemSerial->println(eepromMotorConfig.is_calibrated ? "Yes" : "No");
    SystemSerial->print("First Boot: ");
    SystemSerial->println(eepromMotorConfig.is_first_boot ? "Yes" : "No");
    
    SystemSerial->print("Checksum: 0x");
    SystemSerial->println(eepromMotorConfig.checksum, HEX);
    SystemSerial->println("===========================");
}

// Legacy compatibility functions (backward compatibility)

/**
 * @brief Save motor calibration data (legacy compatibility)
 */
void saveMotorDataToEEPROM(float cwOffset, float ccwOffset, float absOffset, bool active) 
{
    if (active)
    {
    
    
        // Update current configuration
        SET_ROTOR_OFFSET_CW(cwOffset);
        SET_ROTOR_OFFSET_CCW(ccwOffset);
        SET_ROTOR_OFFSET_ABS(absOffset);
        SET_MOTOR_CALIBRATED(true);
        
        // Save configuration
        if (saveEEPROMConfig()) {
            SystemSerial->println("Motor data saved to EEPROM");
        } else {
            SystemSerial->println("ERROR: Failed to save motor data");
        }
        while (1); // Halt execution for safety
    }
}

/**
 * @brief Load motor calibration data (legacy compatibility)
 */
void loadMotorDataFromEEPROM(float &cwOffset, float &ccwOffset, float &absOffset, bool debug) 
{  
    // Get values from current configuration
    cwOffset = GET_ROTOR_OFFSET_CW();
    ccwOffset = GET_ROTOR_OFFSET_CCW();
    absOffset = GET_ROTOR_OFFSET_ABS();
    
    if (debug) {
        SystemSerial->print("CW Offset: ");
        SystemSerial->println(cwOffset, SERIAL1_DECIMAL_PLACES);
        SystemSerial->print("CCW Offset: ");
        SystemSerial->println(ccwOffset, SERIAL1_DECIMAL_PLACES);
        SystemSerial->print("Absolute Offset: ");
        SystemSerial->println(absOffset, SERIAL1_DECIMAL_PLACES);
    }
}
