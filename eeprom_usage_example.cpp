/**
 * @file eeprom_usage_example.cpp
 * @brief Example usage of new structured EEPROM system
 * 
 * This example shows how to use the new EEPROM system with structured data
 * while maintaining backward compatibility with legacy functions.
 */

#include "eeprom_utils.h"

void demonstrateNewEEPROM() 
{
    SystemSerial->println("=== New EEPROM System Demonstration ===");
    
    // Initialize EEPROM system (call this once in setup())
    initEEPROM();
    
    // Example 1: Check if motor is already calibrated
    if (IS_MOTOR_CALIBRATED()) {
        SystemSerial->println("Motor is already calibrated:");
        SystemSerial->print("  CW Offset: ");
        SystemSerial->println(GET_ROTOR_OFFSET_CW(), 4);
        SystemSerial->print("  CCW Offset: ");
        SystemSerial->println(GET_ROTOR_OFFSET_CCW(), 4);
        SystemSerial->print("  Absolute Offset: ");
        SystemSerial->println(GET_ROTOR_OFFSET_ABS(), 4);
    } else {
        SystemSerial->println("Motor needs calibration");
    }
    
    // Example 2: Set new calibration values (modern way)
    SystemSerial->println("\nSetting new calibration values...");
    SET_ROTOR_OFFSET_CW(1082.0f);
    SET_ROTOR_OFFSET_CCW(924.0f);
    SET_ROTOR_OFFSET_ABS(1286.72f);
    SET_MOTOR_CALIBRATED(true);
    
    // Save the configuration
    if (saveEEPROMConfig()) {
        SystemSerial->println("Configuration saved successfully!");
    } else {
        SystemSerial->println("ERROR: Failed to save configuration");
    }
    
    // Example 3: Read values back to verify
    SystemSerial->println("\nVerification:");
    SystemSerial->print("CW Offset: ");
    SystemSerial->println(GET_ROTOR_OFFSET_CW(), 4);
    SystemSerial->print("CCW Offset: ");
    SystemSerial->println(GET_ROTOR_OFFSET_CCW(), 4);
    SystemSerial->print("Absolute Offset: ");
    SystemSerial->println(GET_ROTOR_OFFSET_ABS(), 4);
    SystemSerial->print("Calibration Status: ");
    SystemSerial->println(IS_MOTOR_CALIBRATED() ? "Calibrated" : "Not Calibrated");
    
    // Example 4: Reset to defaults if needed
    // resetEEPROMToDefaults();
    
    // Example 5: Validate data integrity
    if (validateEEPROMData()) {
        SystemSerial->println("EEPROM data is valid");
    } else {
        SystemSerial->println("WARNING: EEPROM data is corrupted!");
    }
    
    // Example 6: Print complete configuration
    printEEPROMConfig();
}

void demonstrateLegacyCompatibility() 
{
    SystemSerial->println("=== Legacy Compatibility Demonstration ===");
    
    // Old way still works (but shows deprecation warnings)
    float cw_offset, ccw_offset, abs_offset;
    
    // Load using legacy function
    loadMotorDataFromEEPROM(cw_offset, ccw_offset, abs_offset, true);
    
    // Modify values
    cw_offset += 10.0f;
    ccw_offset += 5.0f;
    abs_offset += 15.0f;
    
    // Save using legacy function
    saveMotorDataToEEPROM(cw_offset, ccw_offset, abs_offset);
    
    SystemSerial->println("Legacy functions still work!");
}

/**
 * @brief Example of proper initialization in main.cpp setup()
 */
void exampleSetupIntegration() 
{
    // In your main.cpp setup() function, add:
    
    /*
    void setup() 
    {
        // Initialize system
        systemInit(SERIAL_SYSTEM);
        
        // Initialize EEPROM system
        initEEPROM();
        
        // Check calibration status
        if (!IS_MOTOR_CALIBRATED()) {
            SystemSerial->println("Motor requires calibration");
            // Perform calibration process here...
            // SET_ROTOR_OFFSET_CW(calibrated_value);
            // SET_MOTOR_CALIBRATED(true);
            // saveEEPROMConfig();
        }
        
        // Continue with rest of setup...
    }
    */
}