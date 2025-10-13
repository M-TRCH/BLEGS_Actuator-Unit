# EEPROM System Documentation v1.0

## Overview
ระบบ EEPROM ใหม่ใช้โครงสร้างข้อมูลแบบ structured data พร้อม validation และ checksum เพื่อความน่าเชื่อถือ

## Structure Layout

```cpp
typedef struct {
    // Header (12 bytes)
    uint32_t magic_number;              // 0xBEEF1234 (validation)
    uint16_t version;                   // 0x0100 (v1.0)
    uint16_t data_size;                 // Structure size
    
    // Motor Data (12 bytes)
    float rotor_offset_cw;              // CW offset (degrees)
    float rotor_offset_ccw;             // CCW offset (degrees) 
    float rotor_offset_absolute;        // Absolute offset (degrees)
    
    // Status (2 bytes)
    bool is_calibrated;                 // Calibration flag
    bool is_first_boot;                 // First boot flag
    
    // Future Expansion (56 bytes)
    uint8_t reserved[56];               // Reserved for future features
    
    // Validation (4 bytes)
    uint32_t checksum;                  // CRC32 checksum
} EepromMotorConfig_t;  // Total: 86 bytes
```

## Key Features

### ✅ **Data Integrity**
- Magic number validation (`0xBEEF1234`)
- Version checking (`0x0100`)
- CRC32 checksum validation
- Structure size validation

### ✅ **Future Expandable**
- 56 bytes reserved space
- Version management for upgrades
- Backward compatibility support

### ✅ **Easy Access**
- Convenient macros for common operations
- Modern structured functions
- Legacy function compatibility

## Usage Guide

### Initialization (Required)
```cpp
void setup() {
    systemInit(SERIAL_SYSTEM);
    initEEPROM();  // Must call this first
}
```

### Reading Data
```cpp
// Modern way (recommended)
float cw_offset = GET_ROTOR_OFFSET_CW();
float ccw_offset = GET_ROTOR_OFFSET_CCW();
float abs_offset = GET_ROTOR_OFFSET_ABS();
bool is_calibrated = IS_MOTOR_CALIBRATED();

// Legacy way (still works)
float cw, ccw, abs;
loadMotorDataFromEEPROM(cw, ccw, abs, true);
```

### Writing Data
```cpp
// Modern way (recommended)
SET_ROTOR_OFFSET_CW(1082.0f);
SET_ROTOR_OFFSET_CCW(924.0f);
SET_ROTOR_OFFSET_ABS(1286.72f);
SET_MOTOR_CALIBRATED(true);
saveEEPROMConfig();  // Save all changes

// Legacy way (still works)
saveMotorDataToEEPROM(1082.0f, 924.0f, 1286.72f);
```

### Data Validation
```cpp
// Check if EEPROM data is valid
if (validateEEPROMData()) {
    SystemSerial->println("EEPROM data is valid");
} else {
    SystemSerial->println("EEPROM data corrupted!");
    resetEEPROMToDefaults();
}
```

### Reset to Defaults
```cpp
resetEEPROMToDefaults();  // Resets all values to defaults
```

### Debug Information
```cpp
printEEPROMConfig();  // Print complete configuration
```

## Function Reference

### Modern Functions (Recommended)
| Function | Description |
|----------|-------------|
| `initEEPROM()` | Initialize EEPROM system |
| `saveEEPROMConfig()` | Save current config to EEPROM |
| `loadEEPROMConfig()` | Load config from EEPROM |
| `resetEEPROMToDefaults()` | Reset to default values |
| `validateEEPROMData()` | Validate data integrity |
| `printEEPROMConfig()` | Print current configuration |

### Convenient Macros
| Macro | Description |
|-------|-------------|
| `GET_ROTOR_OFFSET_CW()` | Get CW offset |
| `SET_ROTOR_OFFSET_CW(val)` | Set CW offset |
| `GET_ROTOR_OFFSET_CCW()` | Get CCW offset |
| `SET_ROTOR_OFFSET_CCW(val)` | Set CCW offset |
| `GET_ROTOR_OFFSET_ABS()` | Get absolute offset |
| `SET_ROTOR_OFFSET_ABS(val)` | Set absolute offset |
| `IS_MOTOR_CALIBRATED()` | Check calibration status |
| `SET_MOTOR_CALIBRATED(val)` | Set calibration status |

### Legacy Functions (Backward Compatibility)
| Function | Status | Recommendation |
|----------|--------|----------------|
| `saveFloatToEEPROM()` | Deprecated | Use structured functions |
| `readFloatFromEEPROM()` | Deprecated | Use structured functions |
| `saveMotorDataToEEPROM()` | Compatible | Works but use modern way |
| `loadMotorDataFromEEPROM()` | Compatible | Works but use modern way |

## Migration Guide

### From Legacy System
```cpp
// Old code
saveFloatToEEPROM(0, 1082.0f);
saveFloatToEEPROM(4, 924.0f);
saveFloatToEEPROM(8, 1286.72f);

// New code (recommended)
SET_ROTOR_OFFSET_CW(1082.0f);
SET_ROTOR_OFFSET_CCW(924.0f);
SET_ROTOR_OFFSET_ABS(1286.72f);
SET_MOTOR_CALIBRATED(true);
saveEEPROMConfig();
```

## Memory Layout

```
Address 0x00: Magic Number (0xBEEF1234)
Address 0x04: Version (0x0100)
Address 0x06: Data Size
Address 0x08: CW Offset (float)
Address 0x0C: CCW Offset (float)
Address 0x10: Absolute Offset (float)
Address 0x14: Calibrated Flag (bool)
Address 0x15: First Boot Flag (bool)
Address 0x16: Reserved[56] (future use)
Address 0x4E: Checksum (CRC32)
```

## Future Expansion Plan

สำหรับ Version 2.0 สามารถขยายได้ใน reserved space:
- PID parameters
- Motor specifications
- System settings
- LED configurations
- Safety limits
- User preferences

## Error Handling

### Common Error Scenarios
1. **First Boot**: No valid data → Use defaults
2. **Version Mismatch**: Different version → Handle upgrade
3. **Checksum Error**: Corrupted data → Reset to defaults
4. **Magic Number Error**: Invalid data → Reset to defaults

### Error Recovery
```cpp
if (!validateEEPROMData()) {
    SystemSerial->println("EEPROM corrupted, resetting...");
    resetEEPROMToDefaults();
}
```

## Best Practices

1. **Always call `initEEPROM()`** first in setup()
2. **Use macros** for common operations
3. **Save after changes** with `saveEEPROMConfig()`
4. **Validate data** periodically
5. **Handle errors** gracefully
6. **Use structured approach** instead of legacy functions