
#include "encoder.h"

AS5047P rotor(CS_ENC_PIN, SPI_SPEED);
uint16_t raw_rotor_angle = 0;               // Global variable to hold the raw rotor angle
// Rotor offsets
float const_rotor_offset_cw = 848.0f;       // Constant offset for clockwise rotation
float const_rotor_offset_ccw = 693.0f;      // Constant offset for counter-clockwise rotation
float rotor_offset_cw = 0.0f;               // Offset for clockwise rotation
float rotor_offset_ccw = 0.0f;              // Offset for counter-clockwise rotation
// Rotor angle tracking
int rotor_turns = 0;
float last_raw_angle_deg = 0.0f;

void encoderInit() 
{
    SPI.setMISO(MISO_PIN);
    SPI.setMOSI(MOSI_PIN);
    SPI.setSCLK(SCK_PIN);
    while (!rotor.initSPI()) 
    {
        Serial3.println(F("Encoder initialization failed! Retrying..."));
        delay(5000);
    }
    Serial3.println(F("Encoder initialized successfully!"));
}

void updateRawRotorAngle()
{
    #if RAW_ROTOR_ANGLE_INVERT == true
        raw_rotor_angle = _14_BIT - rotor.readAngleRaw(); 
    #else
        raw_rotor_angle = rotor.readAngleRaw(); 
    #endif
}

float readRotorAngle(bool ccw)
{
    float rotor_angle_with_offset = raw_rotor_angle - (ccw ? rotor_offset_ccw : rotor_offset_cw);
    if (rotor_angle_with_offset < 0) 
    {   
        rotor_angle_with_offset += _14_BIT; // compensate for negative angles
    }
    return rotor_angle_with_offset * RAW_TO_ROTOR_ANGLE;    // compute rotor angle in degrees
}

void updateMultiTurnTracking()
{
    // Update the multi-turn tracking based on the raw rotor angle
    float raw_angle_deg = raw_rotor_angle * RAW_TO_DEGREE;
    float delta = raw_angle_deg - last_raw_angle_deg;
    // Wrap detection
    if (delta > 180.0f)         rotor_turns--;
    else if (delta < -180.0f)   rotor_turns++;
    // Update the last raw angle
    last_raw_angle_deg = raw_angle_deg;
}

float readRotorAbsoluteAngle()
{
    float angle_deg = raw_rotor_angle * RAW_TO_DEGREE;
    float absolute_pos = rotor_turns * 360.0f + angle_deg;
    return absolute_pos;
}
