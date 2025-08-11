
#include "encoder.h"

AS5047P rotor(CS_ENC_PIN, SPI_SPEED);
uint16_t raw_rotor_angle = 0;               // Global variable to hold the raw rotor angle
float const_rotor_offset_cw = 848.0f;       // Constant offset for clockwise rotation
float const_rotor_offset_ccw = 693.0f;      // Constant offset for counter-clockwise rotation
float rotor_offset_cw = 0.0f;               // Offset for clockwise rotation
float rotor_offset_ccw = 0.0f;              // Offset for counter-clockwise rotation

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
    raw_rotor_angle = _14_BIT - rotor.readAngleRaw(); 
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