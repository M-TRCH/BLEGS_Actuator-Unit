
#include "encoder.h"

AS5047P rotor(CS_ENC_PIN, SPI_SPEED);
uint16_t raw_rotor_angle = 0;               // Global variable to hold the raw rotor angle
// Rotor offsets
float const_rotor_offset_cw = 0.0f;         // Constant offset for clockwise rotation
float const_rotor_offset_ccw = 0.0f;        // Constant offset for counter-clockwise rotation
float rotor_offset_cw = 0.0f;               // Offset for clockwise rotation
float rotor_offset_ccw = 0.0f;              // Offset for counter-clockwise rotation
// Absolute rotor angle
int rotor_turns = 0;
float last_raw_angle_deg = 0.0f;
// Absolute rotor angle offset
float rotor_offset_abs = 0.0f;

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
    // Low-pass filter to reduce motor noise (Exponential Moving Average)
    static float filtered_angle = 0.0f;
    static bool filter_initialized = false;
    const float FILTER_ALPHA = 1.0f;    // No filtering (set to 1.0f); adjust between 0.0f (max filtering) and 1.0f (no filtering)

    // Read raw encoder value
    uint16_t raw_reading;
    #if RAW_ROTOR_ANGLE_INVERT == true
        raw_reading = _14_BIT - rotor.readAngleRaw(); 
    #else
        raw_reading = rotor.readAngleRaw(); 
    #endif
    
    // Initialize filter on first run
    if (!filter_initialized)
    {
        filtered_angle = (float)raw_reading;
        filter_initialized = true;
    }
    
    // Apply exponential moving average (EMA) low-pass filter
    filtered_angle = FILTER_ALPHA * (float)raw_reading + (1.0f - FILTER_ALPHA) * filtered_angle;
    
    // Convert back to uint16_t with rounding
    raw_rotor_angle = (uint16_t)(filtered_angle + 0.5f);
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
    
    // Optimized wrap detection with hysteresis and validation
    // Hysteresis threshold: ±(180° - 10°) = ±170° to avoid false triggers
    // This prevents noise near zero-crossing from causing incorrect turn counting
    const float WRAP_THRESHOLD = 170.0f;
    const float MAX_DELTA = 45.0f;  // Maximum reasonable angular change per update (~25kHz SVPWM = 40µs)
    
    // Validate delta is within reasonable bounds (prevents spurious readings)
    if (fabsf(delta) < MAX_DELTA)
    {
        // Normal case - no wrap, keep turn count as is
        last_raw_angle_deg = raw_angle_deg;
        return;
    }
    
    // Wrap detection with hysteresis
    if (delta > WRAP_THRESHOLD)
    {
        // Forward wrap: 359° -> 0° (CCW rotation, turning backward in raw counts)
        rotor_turns--;
    }
    else if (delta < -WRAP_THRESHOLD)
    {
        // Backward wrap: 0° -> 359° (CW rotation, turning forward in raw counts)
        rotor_turns++;
    }
    // If delta is between ±45° and ±170°, it's likely a spurious reading - ignore
    
    // Update the last raw angle
    last_raw_angle_deg = raw_angle_deg;
}

float readRotorAbsoluteAngle(bool with_offset)
{
    float angle_deg = raw_rotor_angle * RAW_TO_DEGREE; 
    float absolute_pos = rotor_turns * 360.0f + angle_deg;
    if (with_offset) absolute_pos -= rotor_offset_abs;
    return absolute_pos; // return absolute position in degrees
}

/*  @brief Computes     the offset angle for inverse kinematics
 *  @param angle_ik     The angle from inverse kinematics
 *  @param angle_mea    The measured angle
 */
float computeOffsetAngleIK(float angle_ik, float angle_mea)
{
    angle_ik *= GEAR_RATIO;         // Convert IK angle to absolute angle
    return angle_mea - angle_ik;    // Compute the offset angle
}