#include <Arduino.h>
#include "system.h"
#include "svpwm.h"
#include "encoder.h"
#include "motor_control.h"
#include "eeprom_utils.h"
#include "led.h"
#include "scurve.h"

uint32_t last_find_time;
float abs_angle, abs_angle_with_offset, calibration_angle;

void setup() 
{
#ifdef SYSTEM_H
    systemInit(SERIAL_SYSTEM);   // Initialize the system with RS232 pins
#endif

#ifdef LED_H
    initLED();                      // Initialize LED system
    setLEDStatus(LED_STATUS_INIT);  // Show initialization status
#endif

#ifdef EEPROM_UTILS_H
    initEEPROM();   // Initialize EEPROM system
    saveMotorDataToEEPROM(803.0f, 664.0f, 629.1f, false);
#endif

#ifdef ENCODER_H
    encoderInit();  // Initialize the encoder
#endif

#ifdef MOTOR_CONTROL_H
    // Motor alignment    
    findConstOffset(false, 2.0f, 0.05f, 0.5f, CW);  // true for active calibration 
    const_rotor_offset_cw = GET_ROTOR_OFFSET_CW();
    const_rotor_offset_ccw = GET_ROTOR_OFFSET_CCW();
    rotor_offset_abs = GET_ROTOR_OFFSET_ABS();
    
    rotor_offset_ccw = findRotorOffset(2.0f, 0.004f, CCW, 2.0f);
    rotor_offset_cw = findRotorOffset(2.0f, 0.004f, CW, 2.0f);
    setPWMdutyCycle();  // reset PWM duty cycle to zero
#endif

    // Wait for start button press
    while (!SW_START_PRESSING)
    {
        updateRawRotorAngle();  
        updateMultiTurnTracking();

        abs_angle = readRotorAbsoluteAngle(WITHOUT_ABS_OFFSET);
        abs_angle_with_offset = readRotorAbsoluteAngle(WITH_ABS_OFFSET);
        calibration_angle = -90.0f; // both motor = -90.0f

        // Check if angle is within calibration range (±5 degrees tolerance)
        float angle_error = abs(abs_angle_with_offset - (calibration_angle * GEAR_RATIO));
        if (angle_error <= 20.0f)
            setLEDStatus(LED_STATUS_READY);  // Green - within calibration range
        else 
            setLEDStatus(LED_STATUS_INIT);  // Red - outside calibration range

        SystemSerial->print("Turns: ");
        SystemSerial->print((float)rotor_turns, SERIAL1_DECIMAL_PLACES);
        SystemSerial->print("\t\tAngle w/o offset: ");
        SystemSerial->print(abs_angle, SERIAL1_DECIMAL_PLACES);
        SystemSerial->print("\t\tOffset: ");
        SystemSerial->print(computeOffsetAngleIK(calibration_angle, abs_angle), SERIAL1_DECIMAL_PLACES);
        SystemSerial->print("\t\tAngle w offset : ");
        SystemSerial->println(abs_angle_with_offset, SERIAL1_DECIMAL_PLACES);
        delay(10);
    }
    
    // Set default vd and vq for commutation test
    vd_cmd = 0.0;  
    vq_cmd = -8.0; 

    // Start after button release
    delay(1500); // Debounce delay
    SystemSerial->println("Starting...");   
    setLEDStatus(LED_STATUS_RUNNING);

    // Initialize position control S-curve
    scurve.plan(abs_angle_with_offset, calibration_angle * GEAR_RATIO, 1000.0f, 40000.0f, 1000.0f, 40000.0f);
    start_scurve_time = micros(); // Record the start time in microseconds
}

void loop()
{
    // Uncomment to test open-loop park function
    // testParkOpenLoop(0.0, 2.0, 0.08, false);
    // updateRawRotorAngle();
    // SystemSerial->println(readRotorAngle());    
    
#ifdef LED_H
    // Update LED effects
    updateLEDStatus();
#endif

    // Get current time
    uint32_t current_time = micros();

    // Update position setpoint for debugging
    if (SystemSerial->available())
    {
        if (SystemSerial->find('#'))
        {
            // #ifdef POSITION_CONTROL_ONLY
                // position_pid.setSetpoint(SystemSerial->parseInt());  // Read position setpoint from serial
            // #endif

            // #ifdef POSITION_CONTROL_WITH_SCURVE
                float new_setpoint = SystemSerial->parseFloat();
                float current_pos = readRotorAbsoluteAngle(WITH_ABS_OFFSET);
                if (new_setpoint != position_pid.setpoint) 
                {
                    scurve.plan(current_pos, new_setpoint, 4000.0f, 180000.0f, 1000.0f, 180000.0f);
                    start_scurve_time = micros(); // Record the start time in microseconds
                }
            // #endif
        }
    }

    // Position controller
    if (current_time - last_position_control_time >= POSITION_CONTROL_PERIOD_US)
    {
        last_position_control_time = current_time;

        // #ifdef POSITION_CONTROL_ONLY
            // positionControl(readRotorAbsoluteAngle(), &vq_cmd);
        // #endif

        // #ifdef POSITION_CONTROL_WITH_SCURVE
            position_pid.setpoint = scurve.getPosition((current_time - start_scurve_time) / 1e6f);
            positionControl(readRotorAbsoluteAngle(), &vq_cmd);
        // #endif
    }
    
    // SVPWM controller
    if (current_time - last_svpwm_time >= SVPWM_PERIOD_US)
    {
        last_svpwm_time = current_time;

        // Update raw rotor angle and absolute angle
        updateRawRotorAngle();
        updateMultiTurnTracking();  

        // apply SVPWM control
        svpwmControl(vd_cmd, vq_cmd, readRotorAngle(vq_cmd>0? CCW: CW) * DEG_TO_RAD);
    }

    // Debug information
    if (current_time - last_debug_time >= DEBUG_PERIOD_US)
    { 
        last_debug_time = current_time;

        SystemSerial->print(position_pid.setpoint, SERIAL1_DECIMAL_PLACES);
        SystemSerial->print("\t");
        SystemSerial->println(readRotorAbsoluteAngle(), SERIAL1_DECIMAL_PLACES);
    }
}