#include <Arduino.h>
#include "system.h"
#include "svpwm.h"
#include "encoder.h"
#include "motor_control.h"
#include "scurve.h"

void setup() 
{
    systemInit();   // Initialize the system
    encoderInit();  // Initialize the encoder  

    // Motor alignment
    setLEDBuiltIn(false, true, false);  // Set CAL LED on, others off
    findConstOffset(false, 2.0f, 0.05f, 0.5f, CW); 
    rotor_offset_ccw = findRotorOffset(2.0f, 0.004f, CCW, 2.0f);
    rotor_offset_cw = findRotorOffset(2.0f, 0.004f, CW, 2.0f);

    // Start up sequence 
    setPWMdutyCycle();  // Set initial PWM duty cycle to zero
    while(!SW_START_PRESSING && WAIT_START_PRESSING_ENABLE)
    {
        updateRawRotorAngle();  
        updateMultiTurnTracking();

        float abs_angle = readRotorAbsoluteAngle(WITHOUT_ABS_OFFSET);
        float abs_angle_with_offset = readRotorAbsoluteAngle(WITH_ABS_OFFSET);

        Serial3.print("Turns: ");
        Serial3.print((float)rotor_turns, SERIAL3_DECIMAL_PLACES);
        Serial3.print("\t\tAbsolute: ");
        Serial3.print(abs_angle, SERIAL3_DECIMAL_PLACES);
        Serial3.print("\t\tOffset: ");
        Serial3.print(computeOffsetAngleIK(HIP_PITCH_CALIBRATION_ANGLE, abs_angle), SERIAL3_DECIMAL_PLACES);
        Serial3.print("\t\tFinal Absolute: ");
        Serial3.println(abs_angle_with_offset, SERIAL3_DECIMAL_PLACES);

        delay(10);
    }
    setLEDBuiltIn(true, false, false);  // Set RUN LED on, CAL LED off
    // Set default vd and vq for commutation test
    vd_cmd = 0.0;  
    vq_cmd = 1.5;     
}

void loop()
{
    // Update position setpoint for debugging
    if (Serial3.available())
    {
        if (Serial3.find('#'))
        {
            #ifdef POSITION_CONTROL_ONLY
                position_pid.setSetpoint(Serial3.parseInt());  // Read position setpoint from serial
            #endif

            #ifdef POSITION_CONTROL_WITH_SCURVE
                float new_setpoint = Serial3.parseFloat();
                float current_pos = readRotorAbsoluteAngle();
                if (new_setpoint != position_pid.setpoint) 
                {
                    scurve.plan(current_pos, new_setpoint, 4000.0f, 180000.0f, 1000.0f, 180000.0f);
                    start_scurve_time = micros(); // Record the start time in microseconds
                }
            #endif
        }
    }    

    // SVPWM controller
    unsigned long current_time = micros();
    if (current_time - last_svpwm_time >= SVPWM_PERIOD_US)
    {
        last_svpwm_time = current_time;

        // Update raw rotor angle and absolute angle
        updateRawRotorAngle();
        updateMultiTurnTracking();  

        // apply SVPWM control
        svpwmControl(vd_cmd, vq_cmd, readRotorAngle(vq_cmd>0? CCW: CW) * DEG_TO_RAD);
    }

    // Position controller
    if (current_time - last_position_control_time >= POSITION_CONTROL_PERIOD_US)
    {
        last_position_control_time = current_time;

        #ifdef POSITION_CONTROL_ONLY
            positionControl(readRotorAbsoluteAngle(), &vq_cmd);
        #endif

        #ifdef POSITION_CONTROL_WITH_SCURVE
            position_pid.setpoint = scurve.getPosition((current_time - start_scurve_time) / 1e6f);
            positionControl(readRotorAbsoluteAngle(), &vq_cmd);
        #endif
    }

    // Debug information
    if (current_time - last_debug_time >= DEBUG_PERIOD_US)
    { 
        last_debug_time = current_time;

        Serial3.print(position_pid.setpoint, SERIAL3_DECIMAL_PLACES);
        Serial3.print("\t");
        Serial3.println(readRotorAbsoluteAngle(), SERIAL3_DECIMAL_PLACES);
    }
}