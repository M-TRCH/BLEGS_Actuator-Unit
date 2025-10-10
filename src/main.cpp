#include <Arduino.h>
#include "system.h"
#include "svpwm.h"
#include "encoder.h"
#include "motor_control.h"
#include "scurve.h"
#include "config.h"
#include "eeprom_utils.h"
#include "ik_utils.h"

/* Compatible with r1.5 board */

void setup() 
{
    systemInit();   // Initialize the system
    encoderInit();  // Initialize the encoder  
    #if WRITE_MOTOR_DATA_TO_EEPROM 
        #if MOTOR_ROLE == HIP_PITCH
            saveMotorDataToEEPROM(1082.0f, 924.0f, 1286.72f);
        #elif MOTOR_ROLE == KNEE_PITCH
            saveMotorDataToEEPROM(324.0f, 150.0f, 1235.18f);
        #endif
    #endif

    // Motor alignment
    setLEDBuiltIn(false, true, false);  // Set CAL LED on, others off
    findConstOffset(false, 2.0f, 0.05f, 0.5f, CW); 
    
    // Load motor data from EEPROM
    loadMotorDataFromEEPROM(const_rotor_offset_cw, const_rotor_offset_ccw, rotor_offset_abs);
    rotor_offset_ccw = findRotorOffset(2.0f, 0.004f, CCW, 2.0f);
    rotor_offset_cw = findRotorOffset(2.0f, 0.004f, CW, 2.0f);

    // Start up sequence 
    setPWMdutyCycle();  // Set initial PWM duty cycle to zero
    uint32_t last_start_up_time = millis();
    while((!SW_START_PRESSING && WAIT_START_PRESSING_ENABLE) || (millis() - last_start_up_time < 3000))
    {
        updateRawRotorAngle();  
        updateMultiTurnTracking();

        float abs_angle = readRotorAbsoluteAngle(WITHOUT_ABS_OFFSET);
        float abs_angle_with_offset = readRotorAbsoluteAngle(WITH_ABS_OFFSET);
        float calibration_angle = MOTOR_ROLE == HIP_PITCH ? HIP_PITCH_CALIBRATION_ANGLE : KNEE_PITCH_CALIBRATION_ANGLE;
        float default_angle = MOTOR_ROLE == HIP_PITCH ? HIP_PITCH_DEFAULT_ANGLE : KNEE_PITCH_DEFAULT_ANGLE;

        Serial3.print("Turns: ");
        Serial3.print((float)rotor_turns, SERIAL3_DECIMAL_PLACES);
        Serial3.print("\t\tAbsolute: ");
        Serial3.print(abs_angle, SERIAL3_DECIMAL_PLACES);
        Serial3.print("\t\tOffset: ");
        Serial3.print(computeOffsetAngleIK(calibration_angle, abs_angle), SERIAL3_DECIMAL_PLACES);
        Serial3.print("\t\tFinal Absolute: ");
        Serial3.println(abs_angle_with_offset, SERIAL3_DECIMAL_PLACES);

        // Set up start s-curve setpoint with calibration angle
        scurve.plan(abs_angle_with_offset, default_angle, 1000.0f, 40000.0f, 1000.0f, 40000.0f);
        start_scurve_time = micros(); // Record the start time in microseconds
        delay(10);

        // Serial2.println(abs_angle_with_offset); // for serial2 testing
    }
    setLEDBuiltIn(true, false, false);  // Set RUN LED on, CAL LED off
    
    // Set default vd and vq for commutation test
    vd_cmd = 0.0;  
    vq_cmd = 0.0; //18.0;     
}

void loop()
{
    // Update position setpoint for debugging
    #ifdef POSITION_CONTROL || POSITION_CONTROL_WITH_SCURVE
        if (Serial3.available())
        {
            if (Serial3.find('#'))
            {
                #ifdef POSITION_CONTROL_ONLY
                    position_pid.setSetpoint(Serial3.parseInt());  // Read position setpoint from serial
                #endif

                #ifdef POSITION_CONTROL_WITH_SCURVE
                    float new_setpoint = Serial3.parseFloat();
                    float current_pos = readRotorAbsoluteAngle(WITH_ABS_OFFSET);
                    if (new_setpoint != position_pid.setpoint) 
                    {
                        scurve.plan(current_pos, new_setpoint, 4000.0f, 180000.0f, 1000.0f, 180000.0f);
                        start_scurve_time = micros(); // Record the start time in microseconds
                    }
                #endif
            }
        }
    #endif

    // Update cartesian position
    #ifdef LEG_CONTROL
        if (Serial2.available())
        {
            if (Serial2.find('#'))
            {
                float x_target = Serial2.parseInt();
                float y_target = Serial2.parseInt();
                float theta1, theta2, new_setpoint;
                ik2dof.setTarget(x_target, y_target, true);
                ik2dof.getJointAnglesDeg(theta1, theta2);
                
                #if MOTOR_ROLE == HIP_PITCH
                    new_setpoint = theta1 * GEAR_RATIO;
                #elif MOTOR_ROLE == KNEE_PITCH
                    new_setpoint = theta2 * -GEAR_RATIO;
                #endif
                
                float current_angle = readRotorAbsoluteAngle(WITH_ABS_OFFSET);
                if (current_angle != new_setpoint)
                {
                    scurve.plan(current_angle, new_setpoint, 4000.0f, 180000.0f, 1000.0f, 180000.0f);
                    start_scurve_time = micros(); // Record the start time in microseconds
                }
                // Serial3.print("Theta1: ");
                // Serial3.print(theta1, SERIAL3_DECIMAL_PLACES);
                // Serial3.print("\t\tTheta2: ");
                // Serial3.print(theta2, SERIAL3_DECIMAL_PLACES);
                // Serial3.print("\t\tActual: ");
                // Serial3.println(readRotorAbsoluteAngle(WITH_ABS_OFFSET), SERIAL3_DECIMAL_PLACES); 
            }
        }
    #endif

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

        #ifdef LEG_CONTROL
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