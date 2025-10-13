#include <Arduino.h>
#include "system.h"
#include "svpwm.h"
#include "encoder.h"
#include "motor_control.h"
#include "eeprom_utils.h"
#include "led.h"
// #include "scurve.h"

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
    saveMotorDataToEEPROM(795.0f, 651.0f, 0.0f, false);
#endif

#ifdef ENCODER_H
    encoderInit();  // Initialize the encoder
#endif

#ifdef MOTOR_CONTROL_H
    // Motor alignment    
    findConstOffset(false, 2.0f, 0.05f, 0.5f, CCW); 
    const_rotor_offset_cw = GET_ROTOR_OFFSET_CW();
    const_rotor_offset_ccw = GET_ROTOR_OFFSET_CCW();

    rotor_offset_ccw = findRotorOffset(2.0f, 0.004f, CCW, 2.0f);
    rotor_offset_cw = findRotorOffset(2.0f, 0.004f, CW, 2.0f);
    setPWMdutyCycle();  // reset PWM duty cycle to zero
#endif

    /*
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
    */

    // Set default vd and vq for commutation test
    vd_cmd = 0.0;  
    vq_cmd = 8.0; 

    // Wait for start button press
    while (!SW_START_PRESSING);
    delay(1500); // Debounce delay
    SystemSerial->println("Starting...");   
    setLEDStatus(LED_STATUS_RUNNING);
}

void loop()
{
    // testParkOpenLoop(0.0, 2.0, 0.06, false);
    // updateRawRotorAngle();
    // SystemSerial->println(readRotorAngle());    

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

#ifdef LED_H
    // Update LED effects
    updateLEDStatus();
#endif

    // Debug information
    if (current_time - last_debug_time >= DEBUG_PERIOD_US)
    { 
        last_debug_time = current_time;

        SystemSerial->print(readRotorAngle(vq_cmd>0? CCW: CW), SERIAL1_DECIMAL_PLACES);
        SystemSerial->print("\t");
        SystemSerial->println(0);
    }

    /*
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
    */
}