#include <Arduino.h>
#include "system.h"
#include "svpwm.h"
#include "encoder.h"
#include "motor_control.h"
#include "eeprom_utils.h"
#include "led.h"
#include "scurve.h"
#include "protocol.h"

// Control mode enumeration (legacy)
enum ControlMode_t {
    POSITION_CONTROL_ONLY = 0,
    POSITION_CONTROL_WITH_SCURVE = 1
};

uint32_t last_find_time;
float abs_angle, abs_angle_with_offset, calibration_angle;
ControlMode_t control_mode = POSITION_CONTROL_WITH_SCURVE;  // Default mode

// Protocol variables
BinaryPacket rx_packet;
bool binary_mode_enabled = true;  // Enable binary protocol by default

void setup() 
{
#ifdef SYSTEM_H
    systemInit(SERIAL_SYSTEM);   // Initialize the system with RS232 pins
#endif

#ifdef PROTOCOL_H
    protocolInit();  // Initialize binary protocol
#endif

#ifdef LED_H
    initLED();                      // Initialize LED system
    setLEDStatus(LED_STATUS_INIT);  // Show initialization status
#endif

#ifdef EEPROM_UTILS_H
    initEEPROM();   // Initialize EEPROM system
    saveMotorDataToEEPROM(803.0f, 664.0f, 629.1f, false);   // motor 1 configuration
    saveMotorDataToEEPROM(486.0f, 325.0f, 709.3f, false);     // motor 2 configuration
#endif

#ifdef ENCODER_H
    encoderInit();  // Initialize the encoder
#endif

#ifdef MOTOR_CONTROL_H
    // Motor alignment    
    findConstOffset(false, 2.0f, 0.05f, 0.5f, CCW);  // true for active calibration 
    const_rotor_offset_cw = GET_ROTOR_OFFSET_CW();
    const_rotor_offset_ccw = GET_ROTOR_OFFSET_CCW();
    rotor_offset_abs = GET_ROTOR_OFFSET_ABS();
    
    rotor_offset_ccw = findRotorOffset(2.0f, 0.004f, CCW, 2.0f);
    rotor_offset_cw = findRotorOffset(2.0f, 0.004f, CW, 2.0f);
    setPWMdutyCycle();  // reset PWM duty cycle to zero
#endif

    // Wait for start button press or serial start command
    bool start_requested = false;
    while (!start_requested)
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
        
        // Check for button press
        if (SW_START_PRESSING)
        {
            start_requested = true;
        }
        
        // Check for serial start command (ASCII: 'S' or 's')
        if (SystemSerial->available())
        {
            char cmd = SystemSerial->peek();  // Peek without consuming
            if (cmd == 'S' || cmd == 's')
            {
                SystemSerial->read();  // Consume the character
                SystemSerial->println("Start command received via serial");
                start_requested = true;
            }
        }
        
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
    while (0)
    {
        testParkOpenLoop(0.0, 2.0, 0.08, false);
        updateRawRotorAngle();
        SystemSerial->println(readRotorAngle());    
    }
    
#ifdef LED_H
    // Update LED effects
    updateLEDStatus();
#endif

    // Get current time
    uint32_t current_time = micros();

    // Handle incoming serial communication (Hybrid Protocol)
    if (SystemSerial->available())
    {
        // Check if it's a binary packet or ASCII command
        if (binary_mode_enabled && isBinaryPacketAvailable(SystemSerial))
        {
            // Binary protocol path
            if (receivePacket(SystemSerial, &rx_packet, PROTOCOL_TIMEOUT_MS))
            {
                // Process packet based on type
                switch (rx_packet.packet_type)
                {
                    case PKT_CMD_SET_GOAL:
                    {
                        float target_pos;
                        uint16_t duration_ms;
                        uint8_t mode;
                        
                        PayloadSetGoal* goal_payload = (PayloadSetGoal*)rx_packet.payload;
                        
                        if (processSetGoalPayload(goal_payload, &target_pos, &duration_ms, &mode))
                        {
                            float current_pos = readRotorAbsoluteAngle(WITH_ABS_OFFSET);
                            
                            if (mode == MODE_DIRECT_POSITION)
                            {
                                // Direct position control
                                control_mode = POSITION_CONTROL_ONLY;
                                position_pid.setpoint = target_pos;
                            }
                            else if (mode == MODE_SCURVE_PROFILE)
                            {
                                // S-Curve profile control
                                control_mode = POSITION_CONTROL_WITH_SCURVE;
                                scurve.plan(current_pos, target_pos, 4000.0f, 180000.0f, 1000.0f, 180000.0f);
                                start_scurve_time = micros();
                            }
                            
                            // Send status feedback immediately
                            int32_t pos_feedback = (int32_t)(current_pos * 100.0f);  // degrees * 100
                            uint8_t status_flags = computeStatusFlags(
                                abs(current_pos - target_pos) > 1.0f,  // is_moving
                                abs(current_pos - target_pos) <= 1.0f,  // at_goal
                                false  // no error
                            );
                            sendStatusFeedback(SystemSerial, pos_feedback, 0, status_flags);
                        }
                        else
                        {
                            // Invalid payload
                            sendErrorFeedback(SystemSerial, ERR_INVALID_PAYLOAD, PKT_CMD_SET_GOAL);
                        }
                        break;
                    }
                    
                    case PKT_CMD_PING:
                    {
                        // Simple ping response
                        int32_t pos_feedback = (int32_t)(readRotorAbsoluteAngle(WITH_ABS_OFFSET) * 100.0f);
                        sendStatusFeedback(SystemSerial, pos_feedback, 0, 0);
                        break;
                    }
                    
                    default:
                    {
                        // Unknown command
                        sendErrorFeedback(SystemSerial, ERR_UNKNOWN_COMMAND, rx_packet.packet_type);
                        break;
                    }
                }
            }
            else
            {
                // CRC failed or timeout
                sendErrorFeedback(SystemSerial, ERR_CRC_FAILED);
            }
        }
        else
        {
            // Legacy ASCII protocol path
            char cmd = SystemSerial->read();
            
            // Mode selection commands
            if (cmd == 'M' || cmd == 'm') 
            {
                // Read mode number: M0 = POSITION_CONTROL_ONLY, M1 = POSITION_CONTROL_WITH_SCURVE
                int mode = SystemSerial->parseInt();
                if (mode == 0) 
                {
                    control_mode = POSITION_CONTROL_ONLY;
                    SystemSerial->println("Mode: Position Control Only");
                } 
                else if (mode == 1) 
                {
                    control_mode = POSITION_CONTROL_WITH_SCURVE;
                    SystemSerial->println("Mode: Position Control with S-Curve");
                }
            }
            // Position setpoint command
            else if (cmd == '#') 
            {
                float new_setpoint = SystemSerial->parseFloat();
                float current_pos = readRotorAbsoluteAngle(WITH_ABS_OFFSET);
                
                if (control_mode == POSITION_CONTROL_ONLY) 
                {
                    // Direct position control without S-curve
                    position_pid.setpoint = new_setpoint;
                    SystemSerial->print("Direct setpoint: ");
                    SystemSerial->println(new_setpoint, SERIAL1_DECIMAL_PLACES);
                } 
                else if (control_mode == POSITION_CONTROL_WITH_SCURVE) 
                {
                    // Position control with S-curve profile
                    if (new_setpoint != position_pid.setpoint) 
                    {
                        scurve.plan(current_pos, new_setpoint, 4000.0f, 180000.0f, 1000.0f, 180000.0f);
                        start_scurve_time = micros();
                        SystemSerial->print("S-Curve setpoint: ");
                        SystemSerial->println(new_setpoint, SERIAL1_DECIMAL_PLACES);
                    }
                }
            }
            // Toggle binary protocol mode
            else if (cmd == 'B' || cmd == 'b')
            {
                binary_mode_enabled = !binary_mode_enabled;
                SystemSerial->print("Binary mode: ");
                SystemSerial->println(binary_mode_enabled ? "ENABLED" : "DISABLED");
            }
        }
    }

    // Position controller (Comment fot endless drive)
    if (current_time - last_position_control_time >= POSITION_CONTROL_PERIOD_US)
    {
        last_position_control_time = current_time;

        if (control_mode == POSITION_CONTROL_ONLY) 
        {
            // Direct position control
            positionControl(readRotorAbsoluteAngle(), &vq_cmd);
        }
        else if (control_mode == POSITION_CONTROL_WITH_SCURVE) 
        {
            // Position control with S-curve profile
            position_pid.setpoint = scurve.getPosition((current_time - start_scurve_time) / 1e6f);
            positionControl(readRotorAbsoluteAngle(), &vq_cmd);
        }
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

    // Debug information (disable by default)
    if (current_time - last_debug_time >= DEBUG_PERIOD_US)
    { 
        last_debug_time = current_time;

        // SystemSerial->print("Returns:\t");
        // SystemSerial->print(position_pid.setpoint, SERIAL1_DECIMAL_PLACES);
        // SystemSerial->print("\t");
        // SystemSerial->println(readRotorAbsoluteAngle(), SERIAL1_DECIMAL_PLACES);
    }
}