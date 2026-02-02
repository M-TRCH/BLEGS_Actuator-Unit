#include <Arduino.h>
#include "system.h"
#include "svpwm.h"
#include "encoder.h"
#include "current.h"
#include "motor_control.h"
#include "eeprom_utils.h"
#include "led.h"
#include "scurve.h"
#include "protocol.h"

// Initial back to basics control

// Control mode enumeration (legacy)
enum ControlMode_t 
{
    POSITION_CONTROL_ONLY = 0,
    POSITION_CONTROL_WITH_SCURVE = 1
};

uint32_t last_find_time;
float abs_angle, abs_angle_with_offset, calibration_angle;
ControlMode_t control_mode = POSITION_CONTROL_WITH_SCURVE;  // Default mode

// Protocol variables
BinaryPacket rx_packet;
bool binary_mode_enabled = true;  // Enable binary protocol by default
bool emergency_stop_active = false;  // Emergency stop flag (can be reset via PKT_CMD_RESET_EMERGENCY)

// Emergency stop protection - double-confirmation mechanism
bool estop_pending = false;             // First emergency stop received, waiting for confirmation
uint32_t estop_pending_time = 0;        // Time when first emergency stop was received
uint8_t estop_pending_motor_id = 0;     // Motor ID from pending emergency stop

// Command queue to prevent motor jerking
#define CMD_QUEUE_SIZE 4
struct CommandQueue 
{
    float target_positions[CMD_QUEUE_SIZE];
    uint8_t modes[CMD_QUEUE_SIZE];
    SCurveParams scurve_params[CMD_QUEUE_SIZE];  // S-Curve parameters for each command
    uint16_t durations[CMD_QUEUE_SIZE];          // Duration for auto-calc mode
    uint8_t write_idx;
    uint8_t read_idx;
    uint8_t count;
} cmd_queue = {0};

// Rate limiting for setpoint updates
uint32_t last_setpoint_update_time = 0;
#define MIN_SETPOINT_UPDATE_INTERVAL_US 500  // 0.5ms minimum (2000 Hz max) - faster response

// Target tracking
float last_target_position = 0.0f;
#define POSITION_DEADBAND 0.5f  // degrees - ignore small changes

void setup() 
{
#ifdef SYSTEM_H
    systemInit(SERIAL_SYSTEM, 2000);    // Initialize the system with 2s wait time
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
    saveMotorDataToEEPROM(157.0f,     47.0f,    577.0f,     1, false);  // motor FL-1 (OK)
    saveMotorDataToEEPROM(174.0f,   3600.0f,    564.0f,     2, false);  // motor FL-2 (OK)
    saveMotorDataToEEPROM(589.0f,    527.0f,    837.0f,     3, false);  // motor FR-3 (OK) 
    saveMotorDataToEEPROM(102.0f,   1164.0f,    705.0f,     4, false);  // motor FR-4 (OK)
    saveMotorDataToEEPROM(622.0f,    535.0f,    685.0f,     5, false);  // motor RL-5 (OK) 
    saveMotorDataToEEPROM(477.0f,    452.0f,    827.0f,     6, false);  // motor RL-6 (OK) 
    saveMotorDataToEEPROM(727.0f,    648.0f,    737.0f,     7, false);  // motor RR-7 (OK)
    saveMotorDataToEEPROM(786.0f,    680.0f,    762.0f,     8, false);  // motor RR-8 (OK) 
#endif

#ifdef ENCODER_H
    encoderInit();  // Initialize the encoder
#endif

#ifdef CURRENT_H
    currentInit();  // Initialize current sensing
#endif

#ifdef MOTOR_CONTROL_H
    // Motor alignment    
    findConstOffset(false, 4.0f, 0.05f, 0.5f, CCW);  // true for active calibration 
    const_rotor_offset_cw = GET_ROTOR_OFFSET_CW();
    const_rotor_offset_ccw = GET_ROTOR_OFFSET_CCW();
    rotor_offset_abs = GET_ROTOR_OFFSET_ABS();
    
    rotor_offset_ccw = findRotorOffset(2.0f, 0.004f, CCW, 2.0f);
    rotor_offset_cw = findRotorOffset(2.0f, 0.004f, CW, 2.0f);
    setPWMdutyCycle();  // reset PWM duty cycle to zero
#endif

    // Calibration verification loop
    calibration_angle = -90.0f; // both motor = -90.0f
    while (0)
    {
        updateRawRotorAngle();  
        updateMultiTurnTracking();

        abs_angle = readRotorAbsoluteAngle(WITHOUT_ABS_OFFSET);
        abs_angle_with_offset = readRotorAbsoluteAngle(WITH_ABS_OFFSET);
        
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
    }

    // Wait for start button press or serial start command
    bool start_requested = false;
    while (!start_requested)
    {
        // Check for button press
        if (SW_START_PRESSING)
        {
            start_requested = true;
        }
        
        // Check for serial start command (Binary or ASCII)
        if (SystemSerial->available() >= 4)
        {
            // Check if it's a binary packet
            if (isBinaryPacketAvailable(SystemSerial))
            {
                BinaryPacket start_packet;
                if (receivePacket(SystemSerial, &start_packet, 5))
                {
                    // Accept PING as start signal during initialization
                    if (start_packet.packet_type == PKT_CMD_PING)
                    {
                        SystemSerial->println("Start command received via binary protocol (PING)");
                        start_requested = true;
                        
                        // Send response
                        int32_t pos_feedback = (int32_t)(abs_angle_with_offset * 100.0f);
                        sendStatusFeedback(SystemSerial, GET_MOTOR_ID(), pos_feedback, 0, 0);
                    }
                }
            }
        }
        else if (SystemSerial->available())
        {
            // Check for ASCII start command ('S' or 's')
            char cmd = SystemSerial->peek();
            if (cmd == 'S' || cmd == 's')
            {
                SystemSerial->read();  // Consume the character
                SystemSerial->println("Start command received via serial (ASCII)");
                start_requested = true;
            }
        }
        
        delay(10);
    }
    
    // Commutation test voltage (24V supply: Vmax = 13.86V)
    // Recommended: ±2.0 (slow/safe) | ±5.0 (normal) | ±8.0 (fast) | ±10.0 (very fast) | ±13.0 (max/danger)
    vd_cmd = 0.0;  
    vq_cmd = -5.0;  // Normal speed (CW: negative, CCW: positive) 
    
    // Start after button release
    SystemSerial->println("Starting...");   
    setLEDStatus(LED_STATUS_RUNNING);

    // Initialize position control S-curve
    // updateRawRotorAngle();  
    // updateMultiTurnTracking();
    // abs_angle_with_offset = readRotorAbsoluteAngle(WITH_ABS_OFFSET);
    // scurve.plan(abs_angle_with_offset, calibration_angle * GEAR_RATIO, 1000.0f, 40000.0f, 1000.0f, 40000.0f);
    // start_scurve_time = micros(); // Record the start time in microseconds
    
    position_pid.setpoint = -90.0f * GEAR_RATIO; // Target position in degrees (with gear ratio)
}

void loop()
{
    // Update LED effects
    updateLEDStatus();

    // Uncomment to test open-loop park function
    while (0)
    {
        testParkOpenLoop(0.0, 2.0, 0.08, false);
        updateRawRotorAngle();
        SystemSerial->println(readRotorAngle());    
    }
    
    // Get current time
    uint32_t current_time = micros();
    uint32_t current_time_ms = millis();
    
    // DISABLED: Reset emergency stop pending state if timeout expired
    // if (estop_pending && (current_time_ms - estop_pending_time) >= ESTOP_CONFIRMATION_TIMEOUT_MS)
    // {
    //     estop_pending = false;
    //     // Optional: Log timeout
    //     // SystemSerial->println("Emergency stop pending timeout - cancelled");
    // }

    // Set endless_drive_mode to true for continuous rotation
    static bool endless_drive_mode = false; 
    if (endless_drive_mode)
    {

    }
    else 
    {
        // Handle incoming serial communication (Hybrid Protocol)
        // Process only if enough data available to avoid blocking
        if (SystemSerial->available() >= 4)  // At least header + type + len
        {
            // Check if it's a binary packet or ASCII command
            if (binary_mode_enabled && isBinaryPacketAvailable(SystemSerial))
            {
                // Binary protocol path - with non-blocking check
                if (receivePacket(SystemSerial, &rx_packet, 5))  // Reduced timeout to 5ms
                {
                    // Process packet based on type
                    switch (rx_packet.packet_type)
                    {
                        case PKT_CMD_SET_GOAL:
                        {
                            // SIMPLIFIED: Direct position control only (no queue, no S-curve)
                            float target_pos;
                            uint16_t duration_ms;
                            uint8_t mode;
                            
                            PayloadSetGoal* goal_payload = (PayloadSetGoal*)rx_packet.payload;
                            
                            if (processSetGoalPayload(goal_payload, &target_pos, &duration_ms, &mode))
                            {
                                // Direct position control - set setpoint immediately
                                position_pid.setpoint = target_pos;
                                
                                // Send ACK with current status
                                float current_pos = readRotorAbsoluteAngle(WITH_ABS_OFFSET);
                                int32_t pos_feedback = (int32_t)(current_pos * 100.0f);
                                currentUpdate();
                                int16_t current_mA = (int16_t)(currentEstimateDC() * 1000.0f);
                                sendStatusFeedback(SystemSerial, GET_MOTOR_ID(), pos_feedback, current_mA, 0x01);
                            }
                            else
                            {
                                sendErrorFeedback(SystemSerial, ERR_INVALID_PAYLOAD, PKT_CMD_SET_GOAL);
                            }
                            break;
                        }
                        
                        case PKT_CMD_PING:
                        {
                            // Simple ping response
                            int32_t pos_feedback = (int32_t)(readRotorAbsoluteAngle(WITH_ABS_OFFSET) * 100.0f);
                            currentUpdate();
                            int16_t current_mA = (int16_t)(currentEstimateDC() * 1000.0f);
                            sendStatusFeedback(SystemSerial, GET_MOTOR_ID(), pos_feedback, current_mA, 0);
                            break;
                        }
                        
                        // DISABLED: Emergency stop commands (testing direct position control only)
                        // case PKT_CMD_EMERGENCY_STOP:
                        // case PKT_CMD_RESET_EMERGENCY:
                        
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
            // DISABLED: ASCII protocol and command queue processing
            // else
            // {
            //     char cmd = SystemSerial->read();
            //     // ... ASCII protocol handling ...
            // }
        }

        // DISABLED: Command queue processing (testing direct position control only)
        // if (!emergency_stop_active && cmd_queue.count > 0 && ...)

        // Position controller - DIRECT POSITION CONTROL ONLY
        if (current_time - last_position_control_time >= POSITION_CONTROL_PERIOD_US)
        {
            last_position_control_time = current_time;
            
            // Direct position control (no velocity feedforward, no emergency stop check)
            positionControl(readRotorAbsoluteAngle(WITH_ABS_OFFSET), &vq_cmd, 0.0f);
        }
    }

    // SVPWM controller - NO SAFETY CHECKS
    if (current_time - last_svpwm_time >= SVPWM_PERIOD_US)
    {
        last_svpwm_time = current_time;

        // Update raw rotor angle and absolute angle
        updateRawRotorAngle();
        updateMultiTurnTracking();  

        // Apply SVPWM control directly
        svpwmControl(vd_cmd, vq_cmd, readRotorAngle(vq_cmd>0? CCW: CW) * DEG_TO_RAD);
    }

    // Debug (disable by default)
    if ((current_time - last_debug_time >= DEBUG_PERIOD_US) && false)
    { 
        last_debug_time = current_time;
        // SystemSerial->print(position_pid.setpoint, SERIAL1_DECIMAL_PLACES);
        // SystemSerial->print("\t");
        // SystemSerial->println(readRotorAbsoluteAngle(), SERIAL1_DECIMAL_PLACES);
        currentUpdate();
        float dc_instant = currentEstimateDC();
        SystemSerial->println(dc_instant, 3);
    }
}