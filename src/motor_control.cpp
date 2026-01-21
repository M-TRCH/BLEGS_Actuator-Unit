 
#include "motor_control.h"

// (1) Global variables
// svpwm variables
float vq_cmd = 0.0f;    // Current q-axis voltage command
float vd_cmd = 0.0f;    // Current d-axis voltage command
// scheduling variables
unsigned long last_svpwm_time = 0;               // Last SVPWM time in microseconds
unsigned long last_position_control_time = 0;    // Last position control time in microseconds
unsigned long last_debug_time = 0;               // Last debug time in microseconds
// position control variables
// Position PID controller (Kp, Ki, Kd, integral_limit, output_limit, tolerance)
// Kp = 0.04 (Proportional gain), Ki = 0.02 (Integral gain), Kd = 0.0 (Derivative gain)
// integral_limit = 4000.0 (Anti-windup limit), output_limit = 20.0 (Output saturation)
// tolerance = 0.5 (Deadband/error threshold for zero integral)
PIDController position_pid(0.04f, 0.02f, 0.0f, 2000.0f, 10.0f, 0.5f);

/* @brief   Find the constant offset for the rotor angle based on the applied voltage and step size
 * @param   active      Flag to indicate if the function is active
 * @param   v_mag       Magnitude of the voltage vector 
 * @param   step_angle  Step size for the angle in radians
 * @param   step_offset Step size for the offset in rotor angle
 * @param   ccw         Direction of rotation (true for counter-clockwise, false for clockwise)
 */
void findConstOffset(bool active, float v_mag, float step_angle, float step_offset, bool ccw) 
{
    static float theta_cmd = 0.0f;
    static float offset_cmd = 0.0f;

    while (active) 
    {
        // Update theta command base on direction
        if (ccw)
        {
            theta_cmd += step_angle;
            if (theta_cmd >= 2 * PI * POLE_PAIRS)   theta_cmd = 0;
        }
        else
        {   
            theta_cmd -= step_angle;
            if (theta_cmd < 0)  theta_cmd = 2 * PI * POLE_PAIRS;
        }
        
        // Clarke transformation
        float v_alpha = v_mag * cos(theta_cmd);
        float v_beta  = v_mag * sin(theta_cmd);
        applySVPWM(v_alpha, v_beta);

        // Read encoder and calculate offset
        updateRawRotorAngle();
        float rotor_angle = raw_rotor_angle; 
        float offset_rotor_angle = rotor_angle - offset_cmd;
        if (offset_rotor_angle < 0)     offset_rotor_angle += _14_BIT; 

        // // Update offset command
        offset_cmd += step_offset;
        if (offset_cmd > _14_BIT) offset_cmd = 0;

        // Debug info (theta_cmd, offset_rotor_angle, offset_cmd, offset calculated)
        SystemSerial->print(theta_cmd * RAD_TO_DEG / RAW_TO_ROTOR_ANGLE);
        SystemSerial->print("\t");
        SystemSerial->print(offset_rotor_angle);
        SystemSerial->print("\t");
        SystemSerial->print(offset_cmd);
        SystemSerial->print("\t");

        // Calculate base offset within electrical cycle
        float const_offset = offset_cmd; 
        while (const_offset >= _14_BIT_DIVIDED_PP) 
        {
            const_offset -= _14_BIT_DIVIDED_PP;
        }
        SystemSerial->println(const_offset);
    }
}

/* @brief   Find the rotor offset based on the applied voltage and step size
 * @param   v_mag       Magnitude of the voltage vector 
 * @param   step_angle  Step size for the angle in radians
 * @param   ccw         Direction of rotation (true for counter-clockwise, false for clockwise)
 * @param   revolution  Number of revolutions to sweep
 * @return  The calculated rotor offset in counts
 */
float findRotorOffset(float v_mag, float step_angle, bool ccw, float revolution) 
{
    // 0) Initialize the offset
    revolution *= 2.0f;

    // 1) Sweep the electrical angle one revolution
    float sweep_start = ccw ? -revolution * PI : revolution * PI;
    float sweep_end   = ccw ? 0.0f : 0.0;

    for (float theta = sweep_start; 
        ccw ? (theta <= sweep_end) : (theta >= sweep_end); 
        theta += ccw ? step_angle : -step_angle)
    {
        float v_alpha = v_mag * cos(theta);
        float v_beta  = v_mag * sin(theta);
        applySVPWM(v_alpha, v_beta);
    }

    // 2) Read raw rotor angle after alignment
    updateRawRotorAngle();
    float offset_coarse = raw_rotor_angle;
    float offset_fine = 0.0f;

    // 3) Calculate fine offset based on pole pair subdivision
    for (int pp = 0; pp <= POLE_PAIRS; pp++) 
    {
        if (offset_coarse < _14_BIT_DIVIDED_PP - (CCW ? const_rotor_offset_ccw : const_rotor_offset_cw)) 
        {
            if (pp == 0)
                offset_fine = raw_rotor_angle;  
            else  
                offset_fine = (ccw? const_rotor_offset_ccw: const_rotor_offset_cw) + (pp * _14_BIT_DIVIDED_PP);  
            break;
        } 
        else 
        {
            offset_coarse -= _14_BIT_DIVIDED_PP;
        }
    }
    return offset_fine; // Return encoder offset in counts
}

void svpwmControl(float vd_ref, float vq_ref, float angle_rad)
{
    applySVPWM(vd_ref, vq_ref, angle_rad); // Apply SVPWM with rotor angle in radians
}

/* @brief   Position control using PID with velocity feedforward
 * @param   measured     The measured position
 * @param   output       Pointer to the output voltage
 * @param   velocity_ff  Velocity feedforward (degrees/second)
 */
void positionControl(float measured, float *output, float velocity_ff)
{
    // PID feedback control
    float pid_output = position_pid.compute(measured, POSITION_CONTROL_PERIOD);
    
    // Velocity feedforward: convert velocity (deg/s) to voltage
    float ff_output = velocity_ff * VELOCITY_FF_GAIN;
    
    // Combined output: PID + Feedforward
    *output = pid_output + ff_output;
    
    // Apply output limit (same as PID output limit)
    if (*output > position_pid.output_limit) *output = position_pid.output_limit;
    else if (*output < -position_pid.output_limit) *output = -position_pid.output_limit;
}

