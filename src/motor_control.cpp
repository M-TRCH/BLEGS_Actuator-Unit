 
#include "motor_control.h"

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
        Serial3.print(theta_cmd * RAD_TO_DEG / RAW_TO_ROTOR_ANGLE);
        Serial3.print("\t");
        Serial3.print(offset_rotor_angle);
        Serial3.print("\t");
        Serial3.print(offset_cmd);
        Serial3.print("\t");

        // Calculate base offset within electrical cycle
        float const_offset = offset_cmd; 
        while (const_offset >= _14_BIT_DIVIDED_PP) 
        {
            const_offset -= _14_BIT_DIVIDED_PP;
        }
        Serial3.println(const_offset);
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


