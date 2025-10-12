
#include "svpwm.h"

/* @brief   Set the PWM duty cycles for the three motor phases
 * @param   dutyA  Duty cycle for phase A (0 to 4095 for 12-bit resolution)
 * @param   dutyB  Duty cycle for phase B (0 to 4095 for 12-bit resolution)
 * @param   dutyC  Duty cycle for phase C (0 to 4095 for 12-bit resolution)
 */
void setPWMdutyCycle(float dutyA, float dutyB, float dutyC) 
{
    analogWrite(PWM_A_PIN, dutyA);
    analogWrite(PWM_B_PIN, dutyB);
    analogWrite(PWM_C_PIN, dutyC);
}

/* @brief   Apply Space Vector PWM (SVPWM) to generate the appropriate PWM signals
 *          for the three motor phases based on the given αβ voltages.
 * @param   v_alpha  α-axis voltage component
 * @param   v_beta   β-axis voltage component
 */
void applySVPWM(float v_alpha, float v_beta)
{
    // 0) Vdc is the DC bus voltage, used to normalize the voltages
    float v_mag = sqrt(v_alpha * v_alpha + v_beta * v_beta);
    float v_limit = VDC / sqrt(3.0f);  // Max magnitude in SVPWM linear region

    if (v_mag > v_limit) 
    {
        float scale = v_limit / v_mag;
        v_alpha *= scale;
        v_beta  *= scale;
    }

    // 1) Convert αβ voltages to abc voltages
    float va = v_alpha;
    float vb = -0.5f * v_alpha + sqrt(3.0f)/2.0f * v_beta;
    float vc = -0.5f * v_alpha - sqrt(3.0f)/2.0f * v_beta;

    // 2) Center-aligned modulation requires offsetting the voltages to the range [0, Vdc]
    float v_max = max(va, max(vb, vc));
    float v_min = min(va, min(vb, vc));
    float v_offset = 0.5f * (v_max + v_min);

    float va_norm = (va - v_offset) / VDC + 0.5f;
    float vb_norm = (vb - v_offset) / VDC + 0.5f;
    float vc_norm = (vc - v_offset) / VDC + 0.5f;

    // 3) Ensure the normalized voltages are within [0.0, 1.0]
    va_norm = constrain(va_norm, 0.0f, 1.0f);
    vb_norm = constrain(vb_norm, 0.0f, 1.0f);
    vc_norm = constrain(vc_norm, 0.0f, 1.0f);

    // 4) Convert normalized voltages to PWM duty cycles
    const int PWM_MAX = 4095;  // 12-bit resolution
    uint16_t pwm_a = (uint16_t)(va_norm * PWM_MAX);
    uint16_t pwm_b = (uint16_t)(vb_norm * PWM_MAX);
    uint16_t pwm_c = (uint16_t)(vc_norm * PWM_MAX);

    // 5) Write the PWM values to the respective pins
    setPWMdutyCycle(pwm_a, pwm_b, pwm_c);
    // SystemSerial->print(pwm_a);  SystemSerial->print("\t");
    // SystemSerial->print(pwm_b);  SystemSerial->print("\t");
    // SystemSerial->println(pwm_c);
}

/* @brief   Apply Space Vector PWM (SVPWM) to generate the appropriate PWM signals
 *          for the three motor phases based on the given dq voltages and rotor angle.
 * @param   v_d        d-axis voltage component
 * @param   v_q        q-axis voltage component
 * @param   theta_rad  Electrical angle of the rotor in radians
 */
void applySVPWM(float v_d, float v_q, float theta_rad)
{
    // 0) Limit vd, vq to prevent overmodulation 
    float v_mag = sqrt(v_d * v_d + v_q * v_q);
    float v_limit = VDC / sqrt(3.0f);  // ≈ 0.577 * Vdc

    if (v_mag > v_limit) 
    {
        float scale = v_limit / v_mag;
        v_d *= scale;
        v_q *= scale;
    }

    // 1) Inverse Park Transform: dq to αβ
    float cos_theta = cos(theta_rad);
    float sin_theta = sin(theta_rad);

    float v_alpha = v_d * cos_theta - v_q * sin_theta;
    float v_beta  = v_d * sin_theta + v_q * cos_theta;

    // 2) Convert αβ voltages to abc voltages
    float va = v_alpha;
    float vb = -0.5f * v_alpha + sqrt(3.0f) / 2.0f * v_beta;
    float vc = -0.5f * v_alpha - sqrt(3.0f) / 2.0f * v_beta;

    // 3) Center-aligned modulation requires offsetting the voltages to the range [0, Vdc]
    float v_max = max(va, max(vb, vc));
    float v_min = min(va, min(vb, vc));
    float v_offset = 0.5f * (v_max + v_min);

    float va_norm = (va - v_offset) / VDC + 0.5f;
    float vb_norm = (vb - v_offset) / VDC + 0.5f;
    float vc_norm = (vc - v_offset) / VDC + 0.5f;

    // 4) Ensure the normalized voltages to [0.0, 1.0]
    va_norm = constrain(va_norm, 0.0f, 1.0f);
    vb_norm = constrain(vb_norm, 0.0f, 1.0f);
    vc_norm = constrain(vc_norm, 0.0f, 1.0f);

    // 5) Scale the normalized voltages to 12-bit PWM duty cycles
    const int PWM_MAX = 4095;  // 12-bit
    uint16_t pwm_a = (uint16_t)(va_norm * PWM_MAX);
    uint16_t pwm_b = (uint16_t)(vb_norm * PWM_MAX);
    uint16_t pwm_c = (uint16_t)(vc_norm * PWM_MAX);
  
    // 6) Write the PWM values to the respective pins
    setPWMdutyCycle(pwm_a, pwm_b, pwm_c);
    // SystemSerial->print(pwm_a);  SystemSerial->print("\t");
    // SystemSerial->print(pwm_b);  SystemSerial->print("\t");
    // SystemSerial->println(pwm_c);
}
 
/* @brief   Test function to apply a constant dq voltage vector while
 *          incrementally changing the electrical angle to simulate rotation.
 * @param   v_d        d-axis voltage component
 * @param   v_q        q-axis voltage component
 * @param   increment  Incremental change in electrical angle (radians)
 * @param   ccw        Direction of rotation (true for CCW, false for CW)
 */
void testParkOpenLoop(float v_d, float v_q, float increment, bool ccw)
{
    static float electrical_angle = 0.0f; // Initialize the electrical angle
    if (ccw)
    {
        electrical_angle += increment; // Increment the angle for testing
        if (electrical_angle > 2.0f * 3.14f)
        {
            electrical_angle -= 2.0f * 3.14f;
        }
    }
    else
    {
        electrical_angle -= increment; // Decrement the angle for testing
        if (electrical_angle < 0.0f)
        {
            electrical_angle += 2.0f * 3.14f;
        }
    }
    applySVPWM(v_d, v_q, electrical_angle);
}