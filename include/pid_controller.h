#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <cmath> // For fabs

class PIDController 
{
public:
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float previous_error;
    float integral_limit; 
    float output_limit;  
    float tolerance; // Error tolerance
    float error_limit; // Maximum allowable error (error clamping)

    // Constructor with tolerance and error_limit parameters
    PIDController(float kp, float ki, float kd, float i_limit = 0, float o_limit = 0, float tol = 0.0f, float err_limit = 0.0f)
        : Kp(kp), Ki(ki), Kd(kd), setpoint(0), integral(0), previous_error(0),
          integral_limit(i_limit), output_limit(o_limit), tolerance(tol), error_limit(err_limit) {}

    // Set a new setpoint and reset integral and previous error
    void setSetpoint(float sp) 
    {
        setpoint = sp;
        integral = 0;
        previous_error = 0;
    }

    // Compute PID output with back-calculation anti-windup and error clamping
    float compute(float measured, float dt) 
    {
        float error = setpoint - measured;

        // If error is within tolerance, reset integral and set error to zero
        if (fabs(error) <= tolerance) 
        {
            error = 0;
            integral = 0;
        }
        
        // Error clamping: limit maximum error to prevent motor stall/lock
        // This prevents P-term from becoming too large when target is far away
        if (error_limit > 0)
        {
            if (error > error_limit) error = error_limit;
            else if (error < -error_limit) error = -error_limit;
        }

        // Calculate P and D terms
        float P_term = Kp * error;
        float derivative = (error - previous_error) / dt;
        float D_term = Kd * derivative;
        previous_error = error;

        // Calculate unsaturated output (with current integral)
        float I_term = Ki * integral;
        float output_unsaturated = P_term + I_term + D_term;

        // Saturate output
        float output = output_unsaturated;
        if (output_limit > 0) 
        {
            if (output > output_limit) output = output_limit;
            else if (output < -output_limit) output = -output_limit;
        }

        // Back-calculation anti-windup:
        // Only integrate if NOT saturated, OR if error helps desaturate
        // (error sign opposite to output sign means it's helping to come back)
        bool is_saturated = (output != output_unsaturated);
        bool error_helps_desaturate = (error * output_unsaturated < 0);

        if (!is_saturated || error_helps_desaturate) 
        {
            // Safe to integrate
            if (fabs(error) > tolerance)  // Don't integrate if within deadband
            {
                integral += error * dt;
            }
            
            // Apply integral limit as secondary protection
            if (integral_limit > 0) 
            {
                if (integral > integral_limit) integral = integral_limit;
                else if (integral < -integral_limit) integral = -integral_limit;
            }
        }
        // If saturated and error is making it worse, don't integrate (freeze integral)

        return output;
    }
};

#endif // PID_CONTROLLER_H