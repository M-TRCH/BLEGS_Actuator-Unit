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

    // Constructor with tolerance parameter
    PIDController(float kp, float ki, float kd, float i_limit = 0, float o_limit = 0, float tol = 0.0f)
        : Kp(kp), Ki(ki), Kd(kd), setpoint(0), integral(0), previous_error(0),
          integral_limit(i_limit), output_limit(o_limit), tolerance(tol) {}

    // Set a new setpoint and reset integral and previous error
    void setSetpoint(float sp) 
    {
        setpoint = sp;
        integral = 0;
        previous_error = 0;
    }

    // Compute PID output
    float compute(float measured, float dt) 
    {
        float error = setpoint - measured;

        // If error is within tolerance, reset integral and set error to zero
        if (fabs(error) <= tolerance) 
        {
            error = 0;
            integral = 0;
        } 
        else 
        {
            integral += error * dt;
        }

        // Limit integral value
        if (integral_limit > 0) 
        {
            if (integral > integral_limit) integral = integral_limit;
            else if (integral < -integral_limit) integral = -integral_limit;
        }

        float derivative = (error - previous_error) / dt;
        previous_error = error;
        float output = Kp * error + Ki * integral + Kd * derivative;

        // Limit output value
        if (output_limit > 0) 
        {
            if (output > output_limit) output = output_limit;
            else if (output < -output_limit) output = -output_limit;
        }

        return output;
    }
};

#endif // PID_CONTROLLER_H