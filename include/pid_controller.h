
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController 
{
public:
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float previous_error;

    PIDController(float kp, float ki, float kd)
        : Kp(kp), Ki(ki), Kd(kd), setpoint(0), integral(0), previous_error(0) {}

    void setSetpoint(float sp) 
    {
        setpoint = sp;
        integral = 0;
        previous_error = 0;
    }

    float compute(float measured, float dt) 
    {
        float error = setpoint - measured;
        integral += error * dt;
        float derivative = (error - previous_error) / dt;
        previous_error = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }
};

#endif // PID_CONTROLLER_H