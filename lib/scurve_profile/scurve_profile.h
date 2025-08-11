#ifndef SCURVE_PROFILE_H
#define SCURVE_PROFILE_H

class ScurveProfile 
{
public:
    float q0;        // Start position
    float q1;        // End position
    float vmax;      // Max velocity
    float amax;      // Max acceleration
    float jmax;      // Max jerk
    float Tj;        // Jerk time
    float Ta;        // Acceleration time
    float Tv;        // Constant velocity time
    float totalTime; // Total profile time

    // Plan the S-curve profile parameters
    void plan(float start, float end, float v_max, float a_max, float j_max);

    // Get position at time t
    float getPosition(float t);
};

#endif // SCURVE_PROFILE_H