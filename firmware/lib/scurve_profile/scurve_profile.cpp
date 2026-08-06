#include "scurve_profile.h"
#include <cmath>

/* @brief   Plan the trapezoidal profile parameters with auto amax limit
 * @param   start   Start position
 * @param   end     End position
 * @param   v_max   Max velocity
 * @param   a_max   Max acceleration (suggested)
 * @param   amax_min Minimum acceleration limit
 * @param   amax_max Maximum acceleration limit
 */
void ScurveProfile::plan(float start, float end, float v_max, float a_max, float amax_min, float amax_max) 
{
    q0 = start;
    q1 = end;
    vmax = v_max;

    float dq = fabs(q1 - q0);

    // Auto adjust amax based on distance and velocity, within min/max limits
    float amax_auto = vmax * vmax / (dq > 1e-6f ? dq : 1e-6f);
    amax = fmax(amax_min, fmin(amax_max, fmin(a_max, amax_auto)));
    amax = fmax(amax, 1e-6f); // Prevent divide by zero

    this->amax = amax;

    // Calculate acceleration time
    Ta = vmax / amax;
    float d_acc = 0.5f * amax * Ta * Ta;

    // Calculate constant velocity time
    Tv = (dq - 2.0f * d_acc) / vmax;
    if (Tv < 0) {
        // Triangle profile (no constant velocity)
        Ta = sqrt(dq / amax);
        Tv = 0.0f;
    }

    totalTime = 2.0f * Ta + Tv;

    // Set unused S-curve variables to zero for safety
    jmax = 0.0f;
    Tj = 0.0f;
}

/* @brief   Get the position at time t (trapezoidal profile)
 * @param   t       Time in seconds
 * @return  Position in units of q0/q1
 */
float ScurveProfile::getPosition(float t) 
{
    float dq = fabs(q1 - q0);
    float dir = (q1 > q0) ? 1.0f : -1.0f;

    if (t < 0) return q0;
    if (t > totalTime) return q1;

    if (t < Ta) {
        // Acceleration phase
        return q0 + dir * 0.5f * amax * t * t;
    } else if (t < (Ta + Tv)) {
        // Constant velocity phase
        float d_acc = 0.5f * amax * Ta * Ta;
        return q0 + dir * (d_acc + vmax * (t - Ta));
    } else {
        // Deceleration phase
        float td = t - (Ta + Tv);
        float d_acc = 0.5f * amax * Ta * Ta;
        return q0 + dir * (d_acc + vmax * Tv + vmax * td - 0.5f * amax * td * td);
    }
}

/* @brief   Get the velocity at time t (trapezoidal profile)
 * @param   t       Time in seconds
 * @return  Velocity in units of q/s (degrees/second)
 */
float ScurveProfile::getVelocity(float t) 
{
    float dir = (q1 > q0) ? 1.0f : -1.0f;

    if (t < 0) return 0.0f;
    if (t > totalTime) return 0.0f;

    if (t < Ta) {
        // Acceleration phase: v = a * t
        return dir * amax * t;
    } else if (t < (Ta + Tv)) {
        // Constant velocity phase
        return dir * vmax;
    } else {
        // Deceleration phase: v = vmax - a * (t - Ta - Tv)
        float td = t - (Ta + Tv);
        return dir * (vmax - amax * td);
    }
}