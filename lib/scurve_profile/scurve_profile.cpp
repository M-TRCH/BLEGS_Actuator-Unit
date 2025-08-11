
#include "scurve_profile.h"
#include <cmath>

/* @brief   Plan the S-curve profile parameters
 * @param   start   Start position
 * @param   end     End position
 * @param   v_max   Max velocity
 * @param   a_max   Max acceleration
 * @param   j_max   Max jerk
 */
void ScurveProfile::plan(float start, float end, float v_max, float a_max, float j_max) 
{
    q0 = start;
    q1 = end;
    vmax = v_max;
    amax = a_max;
    jmax = j_max;

    float dq = fabs(q1 - q0);

    Tj = amax / jmax;
    Ta = Tj + (vmax - amax * Tj) / amax;
    Tv = (dq - (amax * Ta * Ta + amax * Tj * Tj) / 2) / vmax;
    if (Tv < 0) Tv = 0;

    totalTime = 2 * Ta + Tv;
}

/* @brief   Get the position at time t
 * @param   t       Time in seconds
 * @return  Position in radians
 */
float ScurveProfile::getPosition(float t) 
{
    float dir = (q1 > q0) ? 1.0f : -1.0f;
    float pos = q0;

    if (t < 0) return q0;
    if (t > totalTime) return q1;

    if (t < Tj) {
        pos += dir * (jmax * t * t * t / 6.0f);
    } else if (t < (Ta - Tj)) {
        pos += dir * (amax / 6.0f * (3 * t * t - 3 * Tj * t + Tj * Tj));
    } else if (t < Ta) {
        float dt = t - (Ta - Tj);
        pos += dir * (amax * (Ta - Tj) * (Ta - Tj) / 2.0f + amax * (Ta - Tj) * dt + jmax * dt * dt * dt / 6.0f - amax * dt * dt / 2.0f);
    } else if (t < (Ta + Tv)) {
        pos += dir * (amax * Ta * Ta / 2.0f + vmax * (t - Ta));
    } else if (t < (Ta + Tv + Tj)) {
        float dt = t - (Ta + Tv);
        pos += dir * (amax * Ta * Ta / 2.0f + vmax * Tv + vmax * dt - jmax * dt * dt * dt / 6.0f);
    } else if (t < (totalTime - Tj)) {
        float dt = t - (Ta + Tv + Tj);
        pos += dir * (amax * Ta * Ta / 2.0f + vmax * Tv + vmax * Tj - amax / 6.0f * (3 * dt * dt + 3 * Tj * dt + Tj * Tj));
    } else {
        float dt = t - (totalTime - Tj);
        pos = q1 - dir * (jmax * (Tj - dt) * (Tj - dt) * (Tj - dt) / 6.0f);
    }

    return pos;
}