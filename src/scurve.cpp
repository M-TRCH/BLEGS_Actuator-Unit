#include "scurve.h"

ScurveProfile scurve;
float scurve_time = 0.0f;
unsigned long start_scurve_time = 0;

/* @brief   Plan the S-curve profile parameters
 * @param   start   Starting position
 * @param   end     Ending position
 * @param   vmax    Maximum velocity
 * @param   amax    Maximum acceleration
 * @param   jmax    Maximum jerk
 */
void scurvePlan(float start, float end, float vmax, float amax) 
{
    scurve.plan(start, end, vmax, amax);
    scurve_time = 0.0f;
    start_scurve_time = micros(); // Record the start time in microseconds
}

/*
 * @brief   Update the S-curve profile time
 * @param   dt      Time increment
 */
void scurveUpdateTime(float dt) 
{
    scurve_time += dt;
}

/*
 * @brief   Get the current position from the S-curve profile
 */
float scurveGetPosition(float time) 
{
    return scurve.getPosition(time);
}