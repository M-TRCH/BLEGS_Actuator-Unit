#ifndef SCURVE_H
#define SCURVE_H

#include <Arduino.h>
#include "scurve_profile.h"

// Declare global S-curve profile object and variables
extern ScurveProfile scurve;
extern float scurve_time;
extern unsigned long start_scurve_time;

void scurvePlan(float start, float end, float vmax, float amax);
void scurveUpdateTime(float dt);
float scurveGetPosition(float time);

#endif // SCURVE_H