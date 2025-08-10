
#ifndef SVPWM_H
#define SVPWM_H
#include <Arduino.h>
#include "system.h"

// (0) System constants
#define VDC     24.0f // DC bus voltage, used for SVPWM calculations

// (1) Function declarations
void setPWMdutyCycle(float dutyA=0, float dutyB=0, float dutyC=0);
void applySVPWM(float v_alpha, float v_beta);
void testClarkeOpenLoop(float amplitude, float velocity, float sampleTime, bool ccw=true);
void applySVPWM(float v_d, float v_q, float theta_rad);
void testParkOpenLoop(float v_d, float v_q, float velocity, float sampleTime, bool ccw=true);

#endif // SVPWM_H