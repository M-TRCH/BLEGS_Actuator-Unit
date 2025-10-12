
#ifndef SVPWM_H
#define SVPWM_H
#include <Arduino.h>
#include "system.h"

// System constants
#define VDC     12.0f // DC bus voltage, used for SVPWM calculations

// Function prototypes
void setPWMdutyCycle(float dutyA=0, float dutyB=0, float dutyC=0);
void applySVPWM(float v_alpha, float v_beta);
void applySVPWM(float v_d, float v_q, float theta_rad);
void testParkOpenLoop(float v_d=0.0f, float v_q=2.0f, float increment=0.06f, bool ccw=false);

#endif // SVPWM_H