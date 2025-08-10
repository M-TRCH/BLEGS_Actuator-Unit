
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <Arduino.h>
#include "motor_conf.h"
#include "svpwm.h"
#include "encoder.h"

// (0) System constants

// (1) Function declarations

void findConstOffset(bool active, float v_mag, float step_angle, float step_offset, bool ccw = true);
float findRotorOffset(float v_mag, float step_angle, bool ccw = true, float revolution = 1.0f);

#endif // MOTOR_CONTROL_H
