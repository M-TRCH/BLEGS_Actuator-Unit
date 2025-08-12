
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <Arduino.h>
#include "motor_conf.h"
#include "svpwm.h"
#include "encoder.h"
#include "pid_controller.h"

// (0) System constants
#define SVPWM_FREQUENCY             10000.0                                 // 10 kHz PWM frequency
#define POSITION_CONTROL_FREQUENCY  5000.0                                  // 2 kHz position control frequency
#define DEBUG_FREQUENCY             50.0                                    // 50 Hz debug frequency
#define S_CURVE_FREQUENCY           500.0                                   // 500 Hz debug frequency
#define SVPWM_PERIOD                (1.0f / SVPWM_FREQUENCY)                // PWM period in nanoseconds
#define SVPWM_PERIOD_US             (SVPWM_PERIOD * 1e6)                    // PWM period in microseconds
#define POSITION_CONTROL_PERIOD     (1.0f / POSITION_CONTROL_FREQUENCY)     // Position control period in seconds
#define POSITION_CONTROL_PERIOD_US  (POSITION_CONTROL_PERIOD * 1e6)         // Position control period in microseconds
#define DEBUG_PERIOD                (1.0f / DEBUG_FREQUENCY)                // Debug period in seconds
#define DEBUG_PERIOD_US             (DEBUG_PERIOD * 1e6)                    // Debug period in microseconds 
#define S_CURVE_PERIOD              (1.0f / S_CURVE_FREQUENCY)              // S-curve period in seconds
#define S_CURVE_PERIOD_US           (S_CURVE_PERIOD * 1e6)                  // S-curve period in microseconds

// (1) Global variables
// svpwm variables
extern float vq_cmd; // Current q-axis voltage command
extern float vd_cmd; // Current d-axis voltage command
// scheduling variables
extern unsigned long last_svpwm_time;               // Last SVPWM time in microseconds
extern unsigned long last_position_control_time;    // Last position control time in microseconds
extern unsigned long last_debug_time;               // Last debug time in microseconds
// position control variables
extern PIDController position_pid;                  // Position PID controller

// (2) Configuration
// #define POSITION_CONTROL_ONLY
#define POSITION_CONTROL_WITH_SCURVE

void findConstOffset(bool active, float v_mag, float step_angle, float step_offset, bool ccw = true);
float findRotorOffset(float v_mag, float step_angle, bool ccw = true, float revolution = 1.0f);
void svpwmControl(float vd_ref = 0.0f, float vq_ref = 0.0f, float angle_rad = 0.0f);
void positionControl(float measured, float *output);

#endif // MOTOR_CONTROL_H
