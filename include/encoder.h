
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <SPI.h>
#include "AS5047P.h"
#include "system.h"
#include "motor_conf.h"
#include "config.h" 
    
// (0) Constants values
#define SPI_SPEED           200000
#define _14_BIT             16384
#define RAW_TO_ROTOR_ANGLE  (360.0 * POLE_PAIRS / _14_BIT)
#define RAW_TO_DEGREE       (360.0 / _14_BIT)
#define _14_BIT_DIVIDED_PP  (_14_BIT / POLE_PAIRS)
#define CCW                 true
#define CW                  false
#define WITH_ABS_OFFSET     true
#define WITHOUT_ABS_OFFSET  false

// (1) Global variables
// Encoder readings
extern uint16_t raw_rotor_angle; 
// Rotor offsets
extern float const_rotor_offset_cw; 
extern float const_rotor_offset_ccw;
extern float rotor_offset_cw;       
extern float rotor_offset_ccw;       
// Absolute rotor angle 
extern int rotor_turns;
extern float last_raw_angle_deg;
// Absolute rotor angle offset
extern float rotor_offset_abs;

// (2) Objects definition
extern AS5047P rotor;

void encoderInit(); 
void updateRawRotorAngle();
float readRotorAngle(bool ccw=true);
void updateMultiTurnTracking();
float readRotorAbsoluteAngle(bool with_offset=true);
float computeOffsetAngleIK(float angle_ik, float angle_mea);

#endif // ENCODER_H