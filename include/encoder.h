
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <SPI.h>
#include "AS5047P.h"
#include "system.h"
#include "motor_conf.h"

// (0) Constants values
#define SPI_SPEED           200000
#define _14_BIT             16384
#define RAW_TO_ROTOR_ANGLE  (360.0 * POLE_PAIRS / _14_BIT)
#define _14_BIT_DIVIDED_PP  (_14_BIT / POLE_PAIRS)
#define CCW                 true
#define CW                  false

// (1) Global variables
extern uint16_t raw_rotor_angle; 
extern float const_rotor_offset_cw; 
extern float const_rotor_offset_ccw;
extern float rotor_offset_cw;       
extern float rotor_offset_ccw;       

// (2) Objects definition
extern AS5047P rotor;

// (3) Function prototypes
void encoderInit();
void updateRawRotorAngle();
float readRotorAngle(bool ccw=true);

#endif // ENCODER_H