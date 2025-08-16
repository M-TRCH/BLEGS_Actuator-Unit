
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// (0) General configuration
#define WAIT_START_PRESSING_ENABLE      true    // Enable waiting for start button press before starting the motor

// (1) Motor configuration
#define HIP_PITCH   0
#define KNEE_PITCH  1
#define MOTOR_ROLE  HIP_PITCH                   // Define the motor role (HIP_PITCH or KNEE_PITCH)

// (2) Sensor configuration
#define RAW_ROTOR_ANGLE_INVERT          true    // Set to true if the raw rotor angle is inverted.   

#if MOTOR_ROLE == HIP_PITCH
    #define ROTOR_ABSOLUTE_ANGLE_INVERT     false   
#elif MOTOR_ROLE == KNEE_PITCH
    #define ROTOR_ABSOLUTE_ANGLE_INVERT     true    
#endif

// (3) Inverse kinematics configuration
#define HIP_PITCH_CALIBRATION_ANGLE     -152.4f // Calibration angle for hip pitch
#define KNEE_PITCH_CALIBRATION_ANGLE    136.6f  // Calibration angle for knee pitch

#endif // CONFIG_H