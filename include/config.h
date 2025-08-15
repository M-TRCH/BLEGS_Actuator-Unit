
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// (0) General configuration
#define WAIT_START_PRESSING_ENABLE      false   // Enable waiting for start button press before starting the motor

// (1) Motor configuration

// (2) Sensor configuration
#define RAW_ROTOR_ANGLE_INVERT          true    // Set to true if the raw rotor angle is inverted.
#define ROTOR_ABSOLUTE_ANGLE_INVERT     false   // Set to true if the absolute rotor angle is inverted.

#endif // CONFIG_H