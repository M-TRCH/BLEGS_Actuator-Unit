
#include <Arduino.h>
#include "system.h"
#include "svpwm.h"
#include "encoder.h"
#include "motor_control.h"

void setup() 
{
  systemInit();   // Initialize the system
  encoderInit();  // Initialize the encoder  
  
  // Motor alignment
  findConstOffset(false, 2.0f, 0.05f, 0.5f, CW); 
  rotor_offset_ccw = findRotorOffset(2.0f, 0.004f, CCW, 2.0f);
  rotor_offset_cw = findRotorOffset(2.0f, 0.004f, CW, 2.0f);

  // Start up sequence 
  setPWMdutyCycle();  // Set initial PWM duty cycle to zero
  setLEDBuiltIn(false, true, false);  // Set CAL LED on, others off
  while(!SW_START_PRESSING);
  setLEDBuiltIn(true, false, false);  // Set RUN LED on, CAL LED off
}
  
void loop() 
{
  // Try to commutate the motor
  updateRawRotorAngle();
  applySVPWM(0.0f, -20.5f, readRotorAngle(CW) * DEG_TO_RAD); // Apply SVPWM with rotor angle in radians
  delayMicroseconds(1); // Delay for stability
  

  // updateRawRotorAngle();
  // Serial3.println(readRotorAngle(CCW));
  // testParkOpenLoop(0.0f, 2.5f, 60.0f, 1.0f, true); // Test Park transformation with zero q-axis voltage
}