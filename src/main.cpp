
#include <Arduino.h>
#include "system.h"
#include "svpwm.h"
#include "encoder.h"

void setup() 
{
  systemInit();   // Initialize the system
  encoderInit();  // Initialize the encoder  
  
  // Simplify motor alignment
  applySVPWM(0.0f, 2.5f, 0.0f);  
  delay(1000);                    // Wait for the motor to stabilize
  updateRawRotorAngle();          // Read the raw rotor angle
  const_rotor_offset_cw = raw_rotor_angle; 
  const_rotor_offset_ccw = raw_rotor_angle; 

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
  applySVPWM(0.0f, 2.0f, readRotorAngle(CCW) * (PI / 180.0f)); // Apply SVPWM with rotor angle in radians
  delayMicroseconds(1); // Delay for stability
  
  // Serial3.println(readRotorAngle());
  // testParkOpenLoop(0.0f, 2.5f, 60.0f, 1.0f, false); // Test Park transformation with zero q-axis voltage
}