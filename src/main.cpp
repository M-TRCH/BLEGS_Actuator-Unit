
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
  vd_cmd = 0.0;   // Set default vd command
  vq_cmd = 0.0;  // Set default vq command
}
  
void loop() 
{

  // Update position setpoint
  if (Serial3.available())
  {
    if (Serial3.find('#'))
    {
      position_pid.setSetpoint(Serial3.parseInt());  // Read position setpoint from serial
    }
  }    

  // Update raw rotor angle   
  updateRawRotorAngle();
  
  // SVPWM controller
  if (micros() - last_svpwm_time >= SVPWM_PERIOD_US)
  {
    last_svpwm_time = micros();
    svpwmControl(vd_cmd, vq_cmd, readRotorAngle(vq_cmd>0? CCW: CW) * DEG_TO_RAD);  
  }

  // Position controller
  if (micros() - last_position_control_time >= POSITION_CONTROL_PERIOD_US)
  {
    last_position_control_time = micros();
    positionControl(raw_rotor_angle, &vq_cmd);
    vq_cmd = constrain(vq_cmd, -10.0f, 10.0f); // Constrain q-axis voltage command
  }

  // Debug information
  if (micros() - last_debug_time >= DEBUG_PERIOD_US)
  { 
    last_debug_time = micros();
    Serial3.print(position_pid.setpoint);
    Serial3.print("\t");
    Serial3.println(raw_rotor_angle);
  }

}