
#include <Arduino.h>
#include "System.h"
#include "SVPWM.h"

void setup() 
{
  systemInit();                       // Initialize the system
  
  // start up sequence 
  setLEDBuiltIn(false, true, false);  // Set CAL LED on, others off
  while(!SW_START_PRESSING);
  setLEDBuiltIn(true, false, false);  // Set RUN LED on, CAL LED off
}
  
void loop() 
{
  // static bool ledState = LOW;
  // digitalWrite(LED_RUN_PIN, ledState); // Toggle the LED state
  // ledState = !ledState; // Update the state for next toggle
  // delay(1000); // Wait for 1 second
  // Serial3.println(VDC);

  testParkOpenLoop(0.0f, 2.5f, 60.0f, 1.0f); // Test Park transformation with zero q-axis voltage
}
