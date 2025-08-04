
#include <Arduino.h>
#include "System.h"


void setup() 
{
  systemInit(); // Initialize the system
}
  
void loop() 
{
  static bool ledState = LOW;
  digitalWrite(LED_RUN_PIN, ledState); // Toggle the LED state
  ledState = !ledState; // Update the state for next toggle
  delay(1000); // Wait for 1 second
}
