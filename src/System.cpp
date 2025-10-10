
#include "system.h"

HardwareSerial Serial1;
HardwareSerial Serial3;

// HardwareSerial Serial1(RX_PIN, TX_PIN);     // RX, TX pins for Serial1
// HardwareSerial Serial3(RX3_PIN, TX3_PIN);   // RX, TX pins for Serial3

void systemInit() 
{
    // Initialize Serial1 with the defined baud rate
    Serial1.begin(SERIAL1_BAUDRATE);
    Serial1.setTimeout(SERIAL1_TIMEOUT);

    // Initialize Serial3 with the defined baud rate
    Serial3.begin(SERIAL3_BAUDRATE);
    Serial3.setTimeout(SERIAL3_TIMEOUT);

    // Initialize pins
    pinMode(LED_RUN_PIN, OUTPUT);
    pinMode(LED_CAL_PIN, OUTPUT);
    pinMode(LED_ERR_PIN, OUTPUT); 
    pinMode(SW_STOP_PIN, INPUT);
    pinMode(SW_START_PIN, INPUT);
    pinMode(SEN_1_PIN, INPUT);
    pinMode(SEN_2_PIN, INPUT);

    analogReadResolution(ANALOG_READ_RESOLUTION); // Set ADC resolution to 12 bits for current sensing

    analogWriteFrequency(PWM_FREQUENCY);  
    analogWriteResolution(PWM_RESOLUTION);  // Set PWM resolution to 12 bits for SVPWM
}

/* @brief       Set the built-in LEDs
 * @param run   State for the RUN LED
 * @param cal   State for the CAL LED
 * @param err   State for the ERR LED
 */
void setLEDBuiltIn(bool run, bool cal, bool err)
{
    digitalWrite(LED_RUN_PIN, run);
    digitalWrite(LED_CAL_PIN, cal);
    digitalWrite(LED_ERR_PIN, err);
}