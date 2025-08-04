
#include "System.h"

HardwareSerial Serial3(RX3_PIN, TX3_PIN); // RX, TX pins for Serial3

void systemInit() 
{
    // Initialize Serial3 with the defined baud rate
    Serial3.begin(SERIAL3_BAUDRATE); // Initialize Serial3 with defined baud rate

    // Initialize pins
    pinMode(LED_RUN_PIN, OUTPUT);
    pinMode(LED_CAL_PIN, OUTPUT);
    pinMode(LED_ERR_PIN, OUTPUT); 
    pinMode(SW_STOP_PIN, INPUT);
    pinMode(SW_START_PIN, INPUT);
    pinMode(SEN_1_PIN, INPUT);
    pinMode(SEN_2_PIN, INPUT);

    analogReadResolution(12); // Set ADC resolution to 12 bits
}