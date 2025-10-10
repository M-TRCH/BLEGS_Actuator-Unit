
#include "system.h"

HardwareSerial Serial1(SYS_RX1_PIN, SYS_TX1_PIN);       // System Serial1 
HardwareSerial Serial3(RS485_RX3_PIN, RS485_TX3_PIN);   // RS485 Serial3 

/* @brief       Initialize the system
 * @details     This function initializes the serial communication, configures the pins,
 *              sets the ADC resolution for current sensing, and sets the PWM frequency
 *              and resolution for SVPWM.
 */
void systemInit() 
{
    // Initialize Serial1 with the defined baud rate
    Serial1.begin(SERIAL1_BAUDRATE);
    Serial1.setTimeout(SERIAL1_TIMEOUT);

    // Initialize Serial3 with the defined baud rate
    Serial3.begin(SERIAL3_BAUDRATE);
    Serial3.setTimeout(SERIAL3_TIMEOUT);

    // Initialize pins
    pinMode(LED_STATUS_PIN, OUTPUT);       
    pinMode(SW_CALC_PIN, INPUT);           
    pinMode(SW_START_PIN, INPUT);
    pinMode(SEN_1_PIN, INPUT);
    pinMode(SEN_2_PIN, INPUT);

    analogReadResolution(ANALOG_READ_RESOLUTION); // Set ADC resolution to 12 bits for current sensing

    analogWriteFrequency(PWM_FREQUENCY);  
    analogWriteResolution(PWM_RESOLUTION);  // Set PWM resolution to 12 bits for SVPWM
}