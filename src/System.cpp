
#include "system.h"

// Serial objects - will be initialized in systemInit()
HardwareSerial* SystemSerial = nullptr;                 // Dynamic System Serial pointer
HardwareSerial Serial3(RS485_RX3_PIN, RS485_TX3_PIN);   // RS485 Serial3

// Static Serial objects for different configurations
static HardwareSerial SystemSerial_System(SYS_RX1_PIN, SYS_TX1_PIN);        // System Serial via System pins
static HardwareSerial SystemSerial_RS232(RS232_RX1_PIN, RS232_TX1_PIN);     // System Serial via RS232 pins 

/* @brief       Initialize the system
 * @details     This function initializes the serial communication, configures the pins,
 *              sets the ADC resolution for current sensing, and sets the PWM frequency
 *              and resolution for SVPWM.
 * @param       serial_output  Select System Serial output type (SERIAL_SYSTEM or SERIAL_RS232)
 */
void systemInit(SerialOutputType serial_output) 
{
    // Select and initialize SystemSerial based on the output type
    switch (serial_output) 
    {
        case SERIAL_SYSTEM:
            SystemSerial = &SystemSerial_System;
            break;
        case SERIAL_RS232:
            SystemSerial = &SystemSerial_RS232;
            break;
        default:
            SystemSerial = &SystemSerial_System;  // Default to system output
            break;
    }

    // Initialize selected SystemSerial with the defined baud rate
    SystemSerial->begin(SERIAL1_BAUDRATE);
    SystemSerial->setTimeout(SERIAL1_TIMEOUT);

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

    analogWriteFrequency(SVPWM_FREQUENCY);  
    analogWriteResolution(SVPWM_RESOLUTION);  // Set PWM resolution to 12 bits for SVPWM
}