
#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h" 

// Pin definitions for the STM32G431CBU6 board
// Updated pin definitions - referenced by other files
#define LED_STATUS_PIN  PC13        // Built-in LED for status indication
#define SW_CALC_PIN     PA0         // Calculation/Calibration switch
#define SW_START_PIN    PA1         // Start switch
#define PWM_A_PIN       PB0         // PWM output for motor phase A
#define PWM_B_PIN       PB1         // PWM output for motor phase B
#define PWM_C_PIN       PB13        // PWM output for motor phase C
#define MISO_PIN        PA6         // SPI MISO pin for encoder
#define MOSI_PIN        PA7         // SPI MOSI pin for encoder
#define SCK_PIN         PA5         // SPI SCK pin for encoder
#define CS_ENC_PIN      PB12        // Chip Select for encoder
#define SEN_IA_PIN      PA2         // Current sensor for phase A
#define SEN_IC_PIN      PA3         // Current sensor for phase C
#define SEN_1_PIN       PB9         // General sensor input 1
#define SEN_2_PIN       PA15        // General sensor input 2
#define SYS_RX1_PIN     PA10        // System Serial1 RX pin
#define SYS_TX1_PIN     PA9         // System Serial1 TX pin
#define RS232_RX1_PIN   PB7         // RS232 Serial RX pin
#define RS232_TX1_PIN   PC4         // RS232 Serial TX pin
#define RS485_RX3_PIN   PB11        // RS485 Serial3 RX pin
#define RS485_TX3_PIN   PB10        // RS485 Serial3 TX pin

// Serial selection enum
enum SerialOutputType {
    SERIAL_SYSTEM = 0,      // Use system pins (SYS_RX1_PIN, SYS_TX1_PIN)
    SERIAL_RS232 = 1        // Use RS232 pins (RS232_RX1_PIN, RS232_TX1_PIN)
};

// System constants
#define SERIAL1_BAUDRATE        2000000
#define SERIAL1_TIMEOUT         50
#define SERIAL1_DECIMAL_PLACES  2
#define SERIAL3_BAUDRATE        9600
#define SERIAL3_TIMEOUT         50
#define SERIAL3_DECIMAL_PLACES  1
#define ANALOG_READ_RESOLUTION  12
#define SVPWM_FREQUENCY         20000
#define SVPWM_RESOLUTION        12

// Object definitions
extern HardwareSerial* SystemSerial;   // Dynamic System Serial pointer (set in systemInit)
extern HardwareSerial Serial3;         // Fixed RS485 Serial3

// Macros definitions
#define SW_CALC_PRESSING   (digitalRead(SW_CALC_PIN) == LOW)
#define SW_START_PRESSING  (digitalRead(SW_START_PIN) == LOW)

void systemInit(SerialOutputType serial_output = SERIAL_SYSTEM);

#endif // SYSTEM_H