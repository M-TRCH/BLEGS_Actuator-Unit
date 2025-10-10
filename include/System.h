
#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h" 

// Pin definitions for the STM32G431CBU6 board
#define LED_STATUS_PIN  PC13
#define SW_CALC_PIN     PA0
#define SW_START_PIN    PA1
#define PWM_A_PIN       PB0
#define PWM_B_PIN       PB1
#define PWM_C_PIN       PB13
#define MISO_PIN        PA6 
#define MOSI_PIN        PA7
#define SCK_PIN         PA5
#define CS_ENC_PIN      PB12
#define SEN_IA_PIN      PA2
#define SEN_IC_PIN      PA3
#define SEN_1_PIN       PB9
#define SEN_2_PIN       PA15
#define SYS_RX1_PIN     PA10
#define SYS_TX1_PIN     PA9
#define RS232_RX1_PIN   PB7
#define RS232_TX1_PIN   PC4
#define RS485_RX3_PIN   PB11
#define RS485_TX3_PIN   PB10

// System constants
#define SERIAL1_BAUDRATE        2000000
#define SERIAL1_TIMEOUT         50
#define SERIAL1_DECIMAL_PLACES  2
#define SERIAL3_BAUDRATE        9600
#define SERIAL3_TIMEOUT         50
#define SERIAL3_DECIMAL_PLACES  1
#define ANALOG_READ_RESOLUTION  12
#define PWM_FREQUENCY           20000
#define PWM_RESOLUTION          12

// Object definitions
extern HardwareSerial Serial1;
extern HardwareSerial Serial3;

// Macros definitions
#define SW_CALC_PRESSING   (digitalRead(SW_CALC_PIN) == LOW)
#define SW_START_PRESSING  (digitalRead(SW_START_PIN) == LOW)

void systemInit();

void setLEDBuiltIn(bool run=false, bool cal=false, bool err=false);

#endif // SYSTEM_H