
#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>
#include <HardwareSerial.h>

// (0) Pin definitions for the STM32G431CBU6 board
#define LED_RUN_PIN     PC10
#define LED_CAL_PIN     PC11
#define LED_ERR_PIN     PC13
#define SW_STOP_PIN     PA0
#define SW_START_PIN    PA1
#define PWM_A_PIN       PB0
#define PWM_B_PIN       PB1
#define PWM_C_PIN       PB13
#define MISO_PIN        PA6 
#define MOSI_PIN        PA7
#define SCK_PIN         PA5
#define CS_ENC_PIN      PB12
#define ADC_IA_PIN      PA2
#define ADC_IC_PIN      PA3
#define ADC_VDC_PIN     PA4
#define SEN_1_PIN       PB9
#define SEN_2_PIN       PA15
#define RX_PIN          PB11
#define TX_PIN          PB10
#define RX3_PIN         PA10
#define TX3_PIN         PA9

// (1) System constants
#define SERIAL3_BAUDRATE        2000000U
#define SERIAL3_TIMEOUT         50U
#define SERIAL3_DECIMAL_PLACES  2
#define ANALOG_READ_RESOLUTION  12U
#define PWM_FREQUENCY           20000U
#define PWM_RESOLUTION          12U

// (2) Object definitions
extern HardwareSerial Serial3;

// (3) Macros definitions
#define SW_START_PRESSING  (digitalRead(SW_START_PIN) == LOW)

// (4) Function declarations
void systemInit();
void setLEDBuiltIn(bool run=false, bool cal=false, bool err=false);

#endif // SYSTEM_H