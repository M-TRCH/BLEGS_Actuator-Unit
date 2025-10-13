#ifndef LED_H
#define LED_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "system.h"

// LED Color Definitions (16 preset colors)
typedef enum {
    LED_COLOR_OFF = 0,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,
    LED_COLOR_YELLOW,
    LED_COLOR_CYAN,
    LED_COLOR_MAGENTA,
    LED_COLOR_WHITE,
    LED_COLOR_ORANGE,
    LED_COLOR_PINK,
    LED_COLOR_LIME,
    LED_COLOR_PURPLE,
    LED_COLOR_NAVY,
    LED_COLOR_MAROON,
    LED_COLOR_OLIVE,
    LED_COLOR_TEAL
} LEDColor_t;

// LED Brightness levels (0-255)
typedef enum {
    LED_BRIGHTNESS_OFF = 0,
    LED_BRIGHTNESS_LOW = 32,
    LED_BRIGHTNESS_MEDIUM = 128,
    LED_BRIGHTNESS_HIGH = 200,
    LED_BRIGHTNESS_MAX = 255
} LEDBrightness_t;

// LED Effect Types
typedef enum {
    LED_EFFECT_NONE = 0,
    LED_EFFECT_BLINK,
    LED_EFFECT_BREATHING,
    LED_EFFECT_RAINBOW,
    LED_EFFECT_PULSE
} LEDEffect_t;

// LED Status for System States
typedef enum {
    LED_STATUS_INIT = 0,        // System initializing
    LED_STATUS_READY,           // System ready
    LED_STATUS_RUNNING,         // System running
    LED_STATUS_ERROR,           // Error state
    LED_STATUS_WARNING,         // Warning state
    LED_STATUS_CALIBRATING,     // Calibration in progress
    LED_STATUS_COMMUNICATION,   // Communication active
    LED_STATUS_STANDBY          // Standby mode
} LEDStatus_t;

// LED Configuration Constants
#define LED_COUNT               1               // Number of LEDs
#define LED_DEFAULT_BRIGHTNESS  LED_BRIGHTNESS_MEDIUM
#define LED_RAINBOW_SPEED       50              // Default rainbow speed (ms)
#define LED_BLINK_ON_TIME       500             // Default blink on time (ms)
#define LED_BLINK_OFF_TIME      500             // Default blink off time (ms)
#define LED_BREATHING_PERIOD    2000            // Default breathing period (ms)

// Global LED object
extern Adafruit_NeoPixel LED_STATUS;

// LED Initialization
void initLED();

// Basic LED Control Functions
void setLEDColor(LEDColor_t color, LEDBrightness_t brightness = LED_DEFAULT_BRIGHTNESS);
void setLEDRGB(uint8_t red, uint8_t green, uint8_t blue, LEDBrightness_t brightness = LED_DEFAULT_BRIGHTNESS);
void setLEDBrightness(LEDBrightness_t brightness);
void turnOffLED();

// LED Effect Functions
void blinkLED(LEDColor_t color, uint16_t on_time_ms = LED_BLINK_ON_TIME, uint16_t off_time_ms = LED_BLINK_OFF_TIME, uint8_t blink_count = 3);
void breathingLED(LEDColor_t color, uint16_t period_ms = LED_BREATHING_PERIOD);
void pulseLED(LEDColor_t color, uint16_t pulse_time_ms = 1000);
void rainbowLED(uint16_t speed_ms = LED_RAINBOW_SPEED);
void stopLEDEffect();

// System Status LED Functions
void setLEDStatus(LEDStatus_t status);
void updateLEDStatus(); // Call this in main loop for effects

// Utility Functions
uint32_t getLEDColorValue(LEDColor_t color);
void setLEDColorValue(uint32_t color_value, LEDBrightness_t brightness = LED_DEFAULT_BRIGHTNESS);
bool isLEDOn();

// Advanced Functions
void fadeLED(LEDColor_t from_color, LEDColor_t to_color, uint16_t duration_ms);
void strobeeLED(LEDColor_t color, uint16_t strobe_time_ms = 100, uint8_t strobe_count = 5);
void sequenceLED(LEDColor_t colors[], uint8_t color_count, uint16_t step_time_ms = 500, bool repeat = false);

#endif // LED_H