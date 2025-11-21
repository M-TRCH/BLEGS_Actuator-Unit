#include "led.h"
#include "system.h"

// Global LED object
Adafruit_NeoPixel LED_STATUS(LED_COUNT, LED_STATUS_PIN, NEO_GRB + NEO_KHZ800);

// LED Color Palette (16 preset colors)
static const uint32_t LED_COLOR_PALETTE[16] = {
    0x000000,  // OFF - Black
    0xFF0000,  // RED
    0x00FF00,  // GREEN  
    0x0000FF,  // BLUE
    0xFFFF00,  // YELLOW
    0x00FFFF,  // CYAN
    0xFF00FF,  // MAGENTA
    0xFFFFFF,  // WHITE
    0xFF8000,  // ORANGE
    0xFF69B4,  // PINK
    0x32CD32,  // LIME
    0x800080,  // PURPLE
    0x000080,  // NAVY
    0x800000,  // MAROON
    0x808000,  // OLIVE
    0x008080   // TEAL
};

// Effect control variables
static LEDEffect_t current_effect = LED_EFFECT_NONE;
static uint32_t effect_start_time = 0;
static uint32_t last_effect_update = 0;
static uint16_t effect_parameter1 = 0;
static uint16_t effect_parameter2 = 0;
static uint8_t effect_parameter3 = 0;
static LEDColor_t effect_color = LED_COLOR_OFF;
static bool effect_active = false;

// Current LED state
static LEDColor_t current_color = LED_COLOR_OFF;
static LEDBrightness_t current_brightness = LED_DEFAULT_BRIGHTNESS;

/**
 * @brief Initialize LED system
 */
void initLED() 
{
    LED_STATUS.begin();
    LED_STATUS.setBrightness(LED_DEFAULT_BRIGHTNESS);
    LED_STATUS.clear();
    LED_STATUS.show();
    
    current_color = LED_COLOR_OFF;
    current_brightness = LED_DEFAULT_BRIGHTNESS;
    current_effect = LED_EFFECT_NONE;
    
    SystemSerial->println("LED system initialized");
}

/**
 * @brief Set LED to specific preset color with brightness
 * @param color       LED color from LEDColor_t enum
 * @param brightness  LED brightness from LEDBrightness_t enum
 */
void setLEDColor(LEDColor_t color, LEDBrightness_t brightness) 
{
    if (color >= 16) {
        color = LED_COLOR_OFF; // Safety check
    }
    
    stopLEDEffect(); // Stop any active effects
    
    uint32_t color_value = LED_COLOR_PALETTE[color];
    
    // Extract RGB components
    uint8_t red = (color_value >> 16) & 0xFF;
    uint8_t green = (color_value >> 8) & 0xFF;
    uint8_t blue = color_value & 0xFF;
    
    // Apply brightness scaling
    red = (red * brightness) / 255;
    green = (green * brightness) / 255;
    blue = (blue * brightness) / 255;
    
    // Set LED color
    LED_STATUS.setPixelColor(0, LED_STATUS.Color(red, green, blue));
    LED_STATUS.show();
    
    current_color = color;
    current_brightness = brightness;
}

/**
 * @brief Set LED to custom RGB color with brightness
 * @param red         Red component (0-255)
 * @param green       Green component (0-255)  
 * @param blue        Blue component (0-255)
 * @param brightness  LED brightness from LEDBrightness_t enum
 */
void setLEDRGB(uint8_t red, uint8_t green, uint8_t blue, LEDBrightness_t brightness) 
{
    stopLEDEffect(); // Stop any active effects
    
    // Apply brightness scaling
    red = (red * brightness) / 255;
    green = (green * brightness) / 255;
    blue = (blue * brightness) / 255;
    
    LED_STATUS.setPixelColor(0, LED_STATUS.Color(red, green, blue));
    LED_STATUS.show();
    
    current_color = LED_COLOR_OFF; // Custom color, not from preset
    current_brightness = brightness;
}

/**
 * @brief Set LED brightness without changing color
 * @param brightness  LED brightness from LEDBrightness_t enum
 */
void setLEDBrightness(LEDBrightness_t brightness) 
{
    LED_STATUS.setBrightness(brightness);
    LED_STATUS.show();
    current_brightness = brightness;
}

/**
 * @brief Turn off LED
 */
void turnOffLED() 
{
    stopLEDEffect();
    LED_STATUS.setPixelColor(0, LED_STATUS.Color(0, 0, 0));
    LED_STATUS.show();
    current_color = LED_COLOR_OFF;
}

/**
 * @brief Start blinking LED effect
 * @param color         LED color from LEDColor_t enum
 * @param on_time_ms    Time LED is on (milliseconds)
 * @param off_time_ms   Time LED is off (milliseconds)  
 * @param blink_count   Number of blinks (0 = infinite)
 */
void blinkLED(LEDColor_t color, uint16_t on_time_ms, uint16_t off_time_ms, uint8_t blink_count) 
{
    current_effect = LED_EFFECT_BLINK;
    effect_color = color;
    effect_parameter1 = on_time_ms;
    effect_parameter2 = off_time_ms;
    effect_parameter3 = blink_count;
    effect_start_time = millis();
    last_effect_update = millis();
    effect_active = true;
}

/**
 * @brief Start breathing LED effect
 * @param color       LED color from LEDColor_t enum
 * @param period_ms   Complete breathing cycle time (milliseconds)
 */
void breathingLED(LEDColor_t color, uint16_t period_ms) 
{
    current_effect = LED_EFFECT_BREATHING;
    effect_color = color;
    effect_parameter1 = period_ms;
    effect_start_time = millis();
    effect_active = true;
}

/**
 * @brief Start pulse LED effect (single pulse)
 * @param color           LED color from LEDColor_t enum
 * @param pulse_time_ms   Pulse duration (milliseconds)
 */
void pulseLED(LEDColor_t color, uint16_t pulse_time_ms) 
{
    current_effect = LED_EFFECT_PULSE;
    effect_color = color;
    effect_parameter1 = pulse_time_ms;
    effect_start_time = millis();
    effect_active = true;
}

/**
 * @brief Start rainbow LED effect
 * @param speed_ms    Speed of color change (milliseconds per step)
 */
void rainbowLED(uint16_t speed_ms) 
{
    current_effect = LED_EFFECT_RAINBOW;
    effect_parameter1 = speed_ms;
    effect_start_time = millis();
    last_effect_update = millis();
    effect_active = true;
}

/**
 * @brief Stop current LED effect
 */
void stopLEDEffect() 
{
    current_effect = LED_EFFECT_NONE;
    effect_active = false;
}

/**
 * @brief Update LED effects (call this in main loop)
 */
void updateLEDStatus() 
{
    if (!effect_active) return;
    
    uint32_t current_time = millis();
    uint32_t elapsed = current_time - effect_start_time;
    
    switch (current_effect) {
        case LED_EFFECT_BLINK:
        {
            static uint8_t blink_state = 1; // 1 = on, 0 = off
            static uint8_t blink_counter = 0;
            
            if (blink_state == 1 && (current_time - last_effect_update >= effect_parameter1)) {
                // Turn off
                turnOffLED();
                blink_state = 0;
                last_effect_update = current_time;
                blink_counter++;
            } else if (blink_state == 0 && (current_time - last_effect_update >= effect_parameter2)) {
                // Check if we should continue blinking
                if (effect_parameter3 == 0 || blink_counter < effect_parameter3) {
                    // Turn on
                    setLEDColor(effect_color, current_brightness);
                    blink_state = 1;
                    last_effect_update = current_time;
                } else {
                    // Finished blinking
                    stopLEDEffect();
                    blink_counter = 0;
                }
            }
            break;
        }
        
        case LED_EFFECT_BREATHING:
        {
            float phase = (float)(elapsed % effect_parameter1) / effect_parameter1;
            float brightness_factor = (sin(phase * 2 * PI) + 1.0f) / 2.0f;
            uint8_t brightness = (uint8_t)(brightness_factor * current_brightness);
            
            uint32_t color_value = LED_COLOR_PALETTE[effect_color];
            uint8_t red = ((color_value >> 16) & 0xFF) * brightness / 255;
            uint8_t green = ((color_value >> 8) & 0xFF) * brightness / 255;
            uint8_t blue = (color_value & 0xFF) * brightness / 255;
            
            LED_STATUS.setPixelColor(0, LED_STATUS.Color(red, green, blue));
            LED_STATUS.show();
            break;
        }
        
        case LED_EFFECT_PULSE:
        {
            if (elapsed <= effect_parameter1) {
                float phase = (float)elapsed / effect_parameter1;
                float brightness_factor = sin(phase * PI); // Single pulse
                uint8_t brightness = (uint8_t)(brightness_factor * current_brightness);
                
                uint32_t color_value = LED_COLOR_PALETTE[effect_color];
                uint8_t red = ((color_value >> 16) & 0xFF) * brightness / 255;
                uint8_t green = ((color_value >> 8) & 0xFF) * brightness / 255;
                uint8_t blue = (color_value & 0xFF) * brightness / 255;
                
                LED_STATUS.setPixelColor(0, LED_STATUS.Color(red, green, blue));
                LED_STATUS.show();
            } else {
                // Pulse finished
                turnOffLED();
                stopLEDEffect();
            }
            break;
        }
        
        case LED_EFFECT_RAINBOW:
        {
            if (current_time - last_effect_update >= effect_parameter1) {
                static uint16_t hue = 0;
                
                uint32_t color = LED_STATUS.gamma32(LED_STATUS.ColorHSV(hue, 255, current_brightness));
                LED_STATUS.setPixelColor(0, color);
                LED_STATUS.show();
                
                hue += 512; // Increment hue for next color
                if (hue >= 65536) hue = 0;
                
                last_effect_update = current_time;
            }
            break;
        }
        
        default:
            break;
    }
}

/**
 * @brief Set LED based on system status
 * @param status System status from LEDStatus_t enum
 */
void setLEDStatus(LEDStatus_t status) 
{
    switch (status) {
        case LED_STATUS_INIT:
            setLEDColor(LED_COLOR_RED, LED_BRIGHTNESS_LOW);
            break;
            
        case LED_STATUS_READY:
            setLEDColor(LED_COLOR_GREEN, LED_BRIGHTNESS_MEDIUM);
            break;
            
        case LED_STATUS_RUNNING:
            setLEDColor(LED_COLOR_BLUE, LED_BRIGHTNESS_LOW);
            break;
            
        case LED_STATUS_ERROR:
            blinkLED(LED_COLOR_RED, 200, 200, 0); // Infinite blink
            break;
            
        case LED_STATUS_WARNING:
            setLEDColor(LED_COLOR_ORANGE, LED_BRIGHTNESS_HIGH);
            break;
            
        case LED_STATUS_CALIBRATING:
            breathingLED(LED_COLOR_YELLOW, 1500);
            break;
            
        case LED_STATUS_COMMUNICATION:
            blinkLED(LED_COLOR_CYAN, 100, 100, 5);
            break;
            
        case LED_STATUS_STANDBY:
            breathingLED(LED_COLOR_GREEN, 3000);
            break;
            
        default:
            setLEDColor(LED_COLOR_PURPLE, LED_BRIGHTNESS_LOW);
            break;
    }
}

/**
 * @brief Get color value from LEDColor_t enum
 * @param color   LED color from LEDColor_t enum
 * @return        32-bit color value (0x00RRGGBB)
 */
uint32_t getLEDColorValue(LEDColor_t color) 
{
    if (color >= 16) {
        return LED_COLOR_PALETTE[LED_COLOR_OFF];
    }
    return LED_COLOR_PALETTE[color];
}

/**
 * @brief Set LED using raw color value
 * @param color_value 32-bit color value (0x00RRGGBB)
 * @param brightness  LED brightness from LEDBrightness_t enum
 */
void setLEDColorValue(uint32_t color_value, LEDBrightness_t brightness) 
{
    uint8_t red = (color_value >> 16) & 0xFF;
    uint8_t green = (color_value >> 8) & 0xFF;
    uint8_t blue = color_value & 0xFF;
    
    setLEDRGB(red, green, blue, brightness);
}

/**
 * @brief Check if LED is currently on
 * @return true if LED is on, false if off
 */
bool isLEDOn() 
{
    return (current_color != LED_COLOR_OFF) || effect_active;
}

/**
 * @brief Fade LED from one color to another
 * @param from_color    Starting color
 * @param to_color      Ending color  
 * @param duration_ms   Fade duration in milliseconds
 */
void fadeLED(LEDColor_t from_color, LEDColor_t to_color, uint16_t duration_ms) 
{
    uint32_t from_value = getLEDColorValue(from_color);
    uint32_t to_value = getLEDColorValue(to_color);
    
    uint8_t from_r = (from_value >> 16) & 0xFF;
    uint8_t from_g = (from_value >> 8) & 0xFF;
    uint8_t from_b = from_value & 0xFF;
    
    uint8_t to_r = (to_value >> 16) & 0xFF;
    uint8_t to_g = (to_value >> 8) & 0xFF;
    uint8_t to_b = to_value & 0xFF;
    
    uint32_t start_time = millis();
    uint32_t current_time;
    
    do {
        current_time = millis();
        float progress = (float)(current_time - start_time) / duration_ms;
        if (progress > 1.0f) progress = 1.0f;
        
        uint8_t r = from_r + (uint8_t)((to_r - from_r) * progress);
        uint8_t g = from_g + (uint8_t)((to_g - from_g) * progress);
        uint8_t b = from_b + (uint8_t)((to_b - from_b) * progress);
        
        setLEDRGB(r, g, b, current_brightness);
        delay(10); // Small delay for smooth transition
        
    } while (current_time - start_time < duration_ms);
}

/**
 * @brief Strobe LED effect
 * @param color           LED color
 * @param strobe_time_ms  Strobe flash time
 * @param strobe_count    Number of strobe flashes
 */
void strobeeLED(LEDColor_t color, uint16_t strobe_time_ms, uint8_t strobe_count) 
{
    for (uint8_t i = 0; i < strobe_count; i++) {
        setLEDColor(color, LED_BRIGHTNESS_MAX);
        delay(strobe_time_ms);
        turnOffLED();
        if (i < strobe_count - 1) {
            delay(strobe_time_ms);
        }
    }
}

/**
 * @brief Sequence LED through multiple colors
 * @param colors        Array of colors to sequence through
 * @param color_count   Number of colors in array
 * @param step_time_ms  Time for each color step
 * @param repeat        Whether to repeat the sequence
 */
void sequenceLED(LEDColor_t colors[], uint8_t color_count, uint16_t step_time_ms, bool repeat) 
{
    do {
        for (uint8_t i = 0; i < color_count; i++) {
            setLEDColor(colors[i], current_brightness);
            delay(step_time_ms);
        }
    } while (repeat);
}