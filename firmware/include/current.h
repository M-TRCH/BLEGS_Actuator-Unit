#ifndef CURRENT_H
#define CURRENT_H

#include <Arduino.h>

// INA240A1 Current Sensor Configuration
#define INA240A1_GAIN               20.0f           // INA240A1 gain in V/V
#define SHUNT_RESISTANCE            0.001f          // Shunt resistor value in Ohms (1mOhm)
#define ADC_RESOLUTION              12              // ADC resolution in bits
#define ADC_MAX_VALUE               4095            // Maximum ADC value (2^12 - 1)
#define ADC_REFERENCE_VOLTAGE       3.3f            // ADC reference voltage in Volts
#define CURRENT_OFFSET_VOLTAGE      1.65f           // Zero current output voltage (Vref/2)

// Current measurement calculation constants
// Voltage per ADC step = ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE
// Current per ADC step = (Voltage per step) / (INA240A1_GAIN * SHUNT_RESISTANCE)
#define ADC_TO_VOLTAGE              (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE)
#define VOLTAGE_TO_CURRENT          (1.0f / (INA240A1_GAIN * SHUNT_RESISTANCE))
#define ADC_TO_CURRENT              (ADC_TO_VOLTAGE * VOLTAGE_TO_CURRENT)

// Filtering configuration
#define CURRENT_FILTER_ALPHA        0.1f            // Low-pass filter coefficient (0.0 to 1.0)
#define DC_CURRENT_SAMPLES          100             // Number of samples for DC current averaging

// Global variables
extern float current_ia;                            // Phase A current in Amperes
extern float current_ic;                            // Phase C current in Amperes
extern float current_ib;                            // Phase B current in Amperes (calculated)
extern float current_dc_estimated;                  // Estimated DC current in Amperes

/**
 * @brief Initialize current sensing
 * 
 * Configures ADC pins and initializes current measurement variables
 */
void currentInit();

/**
 * @brief Read raw ADC value from current sensor
 * 
 * @param pin ADC pin to read from (SEN_IA_PIN or SEN_IC_PIN)
 * @return Raw ADC value (0-4095)
 */
uint16_t currentReadRaw(uint8_t pin);

/**
 * @brief Convert ADC value to voltage
 * 
 * @param adc_value Raw ADC value
 * @return Voltage in Volts
 */
float currentAdcToVoltage(uint16_t adc_value);

/**
 * @brief Convert voltage to current
 * 
 * @param voltage Voltage from INA240A1 output
 * @return Current in Amperes
 */
float currentVoltageToCurrent(float voltage);

/**
 * @brief Read current from a specific phase
 * 
 * @param pin ADC pin to read from (SEN_IA_PIN or SEN_IC_PIN)
 * @return Current in Amperes
 */
float currentReadPhase(uint8_t pin);

/**
 * @brief Update all phase currents
 * 
 * Reads phase A and C currents, calculates phase B current using Kirchhoff's law
 */
void currentUpdate();

/**
 * @brief Apply low-pass filter to current reading
 * 
 * @param new_value New current reading
 * @param filtered_value Previous filtered value
 * @return Filtered current value
 */
float currentFilter(float new_value, float filtered_value);

/**
 * @brief Estimate DC bus current from phase currents
 * 
 * Calculates the approximate DC current that would be measured at the power supply
 * by analyzing the instantaneous phase currents
 * 
 * @return Estimated DC current in Amperes
 */
float currentEstimateDC();

/**
 * @brief Get the estimated DC current with averaging
 * 
 * Performs multiple samples and averages them for a more stable reading
 * similar to what you would see on a power supply meter
 * 
 * @param samples Number of samples to average (default: DC_CURRENT_SAMPLES)
 * @return Averaged DC current in Amperes
 */
float currentGetDCAverage(uint16_t samples = DC_CURRENT_SAMPLES);

/**
 * @brief Print current measurements to serial
 * 
 * Outputs phase currents and estimated DC current for debugging
 */
void currentPrintDebug();

#endif // CURRENT_H
