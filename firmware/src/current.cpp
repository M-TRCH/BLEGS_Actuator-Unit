#include "current.h"
#include "system.h"
#include <math.h>

// Global variables
float current_ia = 0.0f;                // Phase A current in Amperes
float current_ic = 0.0f;                // Phase C current in Amperes
float current_ib = 0.0f;                // Phase B current in Amperes (calculated)
float current_dc_estimated = 0.0f;      // Estimated DC current in Amperes

// Filtered values for smoother readings
static float current_ia_filtered = 0.0f;
static float current_ic_filtered = 0.0f;
static float current_ib_filtered = 0.0f;

void currentInit() {
    // Configure ADC pins for current sensing
    pinMode(SEN_IA_PIN, INPUT_ANALOG);
    pinMode(SEN_IC_PIN, INPUT_ANALOG);
    
    // Initialize variables
    current_ia = 0.0f;
    current_ic = 0.0f;
    current_ib = 0.0f;
    current_dc_estimated = 0.0f;
    current_ia_filtered = 0.0f;
    current_ic_filtered = 0.0f;
    current_ib_filtered = 0.0f;
    
    // Allow ADC to stabilize
    delay(10);
}

uint16_t currentReadRaw(uint8_t pin) {
    return analogRead(pin);
}

float currentAdcToVoltage(uint16_t adc_value) {
    return (float)adc_value * ADC_TO_VOLTAGE;
}

float currentVoltageToCurrent(float voltage) {
    // INA240A1 output: Vout = (I_sense × R_shunt × Gain) + Vref/2
    // Solving for I_sense: I_sense = (Vout - Vref/2) / (R_shunt × Gain)
    float voltage_diff = voltage - CURRENT_OFFSET_VOLTAGE;
    return voltage_diff * VOLTAGE_TO_CURRENT;
}

float currentReadPhase(uint8_t pin) {
    uint16_t adc_value = currentReadRaw(pin);
    float voltage = currentAdcToVoltage(adc_value);
    return currentVoltageToCurrent(voltage);
}

void currentUpdate() {
    // Read phase A and C currents
    current_ia = currentReadPhase(SEN_IA_PIN);
    current_ic = currentReadPhase(SEN_IC_PIN);
    
    // Apply low-pass filter
    current_ia_filtered = currentFilter(current_ia, current_ia_filtered);
    current_ic_filtered = currentFilter(current_ic, current_ic_filtered);
    
    // Calculate phase B current using Kirchhoff's current law
    // In a 3-phase system: Ia + Ib + Ic = 0
    // Therefore: Ib = -(Ia + Ic)
    current_ib = -(current_ia_filtered + current_ic_filtered);
    current_ib_filtered = currentFilter(current_ib, current_ib_filtered);
    
    // Update global filtered values
    current_ia = current_ia_filtered;
    current_ic = current_ic_filtered;
    current_ib = current_ib_filtered;
}

float currentFilter(float new_value, float filtered_value) {
    // Simple exponential moving average (low-pass filter)
    // filtered = alpha * new + (1 - alpha) * old
    return CURRENT_FILTER_ALPHA * new_value + (1.0f - CURRENT_FILTER_ALPHA) * filtered_value;
}

float currentEstimateDC() {
    // Estimate DC current from instantaneous phase currents
    // For a BLDC motor, the DC current can be approximated using the RMS of phase currents
    // or by summing the absolute values of active phases
    
    // Method 1: RMS approach (more accurate for sinusoidal currents)
    // I_dc ≈ sqrt(Ia² + Ib² + Ic²) / sqrt(3/2)
    float sum_of_squares = current_ia * current_ia + 
                           current_ib * current_ib + 
                           current_ic * current_ic;
    float i_rms = sqrt(sum_of_squares / 3.0f);
    
    // Method 2: Absolute value approach (simpler, works well for BLDC)
    // I_dc ≈ (|Ia| + |Ib| + |Ic|) / 2
    float i_abs = (fabs(current_ia) + fabs(current_ib) + fabs(current_ic)) / 2.0f;
    
    // Use the absolute value method as it's simpler and more representative
    // of what a DC ammeter would show on the power supply
    return i_abs;
}

float currentGetDCAverage(uint16_t samples) {
    float sum = 0.0f;
    
    // Take multiple samples and average them
    for (uint16_t i = 0; i < samples; i++) {
        currentUpdate();
        sum += currentEstimateDC();
        
        // Small delay between samples to get different points in the PWM cycle
        delayMicroseconds(50);
    }
    
    current_dc_estimated = sum / (float)samples;
    return current_dc_estimated;
}

void currentPrintDebug() {
    SystemSerial->print("Ia: ");
    SystemSerial->print(current_ia, 3);
    SystemSerial->print(" A, Ib: ");
    SystemSerial->print(current_ib, 3);
    SystemSerial->print(" A, Ic: ");
    SystemSerial->print(current_ic, 3);
    SystemSerial->print(" A, DC_est: ");
    SystemSerial->print(current_dc_estimated, 3);
    SystemSerial->println(" A");
}
