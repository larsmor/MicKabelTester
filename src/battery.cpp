#include "battery.hpp"
#include "pico/stdlib.h"
#include "hardware/adc.h"

// Juster disse hvis din spændingsdeler er anderledes
static const float VREF      = 3.3f;
static const float R1        = 200000.0f;  // top resistor
static const float R2        = 100000.0f;  // bottom resistor
static const float FULL_V    = 4.10f;
static const float EMPTY_V   = 3.20f;

int battery_get_percent()
{
    adc_select_input(1); // ADC0 = GPIO26
    uint16_t raw = adc_read();

    float v_adc = (raw * VREF) / 4095.0f;
    float v_bat = v_adc * ((R1 + R2) / R2);

    // Clamp
    if (v_bat > FULL_V)  v_bat = FULL_V;
    if (v_bat < EMPTY_V) v_bat = EMPTY_V;

    int pct = (int)(100.0f * (v_bat - EMPTY_V) / (FULL_V - EMPTY_V));

    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    return pct;
}
