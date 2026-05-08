#pragma once
#include <cstdint>

struct MicResult {
    bool pin_ok[3];
    bool short_detected;
    bool mic_present;
    float adc_voltage;
};

void mic_init();
void mic_deinit();

MicResult mic_measure_auto();
