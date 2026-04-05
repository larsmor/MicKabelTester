#pragma once
#include <cstdint>

struct TdrResult {
    bool   fault_found;
    bool   is_short;
    int    reflect_index;
    float  distance_m;
};

struct TdrConfig {
    float clkdiv;            // typisk 1.0 → ~125 MHz
    float velocity_factor;   // 0.66 for XLR
};

void        tdr_init(const TdrConfig &cfg);
TdrResult   tdr_measure();
const uint8_t* tdr_get_samples(int &n);
float       tdr_get_sample_period_ns();
void        tdr_set_velocity_factor(float vf);
float       tdr_get_velocity_factor();
