#pragma once
#include <cstdint>

struct TdrConfig {
    float clkdiv;
    float velocity_factor;
};

struct TdrResult {
    bool  fault_found;
    bool  is_short;
    int   reflect_index;
    float distance_m;
};

void tdr_init(const TdrConfig &cfg);
void tdr_deinit();

TdrResult tdr_measure();             // rå måling (uændret)
TdrResult tdr_measure_filtered();    // med filtering
TdrResult tdr_measure_autogain();    // multi-sample "auto-gain"

float     tdr_get_sample_period_ns();

void  tdr_set_velocity_factor(float vf);
float tdr_get_velocity_factor();

const uint8_t* tdr_get_samples(int &n);
