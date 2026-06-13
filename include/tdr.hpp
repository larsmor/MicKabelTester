#pragma once
#include <cstdint>

struct TdrConfig {
    float velocity_factor;
    float clkdiv;
};

// diag ved manglende signal (se tdr_run_hw_diag)
enum TdrDiag : uint8_t {
    TDR_DIAG_NONE         = 0,
    TDR_DIAG_GP23_OK      = 1,  // GP3 følger GP2 → print OK, kabel/stik TDR
    TDR_DIAG_GP23_OPEN    = 2,  // GP3 reagerer ikke på GP2
    TDR_DIAG_GP3_STUCK    = 3,
    TDR_DIAG_GROUND_FAULT = 4,  // stik 4-5 eller 4-6 (GND til puls/sense, ikke 5-6)
};

enum class TdrCalibType : uint8_t {
    Open    = 0,
    Short   = 1,
    Load100 = 2,
};

struct TdrResult {
    bool  fault_found;
    bool  is_short;
    bool  unstable;      // fx. fingre på kabel (ustabil kapacitet)
    bool  weak_signal;   // svag/uklar refleksion (ikke tom stik)
    bool  no_cable;      // ingen tydelig refleksion
    bool  consensus_strong; // 4+ OPEN / 4+ no_cable / 5+ SHORT (stabil visning)
    uint8_t diag;
    int   launch_index;  // sendepuls / launch-artefakt
    int   reflect_index; // kabelende (absolut sample)
    float distance_m;
    uint8_t cal_edges;     // kanter i sidste waveform (kalibrering debug)
    int8_t  cal_best_delta; // bedste delta eller -1 (kalibrering debug)
    uint8_t vote_median_delta; // stabil: median refleksions-delta (debug OLED)
    uint8_t vote_strong_open;  // stabil: stærke OPEN-shots (debug OLED)
    uint8_t vote_short;        // stabil: SHORT-shots uden gp3_follow (debug OLED)
    uint8_t vote_pulse_width;  // stabil: median GP3-pulsbredde (UI hold)
};

struct TdrCalibration {
    bool ok;
    bool no_cable;
    bool short_fault;
    bool open_fault;
    int  offset_idx;
};

struct TdrCalibState {
    int8_t  short_zero_delta;
    int8_t  load100_delta;
    bool    short_valid;
    bool    load_valid;
};

void tdr_init(const TdrConfig &cfg);
void tdr_deinit();

float tdr_get_sample_period_ns();
void  tdr_set_velocity_factor(float vf);
float tdr_get_velocity_factor();

void tdr_set_calibration(const TdrCalibState &cal);
void tdr_get_calibration(TdrCalibState &cal);

TdrResult tdr_measure();
TdrResult tdr_measure_filtered();
TdrResult tdr_measure_stable();
TdrResult tdr_measure_for_calibrate(TdrCalibType type = TdrCalibType::Open);
bool      tdr_calibrate_measurement_ok(const TdrResult &r);
bool      tdr_calibrate_vf_allowed(const TdrResult &r);
bool      tdr_apply_calibrate_vf(float L_ref_m, TdrResult &r);
bool      tdr_calibrate_short_ok(const TdrResult &r);
bool      tdr_apply_calibrate_short(const TdrResult &r, int *out_delta);
bool      tdr_calibrate_load100_ok(const TdrResult &r);
bool      tdr_apply_calibrate_load100(const TdrResult &r, int *out_delta);
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
void tdr_dbg_print_calibrate(const TdrResult &r, bool calib_ok, float ref_m);
#endif
TdrResult tdr_measure_autogain();

TdrCalibration tdr_calibrate();

const uint8_t* tdr_get_samples(int &n);
const uint8_t* tdr_get_filtered(int &n);
