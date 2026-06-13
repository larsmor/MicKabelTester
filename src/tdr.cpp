#include "pico/stdlib.h"
#include "tdr.hpp"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include <cmath>
#include <cstring>
#include <cstdint>
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
#include <cstdio>
#endif

// TDR debug: USB serial @ 115200 baud, one line per tdr_measure_stable (~UI cycle).
// Enable: uncomment #define below, OR cmake -DTDR_DEBUG=ON, then flash and open TDR view.
// #define TDR_DEBUG 1
// Calibrate debug: one line per calibrate measure — cmake -DCALIB_DEBUG=ON or -DTDR_DEBUG=ON.
#ifdef TDR_DEBUG
#define TDR_DBG(...) printf(__VA_ARGS__)
#else
#define TDR_DBG(...) ((void)0)
#endif

#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
struct TdrStableDebugInfo {
    bool  hw_short;
    float max_open_zone_dist_m;
    int   pulse_width_median;
    const char *rule;
    bool  fix_applied;
    float vf_calc;
    float vf_new;
};

static TdrStableDebugInfo g_stable_dbg{};

static void tdr_dbg_print_stable(const TdrResult &r);
static const char *tdr_calib_fail_reason(const TdrResult &r, bool calib_ok);
#endif

// ------------------------------------------------------------
// Hardware-konfiguration
// ------------------------------------------------------------
static const uint TDR_OUT_PIN = 2;
static const uint TDR_IN_PIN  = 3;
static const uint TDR_GROUND_SWITCH_PIN = 14;

static PIO  g_pio = pio0;
static uint g_sm  = 0;
static uint g_offset;

static float g_velocity_factor = 0.66f;
static float g_clkdiv          = 1.0f;
static float g_calib_zmax_m    = -1.0f;
static int8_t  g_cal_short_zero = -1;
static int8_t  g_cal_load_delta = -1;
static bool    g_cal_short_valid = false;
static bool    g_cal_load_valid  = false;

struct TdrCalibUnstableCtx {
    int  spread;
    int  nfound;
    int  n_agree;
    bool pulse_shape_ok;
    bool width_shape_unstable;
    bool amp_shape_unstable;
    bool delta_unstable;
};

static TdrCalibUnstableCtx g_calib_unst{};
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
static const char *g_calib_unst_reason = nullptr;
#endif
// Faktisk GPIO-sampleperiode (kan afvige fra PIO clkdiv pga. minimum delay)
static float g_capture_sample_period_ns = 8.0f;

static uint8_t  g_samples[128];
static uint8_t  g_filtered[128];
static uint8_t  g_baseline_samples[128];
static uint32_t g_sample_time_ns[128];
static bool     g_tdr_active = false;
static bool     g_invert_in  = false;
static bool     g_baseline_valid = false;

// GPIO-capture pulser GP2 ved sample TDR_PULSE_ON (matcher tdr_capture_gpio_pull)
static const int TDR_PULSE_ON              = 10;
// Lang puls: ~3 m åben refleksion ligger ~sample 14 — kort OFF ved 14 skjulte kanten
static const int TDR_PULSE_OFF             = 24;
static const int TDR_CALIBRATE_PULSE_OFF   = TDR_PULSE_OFF;

static int      g_capture_pulse_off = TDR_PULSE_OFF;
static const int TDR_CALIBRATE_OPEN_DELTA_HI = 12;
static const int TDR_LAUNCH_SKIP      = 2;
static const int TDR_LAUNCH_SEARCH_LO = 8;
static const int TDR_LAUNCH_SEARCH_HI = 15;
static const int TDR_MIN_END_GAP      = 1;
static const int TDR_MAX_END_DELTA    = 8;   // ~2–5 m ved ~10 ns/sample (undgå hale-støj)
static const int TDR_SHORT_DELTA_MAX  = 1;   // delta 0-1 ved stik (GP3 følger GP2)
static const int TDR_STABLE_SHORT_MIN_AGREE = 5; // 5/6 shots for SHORT (undgå falsk 0.0 m)
static const int TDR_STABLE_SHORT_BLOCK_OPEN = 4; // OPEN-majoritet kun hvis færre end 4 strict SHORT
static const int TDR_STABLE_STRONG_OPEN_MAX_FOR_SHORT = 2; // SHORT kun hvis færre end 2 stærk OPEN
static const int TDR_STABLE_NO_CABLE_MIN_AGREE = 4; // 4/6 for tom stik
static const int TDR_MEDIAN_OPEN_BLOCK_SHORT_LO = 2; // aldrig SHORT hvis median delta 2-8
static const int TDR_TYPICAL_3M_MEDIAN_LO       = 3; // typisk ~3 m åben kabel
static const int TDR_TYPICAL_3M_MEDIAN_HI       = 6;
static const int TDR_SHORT_MAX_SHOT_DELTA       = 2; // SHORT kun hvis alle shots delta <= 2
static const int TDR_STRONG_OPEN_DELTA_LO       = 3; // ~3 m OPEN; svag delta-4 artefakt på kort
static const int TDR_QUIET_REFL_LO              = 1; // GP3 lav launch+N..N+1
static const int TDR_QUIET_REFL_HI              = 2;
static const int TDR_QUIET_EDGE_LO              = 3; // stigende kant launch+N+3..N+5
static const int TDR_QUIET_EDGE_HI              = 5;
static const int TDR_OPEN_DELTA_LO    = 1;   // svag TDR: 1-8 samples (hurtig capture)
static const int TDR_DIST_MIN_DELTA   = 2;   // delta 1 = kobling — ikke vis afstand
static const int TDR_TYPICAL_3M_DELTA_LO = 3; // ~3 m åben: sample 13-15 ved launch 10
static const int TDR_TYPICAL_3M_DELTA_HI = 6;
static const int TDR_OPEN_DELTA_HI    = 8;
static const int TDR_FAST_CAPTURE_END = 48;  // tæt sampling omkring puls/refleksion
static const int TDR_FAST_EXTRA_CYCLES = 0;
static const int TDR_SAMPLE_LOOP_OVERHEAD_CYCLES = 6;
static const float TDR_MAX_DISTANCE_M       = 5.0f;
static const float TDR_DIST_FORCE_OPEN_M      = 0.5f; // >0.5 m → aldrig SHORT (fx 3 m kabel)
static const float TDR_DIST_SHORT_MAX_M       = 0.2f; // fysisk stik-kortslutning ~0 m
static const float TDR_OPEN_DISPLAY_DIST_LO   = 1.5f; // typisk 3 m kabel — visningsbånd
static const float TDR_OPEN_DISPLAY_DIST_HI   = 4.5f;
static const float TDR_SHORT_BLOCK_ZONE_DIST_M = 2.0f; // zmax/d>=2 → ikke stik-SHORT
static const int   TDR_OPEN_CONSENSUS_MIN_D3  = 3;   // >=3 shots delta>=3 for OPEN
static const int   TDR_RELAXED_OPEN_DELTA_HI  = 10;  // launch+N søgning (~3 m åben)
static const int   TDR_LATE_PEAK_SAMPLE_LO    = 12;  // absolut peak når kun d=1
static const int   TDR_LATE_PEAK_SAMPLE_HI    = 18;
static const int   TDR_OPEN_FALLBACK_MIN_SHOTS = 3; // 3/6 fallback OPEN
static const int   TDR_OPEN_DELTA24_LO        = 2;  // konsensus delta 2-5
static const int   TDR_OPEN_DELTA24_HI        = 5;
static const float TDR_CALIB_DIST_LO_M        = 1.5f;
static const float TDR_CALIB_DIST_HI_M        = 5.0f;
static const int   TDR_OPEN_DELTA24_MIN_AGREE = 3;
static const int   TDR_RELAXED_OPEN_DELTA_LO  = 2;  // accepter delta 2-6 m. bred puls
static const int   TDR_RELAXED_OPEN_DELTA_MID = 6;
static const float TDR_FAST_PERIOD_MIN_NS   = 8.0f;
static const float TDR_FAST_PERIOD_MAX_NS   = 18.0f;  // 25 ns gav d=2→5 m ved 3 m kabel
static const int   TDR_BENCH_FAST_SAMPLES   = 4096;   // >512: bench_us har 1 µs opløsning
static const int   TDR_MIN_REFLECT_GRAD     = 2;   // enkelt-sample støj på flydende GP3
static const int   TDR_OPEN_MIN_RUN         = 2;   // refleksion skal holde niveau
static const int   TDR_OPEN_MIN_QUALITY     = 11;  // min. score for åben refleksion (ikke divider)
static const int   TDR_COUPLING_MAX_QUALITY = 10; // under dette + stabil GP3 = ingen kabel
static const int   TDR_CABLE_MIN_QUALITY    = 13;  // kabel: typisk stærkere end print-kobling
static const int   TDR_BASELINE_MIN_WIN_DIFF = 2;  // samples i OPEN-vindue der afviger fra GP2=0
static const int   TDR_STABLE_MIN_AGREE     = 4;   // min. ens shots for OPEN
static const int   TDR_STABLE_MAX_SPREAD    = 2;   // max delta-spredning for kabel (3 m typisk 0-1)
static const int   TDR_NO_CABLE_SPREAD      = 3;   // stor spredning = intet kabel / ustabil puls
static const int   TDR_OPEN_POP_MAX_SPREAD  = 2;   // max spredning i OPEN-vindue popcount
static const int   TDR_CABLE_MIN_PULSE_WIDTH    = 4;  // min. HI-run på GP3 mens GP2 høj (kabel)
static const int   TDR_WIDE_CABLE_PULSE_MIN     = 13; // ~3 m kabel: tidlig d=1 → sen peak launch+2
static const int   TDR_PULSE_WIDTH_STABLE_SPREAD = 2; // ens pulsform shot-til-shot
static const int   TDR_NO_CABLE_WIDTH_SPREAD    = 3;  // ustabil/kort puls-bredde = tom stik
static const int   TDR_OPEN_AMP_MAX_SPREAD      = 2;  // max HI-run i OPEN-vindue (stabilt kabel)
static const int   TDR_IDLE_STABLE_MAX_EDGES = 2;
static const int   TDR_DIAG_SETTLE_US        = 500;
static const int   TDR_DIAG_DRIVE_US         = 2500;
static const int   TDR_DIAG_SLOW_SETTLE_US   = 3000;
static const int   TDR_DIAG_SLOW_DRIVE_US    = 12000;
static const int   TDR_CALIBRATE_TRIES       = 20;
static const int   TDR_CALIBRATE_RAW_TRIES   = 10;
static const int   TDR_CALIBRATE_SLEEP_MS    = 10;
static const int   TDR_CALIBRATE_SETTLE_MS   = 40;
static const int   TDR_CALIBRATE_MAX_EDGES   = 18;

// ------------------------------------------------------------
// PIO-program
// ------------------------------------------------------------
static const uint16_t tdr_program_instructions[] = {
    0xE081, // set pins, 1
    0xE000, // set pins, 0
    0x4001, // in pins, 1
    0x0002  // jmp 2
};

static const pio_program_t tdr_program = {
    .instructions = tdr_program_instructions,
    .length       = 4,
    .origin       = -1
};

// ------------------------------------------------------------
// Init
// ------------------------------------------------------------
void tdr_init(const TdrConfig &cfg) {
    if (g_tdr_active)
        tdr_deinit();

    g_velocity_factor = cfg.velocity_factor;
    g_clkdiv          = cfg.clkdiv;
    {
        float sysclk = (float)clock_get_hz(clk_sys);
        g_capture_sample_period_ns = 1e9f / (sysclk / g_clkdiv);
    }

    gpio_init(TDR_GROUND_SWITCH_PIN);
    gpio_set_dir(TDR_GROUND_SWITCH_PIN, GPIO_OUT);
    gpio_put(TDR_GROUND_SWITCH_PIN, 1);

    pio_gpio_init(g_pio, TDR_OUT_PIN);
    pio_gpio_init(g_pio, TDR_IN_PIN);

    gpio_disable_pulls(TDR_IN_PIN);

    g_sm = pio_claim_unused_sm(g_pio, true);

    pio_sm_set_pindirs_with_mask(
        g_pio, g_sm,
        (1u << TDR_OUT_PIN),
        (1u << TDR_OUT_PIN)
    );
    pio_sm_set_pindirs_with_mask(
        g_pio, g_sm,
        0,
        (1u << TDR_IN_PIN)
    );

    g_offset = pio_add_program(g_pio, &tdr_program);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_set_pins(&c, TDR_OUT_PIN, 1);
    sm_config_set_in_pins(&c, TDR_IN_PIN);
    sm_config_set_in_shift(&c, false, true, 1);
    sm_config_set_clkdiv(&c, g_clkdiv);

    pio_sm_init(g_pio, g_sm, g_offset, &c);
    pio_sm_set_enabled(g_pio, g_sm, false);
    g_tdr_active = true;
}

// ------------------------------------------------------------
// Deinit
// ------------------------------------------------------------
void tdr_deinit() {
    if (!g_tdr_active)
        return;

    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_remove_program(g_pio, &tdr_program, g_offset);
    pio_sm_unclaim(g_pio, g_sm);

    gpio_put(TDR_GROUND_SWITCH_PIN, 0);

    gpio_init(TDR_OUT_PIN);
    gpio_init(TDR_IN_PIN);

    gpio_set_dir(TDR_OUT_PIN, GPIO_IN);
    gpio_set_dir(TDR_IN_PIN,  GPIO_IN);

    gpio_pull_down(TDR_OUT_PIN);
    gpio_pull_down(TDR_IN_PIN);
    g_tdr_active = false;
}

static bool tdr_samples_vary(const uint8_t *samples, int n) {
    for (int i = 1; i < n; i++) {
        if (samples[i] != samples[0])
            return true;
    }
    return false;
}

static int tdr_get_short_delta_max() {
    if (g_cal_short_valid && g_cal_short_zero >= 0)
        return g_cal_short_zero + 1;
    return TDR_SHORT_DELTA_MAX;
}

static bool tdr_short_cal_active(bool for_calibrate, TdrCalibType cal_type) {
    return for_calibrate && cal_type == TdrCalibType::Short;
}

static uint32_t tdr_zero_corrected_delta_ns(uint32_t delta_ns) {
    if (!g_cal_short_valid || g_cal_short_zero < 0)
        return delta_ns;
    uint32_t zero_ns =
        (uint32_t)((float)g_cal_short_zero * g_capture_sample_period_ns);
    if (delta_ns <= zero_ns)
        return 0;
    return delta_ns - zero_ns;
}

static int tdr_zero_corrected_delta_samples(int delta) {
    if (delta < 0)
        delta = 0;
    if (g_cal_short_valid && g_cal_short_zero >= 0) {
        delta -= g_cal_short_zero;
        if (delta < 0)
            delta = 0;
    }
    return delta;
}

static bool tdr_reflect_matches_load100(int delta) {
    if (!g_cal_load_valid || g_cal_load_delta < 0)
        return false;
    int diff = delta - g_cal_load_delta;
    if (diff < 0)
        diff = -diff;
    return diff <= 1;
}

static void tdr_fill_distance_ns(TdrResult &r, uint32_t delta_ns) {
    delta_ns = tdr_zero_corrected_delta_ns(delta_ns);
    float t = (float)delta_ns * 1e-9f;
    const float C = 299792458.0f;
    r.distance_m = (C * g_velocity_factor * t) / 2.0f;
    if (r.distance_m > TDR_MAX_DISTANCE_M)
        r.distance_m = TDR_MAX_DISTANCE_M;
}

static void tdr_fixup_open_distance_scale(TdrResult &r, int launch_i, int reflect_i);

static bool tdr_is_pulse_off_sample(int i);
static bool tdr_reflect_edge_relaxed(const uint8_t *samples, int n, int edge_i,
                                     int grad);
static bool tdr_is_pcb_coupling(const uint8_t *samples, int n, int launch_i,
                                int edge_i, int grad);
static int tdr_reflection_quality(const uint8_t *samples, int n, int launch_i,
                                  int edge_i, int grad);
static void tdr_normalize_open_distance(TdrResult &r);
static int  tdr_find_preferred_open_reflect(const uint8_t *samples, int n,
                                            int launch_i, int &out_g);
static int  tdr_find_open_zone_edge(const uint8_t *samples, int n,
                                    int launch_i, int &out_g);
static int  tdr_gp3_pulse_width(const uint8_t *samples, int n);
static int  tdr_find_calibrate_max_deriv(const uint8_t *samples, int n,
                                         int launch_i, int &out_g);
static int  tdr_find_calibrate_any_edge(const uint8_t *samples, int n,
                                        int launch_i, int &out_g);
static bool tdr_is_connector_short(const uint8_t *samples, int n, int launch_i,
                                   int *out_reflect_i);
static bool tdr_shot_gp3_immediate_follow(const uint8_t *samples, int n,
                                          int launch_i);
static int  tdr_gp3_rise_delta_after_launch(const uint8_t *samples, int n,
                                            int launch_i);
static bool tdr_shot_blocks_late_open(const uint8_t *samples, int n,
                                      int launch_i);

static void tdr_fill_distance(TdrResult &r, int launch_i, int reflect_i) {
    if (launch_i >= 0 && reflect_i >= launch_i && launch_i < 128 &&
        reflect_i < 128) {
        uint32_t dt = g_sample_time_ns[reflect_i] - g_sample_time_ns[launch_i];
        tdr_fill_distance_ns(r, dt);
    } else {
        int delta = reflect_i - launch_i;
        delta = tdr_zero_corrected_delta_samples(delta);
        tdr_fill_distance_ns(r,
            (uint32_t)((float)delta * g_capture_sample_period_ns));
    }
    tdr_fixup_open_distance_scale(r, launch_i, reflect_i);
}

static float tdr_distance_m_from_indices(int launch_i, int reflect_i) {
    TdrResult tmp{};
    tdr_fill_distance(tmp, launch_i, reflect_i);
    return tmp.distance_m;
}

static float tdr_roundtrip_ns_3m_ref() {
    const float C = 299792458.0f;
    return 2.0f * 3.0f / (C * g_velocity_factor) * 1e9f;
}

// d=2-5 med dist>4.5 m: sample-periode for høj → skaler til ~3 m reference
static void tdr_fixup_open_distance_scale(TdrResult &r, int launch_i, int reflect_i) {
    if (launch_i < 0 || reflect_i < launch_i)
        return;
    int d = reflect_i - launch_i;
    if (d < TDR_OPEN_DELTA24_LO || d > 5)
        return;
    if (r.distance_m <= TDR_OPEN_DISPLAY_DIST_HI)
        return;

    uint32_t dt_ns = 0;
    if (launch_i < 128 && reflect_i < 128)
        dt_ns = g_sample_time_ns[reflect_i] - g_sample_time_ns[launch_i];
    else
        dt_ns = (uint32_t)((float)d * g_capture_sample_period_ns);
    if (dt_ns < 1)
        return;

    float t_3m_ns = tdr_roundtrip_ns_3m_ref();
    r.distance_m *= t_3m_ns / (float)dt_ns;
    if (r.distance_m > TDR_MAX_DISTANCE_M)
        r.distance_m = TDR_MAX_DISTANCE_M;
}

// Stigende kant sample 12-18, delta 2-5 — uden bred-puls fallback (undgå falsk 3 m ved stik-SHORT)
static int tdr_find_late_open_peak_strict(const uint8_t *samples, int n,
                                            int launch_i, int &out_g) {
    if (launch_i < 0)
        return -1;

    const bool wide_cable =
        tdr_gp3_pulse_width(samples, n) >= TDR_WIDE_CABLE_PULSE_MIN;

    int peak_lo = TDR_LATE_PEAK_SAMPLE_LO;
    int peak_hi = TDR_LATE_PEAK_SAMPLE_HI;
    if (peak_lo < launch_i + TDR_DIST_MIN_DELTA)
        peak_lo = launch_i + TDR_DIST_MIN_DELTA;
    if (peak_hi >= n)
        peak_hi = n - 1;

    int peak_best = -1;
    int peak_delta = -1;
    int peak_q     = 0;
    for (int i = peak_lo; i <= peak_hi; i++) {
        if (i < 1 || tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        if (g <= 0)
            continue;
        if (!tdr_reflect_edge_relaxed(samples, n, i, g))
            continue;
        int delta = i - launch_i;
        if (delta < TDR_OPEN_DELTA24_LO || delta > TDR_OPEN_DELTA24_HI)
            continue;
        if (!wide_cable && tdr_is_pcb_coupling(samples, n, launch_i, i, g))
            continue;
        int q = tdr_reflection_quality(samples, n, launch_i, i, g);
        if (q > peak_q || (q == peak_q && delta > peak_delta)) {
            peak_q     = q;
            peak_delta = delta;
            peak_best  = i;
            out_g      = g;
        }
    }
    return peak_best;
}

// Peak-søgning sample 12-18, delta 2-5 (~3 m åben kabel)
static int tdr_find_late_open_peak(const uint8_t *samples, int n, int launch_i,
                                   int &out_g) {
    if (launch_i < 0)
        return -1;

    int peak_best = tdr_find_late_open_peak_strict(samples, n, launch_i, out_g);
    if (peak_best >= 0)
        return peak_best;

    const bool wide_cable =
        tdr_gp3_pulse_width(samples, n) >= TDR_WIDE_CABLE_PULSE_MIN;

    // Bred GP3-puls + kun tidlig kant (fx ri_raw=11): brug launch+2..+5 uanset g>0
    if (wide_cable) {
        int raw_g = 0;
        int raw_i = tdr_find_open_zone_edge(samples, n, launch_i, raw_g);
        int raw_d = (raw_i >= 0) ? (raw_i - launch_i) : 99;
        if (raw_d <= TDR_SHORT_DELTA_MAX) {
            for (int d = TDR_OPEN_DELTA24_LO; d <= TDR_OPEN_DELTA24_HI; d++) {
                int i = launch_i + d;
                if (i < 1 || i >= n || tdr_is_pulse_off_sample(i))
                    continue;
                if (!samples[i] && !(i > 0 && samples[i - 1]))
                    continue;
                int g = (int)samples[i] - (int)samples[i - 1];
                if (g <= 0 && samples[i])
                    g = 1;
                peak_best = i;
                out_g     = g;
                break;
            }
        }
    }
    return peak_best;
}

// Per-shot: ri_raw ved launch+1 + bred puls → ri_use launch+2 (~3 m)
static bool tdr_apply_shot_late_open_upgrade(TdrResult &r, const uint8_t *samples,
                                             int n, int pulse_w) {
    if (pulse_w < TDR_WIDE_CABLE_PULSE_MIN)
        return false;

    int li = r.launch_index >= 0 ? r.launch_index : TDR_PULSE_ON;
    if (pulse_w < TDR_WIDE_CABLE_PULSE_MIN) {
        if (tdr_shot_blocks_late_open(samples, n, li))
            return false;
    }
    int raw_g = 0;
    int raw_i = tdr_find_open_zone_edge(samples, n, li, raw_g);
    if (raw_i < 0 && r.fault_found && r.reflect_index >= 0)
        raw_i = r.reflect_index;
    int raw_d = (raw_i >= 0) ? (raw_i - li) : 99;
    if (raw_d > TDR_SHORT_DELTA_MAX)
        return false;

    int late_g = 0;
    int late_i = tdr_find_late_open_peak(samples, n, li, late_g);
    if (late_i < 0)
        return false;

    int late_d = late_i - li;
    if (late_d < TDR_DIST_MIN_DELTA)
        return false;

    r.fault_found   = true;
    r.is_short      = false;
    r.no_cable      = false;
    r.weak_signal   = false;
    r.launch_index  = li;
    r.reflect_index = late_i;
    tdr_fill_distance(r, li, late_i);
    tdr_fixup_open_distance_scale(r, li, late_i);
    return true;
}

static bool tdr_apply_late_open_reflect(TdrResult &r, const uint8_t *samples, int n) {
    if (!r.fault_found || r.is_short || r.launch_index < 0)
        return false;
    int d = r.reflect_index - r.launch_index;
    if (d > TDR_SHORT_DELTA_MAX)
        return false;

    int li = r.launch_index;
    int zone_g = 0;
    int zone_i = tdr_find_late_open_peak(samples, n, li, zone_g);
    if (zone_i < 0)
        return false;

    r.reflect_index = zone_i;
    tdr_fill_distance(r, li, zone_i);
    tdr_fixup_open_distance_scale(r, li, zone_i);
    return true;
}

static float tdr_median_open_distance_shots(const TdrResult *results, int tries) {
    float vals[6];
    int n = 0;
    for (int i = 0; i < tries; i++) {
        if (!results[i].fault_found || results[i].is_short ||
            results[i].launch_index < 0)
            continue;
        int d = results[i].reflect_index - results[i].launch_index;
        if (d < TDR_DIST_MIN_DELTA)
            continue;
        float dm = results[i].distance_m;
        if (dm < TDR_DIST_FORCE_OPEN_M)
            dm = tdr_distance_m_from_indices(results[i].launch_index,
                                             results[i].reflect_index);
        if (dm >= TDR_DIST_FORCE_OPEN_M)
            vals[n++] = dm;
    }
    if (n <= 0)
        return -1.0f;
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (vals[j] < vals[i]) {
                float t = vals[i];
                vals[i] = vals[j];
                vals[j] = t;
            }
        }
    }
    return vals[n / 2];
}

static void tdr_finalize_stable_open(TdrResult &r, const TdrResult *results,
                                     int tries, float median_open_dist_m) {
    if (!r.fault_found || r.is_short || r.launch_index < 0)
        return;

    int fn = 128;
    const uint8_t *f = tdr_get_filtered(fn);
    int li = r.launch_index;
    int d  = r.reflect_index - li;
    if (d < 0)
        d = 0;

    if (d <= TDR_SHORT_DELTA_MAX)
        tdr_apply_late_open_reflect(r, f, fn);

    d = r.reflect_index - li;
    if (d < 0)
        d = 0;
    tdr_fixup_open_distance_scale(r, li, r.reflect_index);

    if (median_open_dist_m >= TDR_OPEN_DISPLAY_DIST_LO) {
        r.distance_m = median_open_dist_m;
    } else if (r.distance_m < TDR_DIST_FORCE_OPEN_M) {
        if (median_open_dist_m >= TDR_DIST_FORCE_OPEN_M)
            r.distance_m = median_open_dist_m;
        else if (d >= TDR_DIST_MIN_DELTA)
            tdr_normalize_open_distance(r);
    }
    tdr_normalize_open_distance(r);
}

struct TdrStableCycleCache {
    TdrResult shots[6];
    int       tries;
    float     median_open_dist_m;
    float     max_zmax_m;
    int       pulse_width_median;
    int       median_delta;
    int       shot_pref_delta[6];
    bool      valid;
    bool      hw_short;
    bool      short_blocks_open;
};

static TdrStableCycleCache g_stable_cycle{};

static bool tdr_shot_cable_end_reflection(const uint8_t *samples, int n,
                                          int launch_i);

static bool tdr_shot_good_open_delta_dist(const TdrResult &c, int &delta, float &dist_m) {
    if (!c.fault_found || c.is_short || c.launch_index < 0)
        return false;
    int li = c.launch_index;
    delta = c.reflect_index - li;
    if (delta < TDR_DIST_MIN_DELTA)
        return false;
    dist_m = c.distance_m;
    if (dist_m < TDR_DIST_FORCE_OPEN_M)
        dist_m = tdr_distance_m_from_indices(li, c.reflect_index);
    return dist_m >= TDR_DIST_FORCE_OPEN_M;
}

static bool tdr_shot_open_dist_15(const TdrResult &c, int &delta, float &dist_m) {
    if (!tdr_shot_good_open_delta_dist(c, delta, dist_m))
        return false;
    return dist_m >= TDR_OPEN_DISPLAY_DIST_LO;
}

static bool tdr_cycle_any_open_dist15_shot(const TdrResult *results, int tries) {
    for (int i = 0; i < tries; i++) {
        int d = 0;
        float dm = 0.0f;
        if (tdr_shot_open_dist_15(results[i], d, dm))
            return true;
    }
    return false;
}

static bool tdr_result_needs_open_upgrade(const TdrResult &r) {
    if (!r.fault_found || r.is_short || r.no_cable)
        return false;
    int d = 0;
    float dm = 0.0f;
    if (!tdr_shot_good_open_delta_dist(r, d, dm))
        return true;
    return d < TDR_DIST_MIN_DELTA || dm < TDR_DIST_FORCE_OPEN_M;
}

static void tdr_apply_open_shot_indices(TdrResult &out) {
    int n = 128;
    const uint8_t *f = tdr_get_filtered(n);
    int launch_i = out.launch_index >= 0 ? out.launch_index : TDR_PULSE_ON;
    int zone_g = 0;
    int zone_i = -1;
    if (!tdr_shot_blocks_late_open(f, n, launch_i) ||
        tdr_shot_cable_end_reflection(f, n, launch_i)) {
        zone_i = tdr_find_preferred_open_reflect(f, n, launch_i, zone_g);
        if (zone_i < 0)
            zone_i = tdr_find_late_open_peak(f, n, launch_i, zone_g);
    }
    if (zone_i >= 0) {
        out.fault_found   = true;
        out.reflect_index = zone_i;
        out.launch_index  = launch_i;
        tdr_fill_distance(out, launch_i, zone_i);
    }
    tdr_normalize_open_distance(out);
}

static bool tdr_cycle_any_good_open_shot(const TdrResult *results, int tries) {
    for (int i = 0; i < tries; i++) {
        int d = 0;
        float dm = 0.0f;
        if (tdr_shot_good_open_delta_dist(results[i], d, dm))
            return true;
    }
    return false;
}

static bool tdr_result_good_open_final(const TdrResult &r) {
    int d = 0;
    float dm = 0.0f;
    return r.fault_found && !r.is_short && !r.no_cable &&
           tdr_shot_good_open_delta_dist(r, d, dm);
}

static bool tdr_upgrade_from_any_good_open_shot(TdrResult &r, const TdrResult *results,
                                                int tries, float median_open_dist_m) {
    int best_i = -1;
    int best_d = -1;
    float best_dm = -1.0f;
    for (int i = 0; i < tries; i++) {
        int d = 0;
        float dm = 0.0f;
        if (!tdr_shot_open_dist_15(results[i], d, dm) &&
            !tdr_shot_good_open_delta_dist(results[i], d, dm))
            continue;
        if (d > best_d || (d == best_d && dm > best_dm)) {
            best_d  = d;
            best_dm = dm;
            best_i  = i;
        }
    }
    if (best_i < 0)
        return false;

    r = results[best_i];
    r.is_short         = false;
    r.no_cable         = false;
    r.weak_signal      = false;
    r.unstable         = false;
    r.consensus_strong = true;
    tdr_apply_open_shot_indices(r);
    tdr_finalize_stable_open(r, results, tries, median_open_dist_m);
#ifdef TDR_DEBUG
    g_stable_dbg.rule = "OPEN";
#endif
    return true;
}

static int tdr_count_cycle_pref_open_band(const int *shot_pref_delta, int tries) {
    int c = 0;
    for (int i = 0; i < tries; i++) {
        if (shot_pref_delta[i] >= TDR_OPEN_DELTA24_LO &&
            shot_pref_delta[i] <= TDR_OPEN_DELTA24_HI)
            c++;
    }
    return c;
}

static bool tdr_cycle_has_open_evidence(const TdrStableCycleCache &cyc) {
    if (!cyc.valid || cyc.hw_short || cyc.short_blocks_open)
        return false;
    if (cyc.max_zmax_m >= TDR_OPEN_DISPLAY_DIST_LO)
        return true;
    if (tdr_cycle_any_good_open_shot(cyc.shots, cyc.tries))
        return true;
    if (tdr_count_cycle_pref_open_band(cyc.shot_pref_delta, cyc.tries) >= 1 &&
        cyc.pulse_width_median >= TDR_WIDE_CABLE_PULSE_MIN)
        return true;
    if (cyc.median_delta >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO &&
        cyc.pulse_width_median >= TDR_WIDE_CABLE_PULSE_MIN)
        return true;
    return false;
}

static bool tdr_result_bogus_open_cycle(const TdrResult &r) {
    if (r.is_short)
        return false;
    if (r.no_cable)
        return g_stable_cycle.valid &&
               tdr_cycle_has_open_evidence(g_stable_cycle);
    if (!r.fault_found)
        return true;
    if (r.reflect_index < 0)
        return true;
    return r.distance_m < TDR_DIST_FORCE_OPEN_M;
}

static void tdr_save_stable_cycle(const TdrResult *results, int tries,
                                  float median_open_dist_m, float max_zmax_m,
                                  int pulse_width_median, int median_delta,
                                  const int *shot_pref_delta, bool for_calibrate,
                                  bool hw_short, bool short_blocks_open) {
    if (for_calibrate) {
        g_stable_cycle.valid = false;
        return;
    }
    for (int i = 0; i < tries; i++) {
        g_stable_cycle.shots[i] = results[i];
        g_stable_cycle.shot_pref_delta[i] =
            shot_pref_delta ? shot_pref_delta[i] : -1;
    }
    g_stable_cycle.tries              = tries;
    g_stable_cycle.median_open_dist_m = median_open_dist_m;
    g_stable_cycle.max_zmax_m         = max_zmax_m;
    g_stable_cycle.pulse_width_median = pulse_width_median;
    g_stable_cycle.median_delta       = median_delta;
    g_stable_cycle.hw_short           = hw_short;
    g_stable_cycle.short_blocks_open  = short_blocks_open;
    g_stable_cycle.valid              = true;
}

static void tdr_apply_zmax_open_result(TdrResult &r, float zmax_m,
                                       float median_open_dist_m);

static bool tdr_force_open_from_cycle_evidence(TdrResult &r) {
    if (!tdr_cycle_has_open_evidence(g_stable_cycle))
        return false;

    const TdrResult *results = g_stable_cycle.shots;
    const int        tries   = g_stable_cycle.tries;
    const float      md      = g_stable_cycle.median_open_dist_m;
    const float      zmax    = g_stable_cycle.max_zmax_m;

    if (tdr_upgrade_from_any_good_open_shot(r, results, tries, md))
        return true;

    int best_i = -1;
    int best_pd = -1;
    for (int i = 0; i < tries; i++) {
        int pd = g_stable_cycle.shot_pref_delta[i];
        if (pd < TDR_OPEN_DELTA24_LO || pd > TDR_OPEN_DELTA24_HI)
            continue;
        if (results[i].is_short)
            continue;
        if (pd > best_pd) {
            best_pd = pd;
            best_i  = i;
        }
    }
    if (best_i >= 0) {
        r = results[best_i];
        r.is_short         = false;
        r.no_cable         = false;
        r.weak_signal      = false;
        r.unstable         = false;
        r.consensus_strong = true;
        tdr_apply_open_shot_indices(r);
        tdr_finalize_stable_open(r, results, tries, md);
#ifdef TDR_DEBUG
        g_stable_dbg.rule        = "OPEN";
        g_stable_dbg.fix_applied = true;
#endif
        if (r.fault_found && r.distance_m >= TDR_DIST_FORCE_OPEN_M)
            return true;
    }

    if (zmax >= TDR_OPEN_DISPLAY_DIST_LO) {
        tdr_apply_zmax_open_result(r, zmax, md);
#ifdef TDR_DEBUG
        g_stable_dbg.fix_applied = true;
#endif
        return true;
    }

    if (g_stable_cycle.median_delta >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO &&
        g_stable_cycle.pulse_width_median >= TDR_WIDE_CABLE_PULSE_MIN) {
        int li = TDR_PULSE_ON;
        int di = li + g_stable_cycle.median_delta;
        r.fault_found      = true;
        r.is_short         = false;
        r.no_cable         = false;
        r.weak_signal      = false;
        r.unstable         = false;
        r.consensus_strong = true;
        r.launch_index     = li;
        r.reflect_index    = di;
        tdr_fill_distance(r, li, di);
        tdr_finalize_stable_open(r, results, tries, md);
        if (r.distance_m < TDR_OPEN_DISPLAY_DIST_LO) {
            if (md >= TDR_OPEN_DISPLAY_DIST_LO)
                r.distance_m = md;
            else if (zmax >= TDR_OPEN_DISPLAY_DIST_LO)
                r.distance_m = zmax;
        }
        tdr_normalize_open_distance(r);
#ifdef TDR_DEBUG
        g_stable_dbg.rule        = "OPEN";
        g_stable_dbg.fix_applied = true;
#endif
        return r.fault_found && r.distance_m >= TDR_DIST_FORCE_OPEN_M;
    }
    return false;
}

static void tdr_apply_zmax_open_result(TdrResult &r, float zmax_m,
                                       float median_open_dist_m) {
    if (zmax_m < TDR_OPEN_DISPLAY_DIST_LO)
        return;
    r.fault_found      = true;
    r.is_short         = false;
    r.no_cable         = false;
    r.weak_signal      = false;
    r.unstable         = false;
    r.consensus_strong = true;
    if (median_open_dist_m >= TDR_OPEN_DISPLAY_DIST_LO)
        r.distance_m = median_open_dist_m;
    else
        r.distance_m = zmax_m;
    if (r.launch_index < 0)
        r.launch_index = TDR_PULSE_ON;
    tdr_apply_open_shot_indices(r);
    if (r.distance_m < TDR_OPEN_DISPLAY_DIST_LO)
        r.distance_m = zmax_m;
#ifdef TDR_DEBUG
    g_stable_dbg.rule = "OPEN";
#endif
}

static void tdr_fixup_stable_result(TdrResult &r) {
    if (!g_stable_cycle.valid)
        return;

    r.vote_pulse_width = (uint8_t)g_stable_cycle.pulse_width_median;

    if (r.no_cable && tdr_cycle_has_open_evidence(g_stable_cycle)) {
        if (tdr_force_open_from_cycle_evidence(r))
            return;
    }

    if (r.is_short || g_stable_cycle.hw_short || g_stable_cycle.short_blocks_open)
        return;

    const TdrResult *results = g_stable_cycle.shots;
    const int        tries   = g_stable_cycle.tries;
    const float      md      = g_stable_cycle.median_open_dist_m;
    const float      zmax    = g_stable_cycle.max_zmax_m;

#ifdef TDR_DEBUG
    g_stable_dbg.fix_applied = false;
#endif

    const bool any_dist15 = tdr_cycle_any_open_dist15_shot(results, tries);
    const bool any_open   = tdr_cycle_any_good_open_shot(results, tries) ||
                            any_dist15 ||
                            zmax >= TDR_OPEN_DISPLAY_DIST_LO ||
                            tdr_cycle_has_open_evidence(g_stable_cycle);

    if (tdr_result_bogus_open_cycle(r) && tdr_cycle_has_open_evidence(g_stable_cycle)) {
        if (tdr_force_open_from_cycle_evidence(r))
            return;
    }

    if (any_open) {
        if (any_dist15 || !tdr_result_good_open_final(r)) {
            if (tdr_upgrade_from_any_good_open_shot(r, results, tries, md)) {
#ifdef TDR_DEBUG
                g_stable_dbg.fix_applied = true;
#endif
            } else if (zmax >= TDR_OPEN_DISPLAY_DIST_LO) {
                tdr_apply_zmax_open_result(r, zmax, md);
#ifdef TDR_DEBUG
                g_stable_dbg.fix_applied = true;
#endif
            }
        } else if (tdr_result_needs_open_upgrade(r)) {
            if (tdr_upgrade_from_any_good_open_shot(r, results, tries, md)) {
#ifdef TDR_DEBUG
                g_stable_dbg.fix_applied = true;
#endif
            } else if (zmax >= TDR_OPEN_DISPLAY_DIST_LO) {
                tdr_apply_zmax_open_result(r, zmax, md);
#ifdef TDR_DEBUG
                g_stable_dbg.fix_applied = true;
#endif
            } else {
                tdr_finalize_stable_open(r, results, tries, md);
            }
        }
    } else if (r.fault_found && !r.is_short && !r.no_cable) {
        tdr_finalize_stable_open(r, results, tries, md);
    }

    if (zmax >= TDR_OPEN_DISPLAY_DIST_LO &&
        r.fault_found && !r.is_short && !r.no_cable &&
        r.distance_m < TDR_OPEN_DISPLAY_DIST_LO) {
        if (md >= TDR_OPEN_DISPLAY_DIST_LO)
            r.distance_m = md;
        else
            r.distance_m = zmax;
    }

    if (r.fault_found && !r.is_short && !r.no_cable &&
        r.distance_m < TDR_DIST_FORCE_OPEN_M) {
        r.unstable = true;
        if (md >= TDR_OPEN_DISPLAY_DIST_LO)
            r.distance_m = md;
        else if (zmax >= TDR_OPEN_DISPLAY_DIST_LO)
            r.distance_m = zmax;
        if (r.distance_m < TDR_DIST_FORCE_OPEN_M) {
            if (tdr_cycle_has_open_evidence(g_stable_cycle))
                tdr_force_open_from_cycle_evidence(r);
            else {
                r.fault_found = false;
                r.unstable    = true;
            }
        }
    }

    if (tdr_result_bogus_open_cycle(r) && tdr_cycle_has_open_evidence(g_stable_cycle))
        tdr_force_open_from_cycle_evidence(r);
}

// Første kant efter start_idx (til korte kabler / åben ende)
static int tdr_find_edge(const uint8_t *samples, int n, int start_idx, int &out_g) {
    for (int i = start_idx; i < n; i++) {
        int g = (int)samples[i] - (int)samples[i - 1];
        if (g != 0) {
            out_g = g;
            return i;
        }
    }
    out_g = 0;
    return -1;
}

static int tdr_count_edges(const uint8_t *samples, int n) {
    int c = 0;
    for (int i = 1; i < n; i++) {
        if (samples[i] != samples[i - 1])
            c++;
    }
    return c;
}

static int tdr_count_post_launch_edges(const uint8_t *samples, int n,
                                       int launch_i) {
    if (launch_i < 0 || launch_i >= n - 1)
        return 0;
    int hi = launch_i + TDR_CALIBRATE_OPEN_DELTA_HI;
    if (hi >= n)
        hi = n - 1;
    int c = 0;
    for (int i = launch_i + 2; i <= hi; i++) {
        if (samples[i] != samples[i - 1])
            c++;
    }
    return c;
}

static void tdr_fill_calibrate_meta(TdrResult &r) {
    int n = 128;
    const uint8_t *s = tdr_get_samples(n);
    int li = r.launch_index >= 0 ? r.launch_index : TDR_PULSE_ON;

    int edges = tdr_count_post_launch_edges(s, n, li);
    if (edges == 0) {
        const uint8_t *f = tdr_get_filtered(n);
        edges = tdr_count_post_launch_edges(f, n, li);
    }
    if (edges == 0 && r.fault_found && r.reflect_index > li)
        edges = 1;
    if (edges == 0 && g_calib_zmax_m >= 0.5f)
        edges = 1;
    r.cal_edges = (uint8_t)(edges > 255 ? 255 : edges);

    int dbg_g = 0;
    int dbg_i = tdr_find_preferred_open_reflect(g_filtered, 128, li, dbg_g);
    if (dbg_i < 0)
        dbg_i = tdr_find_late_open_peak(g_filtered, 128, li, dbg_g);
    if (dbg_i < 0)
        dbg_i = tdr_find_calibrate_max_deriv(s, n, li, dbg_g);
    if (dbg_i < 0)
        dbg_i = tdr_find_calibrate_any_edge(s, n, li, dbg_g);
    r.cal_best_delta = (dbg_i >= 0) ? (int8_t)(dbg_i - li) : -1;
}

static bool tdr_calib_dist_in_band(float dist_m) {
    return dist_m >= TDR_CALIB_DIST_LO_M && dist_m <= TDR_CALIB_DIST_HI_M;
}

static int tdr_calib_pick_reflect_for_zmax(const uint8_t *samples, int n,
                                           int launch_i, float zmax_m,
                                           int &out_g) {
    if (launch_i < 0 || zmax_m < TDR_CALIB_DIST_LO_M)
        return -1;

    int best_i = -1;
    float best_err = 1e9f;
    int best_q = 0;

    int lo = launch_i + TDR_OPEN_DELTA24_LO;
    int hi = launch_i + TDR_OPEN_DELTA24_HI;
    if (hi >= n)
        hi = n - 1;

    for (int i = lo; i <= hi; i++) {
        if (i < 1 || tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        if (g <= 0 && !tdr_reflect_edge_relaxed(samples, n, i, g))
            continue;
        int delta = i - launch_i;
        if (delta < TDR_DIST_MIN_DELTA || delta > TDR_OPEN_DELTA24_HI)
            continue;
        if (tdr_is_pcb_coupling(samples, n, launch_i, i, g))
            continue;
        float dm = tdr_distance_m_from_indices(launch_i, i);
        if (dm < 0.0f)
            continue;
        float err = dm - zmax_m;
        if (err < 0.0f)
            err = -err;
        int q = tdr_reflection_quality(samples, n, launch_i, i, g);
        if (err < best_err - 0.05f || (err <= best_err + 0.05f && q > best_q)) {
            best_err = err;
            best_q   = q;
            best_i   = i;
            out_g    = g;
        }
    }

    if (best_i >= 0)
        return best_i;

    int md_g = 0;
    int md_i = tdr_find_calibrate_max_deriv(samples, n, launch_i, md_g);
    if (md_i >= 0) {
        int delta = md_i - launch_i;
        if (delta >= TDR_DIST_MIN_DELTA && delta <= TDR_OPEN_DELTA24_HI) {
            out_g = md_g;
            return md_i;
        }
    }
    return -1;
}

static bool tdr_calib_apply_reflect_fixup(TdrResult &r, int li, int zone_i) {
    if (zone_i < 0)
        return false;
    r.reflect_index = zone_i;
    r.launch_index  = li;
    r.is_short      = false;
    r.fault_found   = true;
    tdr_fill_distance(r, li, zone_i);
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
    g_stable_dbg.fix_applied = true;
#endif
    return true;
}

static int tdr_calib_find_reflect_fixup(const uint8_t *f, int fn, int li) {
    int zone_g = 0;
    int zone_i = tdr_find_preferred_open_reflect(f, fn, li, zone_g);
    if (zone_i < 0)
        zone_i = tdr_find_late_open_peak(f, fn, li, zone_g);
    if (zone_i < 0) {
        zone_i = tdr_find_calibrate_max_deriv(f, fn, li, zone_g);
        if (zone_i >= 0) {
            int d = zone_i - li;
            if (d < TDR_DIST_MIN_DELTA || d > TDR_OPEN_DELTA24_HI)
                zone_i = -1;
        }
    }
    if (zone_i < 0 && g_calib_zmax_m >= TDR_CALIB_DIST_LO_M)
        zone_i = tdr_calib_pick_reflect_for_zmax(f, fn, li, g_calib_zmax_m, zone_g);
    return zone_i;
}

static void tdr_fixup_calibrate_result(TdrResult &r) {
    if (!r.fault_found || r.is_short)
        return;

    int fn = 128;
    const uint8_t *f = tdr_get_filtered(fn);
    int li = r.launch_index >= 0 ? r.launch_index : TDR_PULSE_ON;
    if (r.launch_index < 0)
        r.launch_index = li;

    int rd = r.reflect_index - li;
    if (rd < 0)
        rd = 0;

    const bool zmax_fixup =
        g_calib_zmax_m >= TDR_CALIB_DIST_LO_M &&
        (rd < TDR_DIST_MIN_DELTA || !tdr_calib_dist_in_band(r.distance_m) ||
         (tdr_calib_dist_in_band(g_calib_zmax_m) &&
          (r.distance_m < 0.01f ||
           r.distance_m / g_calib_zmax_m < 0.75f ||
           r.distance_m / g_calib_zmax_m > 1.35f)));

    if (rd <= TDR_SHORT_DELTA_MAX || rd < TDR_DIST_MIN_DELTA ||
        r.distance_m < TDR_DIST_FORCE_OPEN_M || zmax_fixup) {
        int zone_i = tdr_calib_find_reflect_fixup(f, fn, li);
        if (zone_i >= 0) {
            tdr_calib_apply_reflect_fixup(r, li, zone_i);
        } else if (rd <= TDR_SHORT_DELTA_MAX) {
            if (tdr_apply_late_open_reflect(r, f, fn)) {
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
                g_stable_dbg.fix_applied = true;
#endif
            }
        }
        if (zone_i < 0 && g_calib_zmax_m >= TDR_CALIB_DIST_LO_M &&
            tdr_calib_dist_in_band(g_calib_zmax_m)) {
            r.distance_m = g_calib_zmax_m;
            if (rd < TDR_DIST_MIN_DELTA) {
                int zg = 0;
                int zi = tdr_calib_pick_reflect_for_zmax(f, fn, li, g_calib_zmax_m, zg);
                if (zi >= 0)
                    tdr_calib_apply_reflect_fixup(r, li, zi);
            }
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
            g_stable_dbg.fix_applied = true;
#endif
        }
    }

    tdr_fill_calibrate_meta(r);
}

static void tdr_calibrate_finalize_unstable(TdrResult &r) {
    const bool valid_reflect =
        r.fault_found && !r.is_short &&
        (tdr_calib_dist_in_band(r.distance_m) ||
         g_calib_zmax_m >= TDR_CALIB_DIST_LO_M);

    const int spread = g_calib_unst.spread;
    const bool shape_bad =
        g_calib_unst.width_shape_unstable || g_calib_unst.amp_shape_unstable;
    const bool spread_extreme = spread > 8;
    const bool spread_high    = spread >= TDR_NO_CABLE_SPREAD;

    const bool truly_unstable =
        g_calib_unst.nfound < 3 ||
        (spread_extreme && g_calib_unst.delta_unstable) ||
        (spread_high && shape_bad && g_calib_unst.delta_unstable);

#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
    g_calib_unst_reason = nullptr;
    if (truly_unstable) {
        if (g_calib_unst.nfound < 3)
            g_calib_unst_reason = "nfound";
        else if (spread_extreme && g_calib_unst.delta_unstable)
            g_calib_unst_reason = "spread";
        else if (spread_high && shape_bad)
            g_calib_unst_reason = "shape";
        else
            g_calib_unst_reason = "unstable";
    }
#endif

    if (r.is_short)
        r.unstable = false;
    else if (valid_reflect)
        r.unstable = false;
    else
        r.unstable = truly_unstable;
}

static int tdr_edge_run(const uint8_t *samples, int n, int edge_i) {
    if (edge_i < 1 || edge_i >= n)
        return 0;
    uint8_t level = samples[edge_i];
    int run = 1;
    for (int j = edge_i + 1; j < n && j < edge_i + 8; j++) {
        if (samples[j] == level)
            run++;
        else
            break;
    }
    return run;
}

static int tdr_abs_grad(int g) {
    return (g < 0) ? -g : g;
}

static bool tdr_gp3_idle_stable();
static bool tdr_hw_connector_shorted();
static bool tdr_hw_ground_fault();
static bool tdr_mic_connector_short();
static void tdr_filter_majority();
static void tdr_gpio_sio_begin();
static void tdr_set_no_signal(TdrResult &r, bool weak);

// Kun GP2 puls-OFF sample — nærliggende kanter valideres via run/grad
static bool tdr_is_pulse_off_sample(int i) {
    return i == g_capture_pulse_off;
}

static bool tdr_is_pulse_off_zone(int i) {
    return i >= g_capture_pulse_off - 1 && i <= g_capture_pulse_off + 1;
}

static int tdr_calibrate_open_hi(int launch_i) {
    int hi = launch_i + TDR_CALIBRATE_OPEN_DELTA_HI;
    if (hi >= 128)
        hi = 127;
    return hi;
}

// Berøring giver ofte enkelt-sample glitches; rigtig refleksion holder niveau
static bool tdr_edge_is_solid(const uint8_t *samples, int n, int edge_i) {
    if (edge_i < 1 || edge_i >= n)
        return false;
    int run = tdr_edge_run(samples, n, edge_i);
    if (edge_i <= g_capture_pulse_off + TDR_OPEN_DELTA_HI)
        return run >= TDR_OPEN_MIN_RUN;
    return run >= 2;
}

// Amplitude-trin ved kant (0/1) — stærkere end enkelt-sample grad=1 støj
static int tdr_edge_level_step(const uint8_t *samples, int n, int edge_i) {
    if (edge_i < 1 || edge_i >= n)
        return 0;
    int before = (int)samples[edge_i - 1];
    int after  = (int)samples[edge_i];
    int step   = after - before;
    if (step < 0)
        step = -step;
    int run = tdr_edge_run(samples, n, edge_i);
    if (run >= 2 && edge_i + 1 < n) {
        int hold = (int)samples[edge_i + 1] - before;
        if (hold < 0)
            hold = -hold;
        if (hold > step)
            step = hold;
    }
    return step;
}

// Styrke af refleksionskant (grad + hold + timing) — skelner ~3 m kabel fra divider-støj
static int tdr_reflection_quality(const uint8_t *samples, int n,
                                int launch_i, int edge_i, int grad) {
    if (edge_i < 1 || launch_i < 0)
        return 0;
    int ag   = tdr_abs_grad(grad);
    int run  = tdr_edge_run(samples, n, edge_i);
    int step = tdr_edge_level_step(samples, n, edge_i);
    int delta = edge_i - launch_i;

    int q = ag * ag * 3 + ag * 2 + run + step * 2;
    if (edge_i > g_capture_pulse_off + 2)
        q += 8;
    else if (edge_i >= g_capture_pulse_off)
        q += 3;
    // Lang puls: ~3 m åben ende typisk launch+4..+6 (sample ~14 ved launch 10)
    if (delta >= 4 && delta <= 7 &&
        edge_i >= TDR_PULSE_ON + 2 && edge_i < g_capture_pulse_off - 4)
        q += 4;
    return q;
}

static int tdr_baseline_diff_count(const uint8_t *pulse, const uint8_t *base,
                                   int lo, int hi) {
    if (lo < 0)
        lo = 0;
    if (hi >= 128)
        hi = 127;
    int c = 0;
    for (int i = lo; i <= hi; i++) {
        if (pulse[i] != base[i])
            c++;
    }
    return c;
}

static int tdr_open_window_baseline_diff(const uint8_t *pulse, const uint8_t *base,
                                         int launch_i) {
    if (!g_baseline_valid || launch_i < 0)
        return 0;
    int lo = launch_i + TDR_OPEN_DELTA_LO;
    int hi = launch_i + TDR_OPEN_DELTA_HI;
    return tdr_baseline_diff_count(pulse, base, lo, hi);
}

// Refleksionskant: styrke + hold — afviser puls-kobling/støj på åben GP3
static bool tdr_reflect_edge_ok(const uint8_t *samples, int n, int edge_i, int grad) {
    if (edge_i < 1 || grad == 0)
        return false;
    if (tdr_is_pulse_off_sample(edge_i))
        return false;

    int ag  = tdr_abs_grad(grad);
    int run = tdr_edge_run(samples, n, edge_i);
    int step = tdr_edge_level_step(samples, n, edge_i);

    // Efter puls-OFF: svag divider-refleksion (fx ~3 m) kan have grad=1 men run>=2
    if (edge_i > g_capture_pulse_off + 1) {
        if (run >= TDR_OPEN_MIN_RUN)
            return true;
        if (ag >= TDR_MIN_REFLECT_GRAD && step >= 1)
            return true;
        return false;
    }

    // Puls-OFF zone: kræv solid kant — undgå GP2-fald kobling
    if (tdr_is_pulse_off_zone(edge_i)) {
        if (run < TDR_OPEN_MIN_RUN || ag < TDR_MIN_REFLECT_GRAD + 1)
            return false;
        return step >= 1;
    }

    // Under lang sendepuls: ~3 m åben refleksion (launch+4..+6) — kræv stærkere kant
    if (edge_i >= TDR_PULSE_ON + TDR_OPEN_DELTA_LO &&
        edge_i < g_capture_pulse_off) {
        int delta = edge_i - TDR_PULSE_ON;
        if (run >= 3)
            return true;
        if (run >= TDR_OPEN_MIN_RUN && ag >= 2 && step >= 1)
            return true;
        if (run >= TDR_OPEN_MIN_RUN && ag >= 1 && step >= 1 &&
            delta >= 4 && delta <= 7)
            return true;
        return false;
    }

    // Under aktiv puls (kort OFF): streng — typisk print-kobling uden kabel
    if (ag < TDR_MIN_REFLECT_GRAD)
        return false;
    if (run < TDR_OPEN_MIN_RUN)
        return false;
    if (edge_i <= g_capture_pulse_off && ag < TDR_MIN_REFLECT_GRAD + 1)
        return false;
    return true;
}

// Afslappet kantcheck til foretrukken ~3 m OPEN (launch+2..+10)
static bool tdr_reflect_edge_relaxed(const uint8_t *samples, int n, int edge_i,
                                     int grad) {
    if (edge_i < 1 || grad == 0)
        return false;
    if (tdr_is_pulse_off_sample(edge_i))
        return false;

    int ag   = tdr_abs_grad(grad);
    int run  = tdr_edge_run(samples, n, edge_i);
    int step = tdr_edge_level_step(samples, n, edge_i);
    if (run >= 1)
        return true;
    if (ag >= 1 && step >= 1)
        return true;
    return false;
}

// Print-kobling via 10k-divider — kun tidlig delta 1-3 (blokerer ikke ~3 m ved delta 4+)
static bool tdr_is_pcb_coupling(const uint8_t *samples, int n,
                                int launch_i, int edge_i, int grad) {
    if (launch_i < 0 || edge_i < 0)
        return false;

    if (tdr_gp3_pulse_width(samples, n) >= TDR_WIDE_CABLE_PULSE_MIN)
        return false;

    int delta = edge_i - launch_i;
    if (delta < TDR_OPEN_DELTA_LO || delta > 3)
        return false;

    int ag   = tdr_abs_grad(grad);
    int run  = tdr_edge_run(samples, n, edge_i);
    int step = tdr_edge_level_step(samples, n, edge_i);
    int q    = tdr_reflection_quality(samples, n, launch_i, edge_i, grad);

    if (edge_i > g_capture_pulse_off + 1 && run >= TDR_OPEN_MIN_RUN &&
        q >= TDR_OPEN_MIN_QUALITY)
        return false;
    if (ag >= 3 && run >= TDR_OPEN_MIN_RUN)
        return false;
    if (ag >= 2 && run >= 3 && step >= 2)
        return false;
    if (q >= TDR_OPEN_MIN_QUALITY + 4)
        return false;

    if (g_baseline_valid) {
        int win_diff = tdr_open_window_baseline_diff(samples, g_baseline_samples,
                                                       launch_i);
        if (win_diff < TDR_BASELINE_MIN_WIN_DIFF && q < TDR_CABLE_MIN_QUALITY)
            return true;
    }

    if (delta <= 2 && run < TDR_OPEN_MIN_RUN && ag < 3)
        return true;

    if (delta == 3 && run < 2 && ag < 3 && q < TDR_COUPLING_MAX_QUALITY)
        return true;

    return false;
}

// Launch = fast sende-sample (GPIO-capture: puls ON ved TDR_PULSE_ON)
static int tdr_find_launch(const uint8_t *samples, int n, int &out_g) {
    const int pulse = TDR_PULSE_ON;
    if (pulse < 1 || pulse >= n)
        return tdr_find_edge(samples, n, TDR_LAUNCH_SEARCH_LO, out_g);

    out_g = (int)samples[pulse] - (int)samples[pulse - 1];
    if (out_g == 0)
        out_g = 1;
    return pulse;
}

// Samples hvor GP3 er høj mens GP2-sendepuls er aktiv (GPIO-capture timing)
static int tdr_gp3_hi_during_send_pulse(const uint8_t *samples, int n) {
    int lo = TDR_PULSE_ON;
    int hi = g_capture_pulse_off - 1;
    if (lo < 0)
        lo = 0;
    if (hi >= n)
        hi = n - 1;
    if (hi < lo)
        return 0;

    int c = 0;
    for (int i = lo; i <= hi; i++) {
        if (samples[i])
            c++;
    }
    return c;
}

// Første stigende GP3-kant lige efter launch (0 = allerede høj ved launch)
static int tdr_gp3_rise_delta_after_launch(const uint8_t *samples, int n,
                                           int launch_i) {
    if (launch_i < 0 || launch_i >= n)
        return 99;

    if (samples[launch_i])
        return 0;

    int search_hi = launch_i + 2;
    if (search_hi >= n)
        search_hi = n - 1;
    for (int i = launch_i + 1; i <= search_hi; i++) {
        if (samples[i] > samples[i - 1])
            return i - launch_i;
    }
    return 99;
}

// GP3 lav lige efter launch, stigende kant ved refleksion delta 3-8 (~3 m åben)
static bool tdr_shot_delayed_open_rise(const uint8_t *samples, int n, int launch_i) {
    if (launch_i < 0 || launch_i + TDR_OPEN_DELTA_HI >= n)
        return false;

    int early_hi = 0;
    for (int i = launch_i + 1; i <= launch_i + 2; i++) {
        if (i < n && samples[i])
            early_hi++;
    }
    if (early_hi >= 2)
        return false;

    int lo = launch_i + TDR_STRONG_OPEN_DELTA_LO;
    int hi = launch_i + TDR_OPEN_DELTA_HI;
    if (hi >= n)
        hi = n - 1;
    for (int i = lo; i <= hi; i++) {
        if (i < 1)
            continue;
        if (samples[i] > samples[i - 1])
            return true;
    }
    return false;
}

// Stor stigning launch+1 → launch+3 (fx sample 11→13) = refleksion, ikke stik-kort
static bool tdr_shot_launch_refl_rise(const uint8_t *samples, int n, int launch_i) {
    int s1 = launch_i + 1;
    int s3 = launch_i + 3;
    if (s3 >= n || s1 < 0)
        return false;
    return !samples[s1] && samples[s3];
}

// GP3 lav launch+1..+2 (fx 11-12), stigende kant launch+3..+5 (fx 13-15) = ~3 m refleksion
static bool tdr_shot_reflection_after_quiet(const uint8_t *samples, int n,
                                            int launch_i) {
    int q_lo = launch_i + TDR_QUIET_REFL_LO;
    int q_hi = launch_i + TDR_QUIET_REFL_HI;
    if (q_hi >= n || q_lo < 0)
        return false;
    for (int i = q_lo; i <= q_hi; i++) {
        if (samples[i])
            return false;
    }
    int e_lo = launch_i + TDR_QUIET_EDGE_LO;
    int e_hi = launch_i + TDR_QUIET_EDGE_HI;
    if (e_hi >= n)
        e_hi = n - 1;
    for (int i = e_lo; i <= e_hi; i++) {
        if (i < 1)
            continue;
        if (samples[i] > samples[i - 1])
            return true;
    }
    return false;
}

static bool tdr_has_open_cable_reflection(const uint8_t *samples, int n,
                                          int launch_i);
static int  tdr_gp3_pulse_width(const uint8_t *samples, int n);
static int  tdr_find_open_zone_edge(const uint8_t *samples, int n,
                                    int launch_i, int &out_g);
static bool tdr_has_later_reflection_peak(const uint8_t *samples, int n,
                                          int launch_i);
static int  tdr_find_preferred_open_reflect(const uint8_t *samples, int n,
                                            int launch_i, int &out_g);
static int  tdr_find_late_open_peak(const uint8_t *samples, int n, int launch_i,
                                    int &out_g);
static int  tdr_find_calibrate_max_deriv(const uint8_t *samples, int n,
                                         int launch_i, int &out_g);
static int  tdr_find_calibrate_any_edge(const uint8_t *samples, int n,
                                        int launch_i, int &out_g);
static bool tdr_open_reflection_blocks_short(const uint8_t *samples, int n,
                                             int launch_i);
static bool tdr_shot_cable_end_reflection(const uint8_t *samples, int n,
                                          int launch_i);
static bool tdr_cable_end_short_evidence(float max_zone_dist_m, int median_delta,
                                         bool hw_short);
static float tdr_shot_late_refl_distance_m(const uint8_t *samples, int n,
                                           int launch_i);
static bool tdr_shot_forbids_connector_short(const uint8_t *samples, int n,
                                             int launch_i, float zone_dist_m);
static bool tdr_cycle_forbids_connector_short(float max_zone_dist_m,
                                              float max_strict_late_m,
                                              int median_delta);
static float tdr_shot_strict_late_refl_distance_m(const uint8_t *samples, int n,
                                                  int launch_i);

// GP3 høj straks ved launch og hele sendepuls (stik 5-6 kort / GP2→GP3)
static bool tdr_shot_gp3_immediate_follow(const uint8_t *samples, int n,
                                          int launch_i) {
    if (launch_i < 0 || launch_i >= n)
        return false;

    if (tdr_shot_delayed_open_rise(samples, n, launch_i))
        return false;
    if (tdr_shot_launch_refl_rise(samples, n, launch_i))
        return false;
    if (tdr_shot_reflection_after_quiet(samples, n, launch_i))
        return false;

    int rise_d = tdr_gp3_rise_delta_after_launch(samples, n, launch_i);
    if (rise_d > tdr_get_short_delta_max())
        return false;

    int pulse_len = g_capture_pulse_off - TDR_PULSE_ON;
    if (pulse_len < 4)
        return false;

    if (launch_i + 1 < n && !samples[launch_i + 1])
        return false;

    return tdr_gp3_hi_during_send_pulse(samples, n) >= pulse_len - 1;
}

// Stik-SHORT / GP3 følger fuld puls med delta<=1 — ingen sen OPEN-peak
static bool tdr_shot_blocks_late_open(const uint8_t *samples, int n,
                                      int launch_i) {
    if (tdr_is_connector_short(samples, n, launch_i, nullptr))
        return true;
    if (!tdr_shot_gp3_immediate_follow(samples, n, launch_i))
        return false;
    int rise_d = tdr_gp3_rise_delta_after_launch(samples, n, launch_i);
    return rise_d <= tdr_get_short_delta_max();
}

// Stik 5-6 kortsluttet: GP3 følger GP2 straks og holder hele sendepuls (ikke divider-kobling)
static bool tdr_is_connector_short(const uint8_t *samples, int n, int launch_i,
                                   int *out_reflect_i) {
    if (launch_i < 0 || launch_i >= n)
        return false;

    if (tdr_open_reflection_blocks_short(samples, n, launch_i))
        return false;
    if (tdr_has_later_reflection_peak(samples, n, launch_i))
        return false;

    int rise_d = tdr_gp3_rise_delta_after_launch(samples, n, launch_i);
    if (rise_d > tdr_get_short_delta_max())
        return false;

    int pulse_len = g_capture_pulse_off - TDR_PULSE_ON;
    if (pulse_len < 4)
        return false;

    int hi_cnt = tdr_gp3_hi_during_send_pulse(samples, n);
    int min_hi = pulse_len - 1;
    if (hi_cnt < min_hi)
        return false;

    int early_end = launch_i + 2;
    if (early_end >= g_capture_pulse_off)
        early_end = g_capture_pulse_off - 1;
    if (early_end >= n)
        early_end = n - 1;

    int early_hi = 0;
    int early_span = early_end - launch_i + 1;
    for (int i = launch_i; i <= early_end; i++) {
        if (samples[i])
            early_hi++;
    }

    if (rise_d == 0) {
        if (early_hi < 2)
            return false;
    } else {
        if (early_hi < early_span || hi_cnt < pulse_len)
            return false;
        if (g_baseline_valid) {
            int win_diff = tdr_open_window_baseline_diff(samples, g_baseline_samples,
                                                           launch_i);
            if (win_diff >= TDR_BASELINE_MIN_WIN_DIFF)
                return false;
        }
    }

    if (launch_i >= 2) {
        int pre_hi = 0;
        for (int i = launch_i - 2; i < launch_i; i++) {
            if (samples[i])
                pre_hi++;
        }
        if (pre_hi >= 2)
            return false;
    }

    if (out_reflect_i) {
        *out_reflect_i = (rise_d == 0) ? launch_i : launch_i + rise_d;
    }
    return true;
}

// Svag åben refleksion: solid kant i OPEN-delta-vinduet (ikke enkelt-sample støj)
static int tdr_find_open_zone_edge(const uint8_t *samples, int n,
                                   int launch_i, int &out_g) {
    if (launch_i < 0)
        return -1;

    int lo = launch_i + TDR_OPEN_DELTA_LO;
    int hi = launch_i + TDR_OPEN_DELTA_HI;
    if (hi >= n)
        hi = n - 1;

    int best_i = -1;
    int best_q = 0;
    for (int i = lo; i <= hi; i++) {
        if (i < 1 || tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        if (!tdr_reflect_edge_ok(samples, n, i, g))
            continue;
        int q = tdr_reflection_quality(samples, n, launch_i, i, g);
        if (q > best_q) {
            best_q = q;
            best_i = i;
            out_g  = g;
        }
    }
    return best_i;
}

// Åben refleksion delta 2-8 med kvalitet — blokér stik-SHORT (fx 3 m kabel)
static bool tdr_open_reflection_blocks_short(const uint8_t *samples, int n,
                                             int launch_i) {
    if (launch_i < 0)
        return false;

    // GP3 følger puls — kun ignorer OPEN-blok hvis ingen refleksion ved kabelende
    if (tdr_shot_gp3_immediate_follow(samples, n, launch_i) &&
        !tdr_shot_cable_end_reflection(samples, n, launch_i))
        return false;

    int zone_g = 0;
    int zone_i = tdr_find_open_zone_edge(samples, n, launch_i, zone_g);
    if (zone_i < 0)
        return false;

    int delta = zone_i - launch_i;
    if (delta < TDR_MEDIAN_OPEN_BLOCK_SHORT_LO || delta > TDR_OPEN_DELTA_HI)
        return false;
    if (!tdr_reflect_edge_ok(samples, n, zone_i, zone_g))
        return false;
    if (tdr_is_pcb_coupling(samples, n, launch_i, zone_i, zone_g))
        return false;

    return tdr_reflection_quality(samples, n, launch_i, zone_i, zone_g) >=
           TDR_OPEN_MIN_QUALITY;
}

static int tdr_open_zone_delta(const uint8_t *samples, int n, int launch_i,
                               int &out_g) {
    int zone_i = tdr_find_open_zone_edge(samples, n, launch_i, out_g);
    if (zone_i < 0 || launch_i < 0)
        return -1;
    return zone_i - launch_i;
}

// Sen refleksion delta>=2 (fx ~3 m ved sample 13-16) — blokér stik-SHORT
static bool tdr_has_later_reflection_peak(const uint8_t *samples, int n,
                                          int launch_i) {
    int zone_g = 0;
    int zd = tdr_open_zone_delta(samples, n, launch_i, zone_g);
    if (zd < TDR_DIST_MIN_DELTA)
        return false;
    int zone_i = launch_i + zd;
    if (!tdr_reflect_edge_ok(samples, n, zone_i, zone_g))
        return false;
    if (tdr_is_pcb_coupling(samples, n, launch_i, zone_i, zone_g))
        return false;
    return tdr_reflection_quality(samples, n, launch_i, zone_i, zone_g) >=
           TDR_OPEN_MIN_QUALITY;
}

// Refleksion ved kabelende (~3 m) — ikke stik 5-6 kort (0 m)
static bool tdr_shot_cable_end_reflection(const uint8_t *samples, int n,
                                          int launch_i) {
    if (launch_i < 0)
        return false;
    if (tdr_has_later_reflection_peak(samples, n, launch_i))
        return true;
    if (tdr_shot_reflection_after_quiet(samples, n, launch_i))
        return true;
    if (tdr_shot_delayed_open_rise(samples, n, launch_i))
        return true;
    if (tdr_shot_launch_refl_rise(samples, n, launch_i))
        return true;
    if (tdr_gp3_pulse_width(samples, n) >= TDR_WIDE_CABLE_PULSE_MIN) {
        int late_g = 0;
        int late_i = tdr_find_late_open_peak_strict(samples, n, launch_i, late_g);
        if (late_i >= 0 && (late_i - launch_i) >= TDR_DIST_MIN_DELTA)
            return true;
        late_i = tdr_find_late_open_peak(samples, n, launch_i, late_g);
        if (late_i >= 0 && (late_i - launch_i) >= TDR_DIST_MIN_DELTA)
            return true;
    }
    return false;
}

static bool tdr_cable_end_short_evidence(float max_zone_dist_m, int median_delta,
                                         bool hw_short) {
    if (!hw_short)
        return false;
    if (max_zone_dist_m >= TDR_SHORT_BLOCK_ZONE_DIST_M)
        return true;
    if (median_delta >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO)
        return true;
    return median_delta >= TDR_TYPICAL_3M_MEDIAN_LO &&
           median_delta <= TDR_TYPICAL_3M_MEDIAN_HI;
}

// Sen refleksionsafstand — strict peak, derefter bred-puls fallback (kabelende kort)
static float tdr_shot_late_refl_distance_m(const uint8_t *samples, int n,
                                           int launch_i) {
    if (launch_i < 0)
        return -1.0f;
    float dist = tdr_shot_strict_late_refl_distance_m(samples, n, launch_i);
    int late_g = 0;
    int late_i = tdr_find_late_open_peak(samples, n, launch_i, late_g);
    if (late_i < 0)
        return dist;
    int delta = late_i - launch_i;
    if (delta < TDR_DIST_MIN_DELTA)
        return dist;
    float d2 = tdr_distance_m_from_indices(launch_i, late_i);
    if (d2 > dist)
        return d2;
    return dist;
}

static bool tdr_shot_forbids_connector_short(const uint8_t *samples, int n,
                                             int launch_i, float zone_dist_m) {
    if (zone_dist_m >= TDR_SHORT_BLOCK_ZONE_DIST_M)
        return true;
    if (tdr_shot_late_refl_distance_m(samples, n, launch_i) >=
        TDR_SHORT_BLOCK_ZONE_DIST_M)
        return true;
    if (tdr_shot_cable_end_reflection(samples, n, launch_i))
        return true;
    return false;
}

static bool tdr_cycle_forbids_connector_short(float max_zone_dist_m,
                                              float max_strict_late_m,
                                              int median_delta) {
    if (max_zone_dist_m >= TDR_SHORT_BLOCK_ZONE_DIST_M)
        return true;
    if (max_strict_late_m >= TDR_SHORT_BLOCK_ZONE_DIST_M)
        return true;
    if (median_delta >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO)
        return true;
    return false;
}

// Foretræk delta 2-6 (~3 m): launch+2..+10 med afslappet kant; peak 12-20 ved kun d=1
static int tdr_find_preferred_open_reflect(const uint8_t *samples, int n,
                                           int launch_i, int &out_g) {
    if (launch_i < 0)
        return -1;

    const int pulse_w = tdr_gp3_pulse_width(samples, n);
    const bool wide_cable = pulse_w >= TDR_WIDE_CABLE_PULSE_MIN;
    bool cable_pulse = pulse_w >= TDR_CABLE_MIN_PULSE_WIDTH;

    // Bred puls + tidlig ri_raw (launch+1): sen peak launch+2 (~sample 12)
    if (wide_cable) {
        int raw_g = 0;
        int raw_i = tdr_find_open_zone_edge(samples, n, launch_i, raw_g);
        int raw_d = (raw_i >= 0) ? (raw_i - launch_i) : 99;
        if (raw_d <= TDR_SHORT_DELTA_MAX) {
            int late_g = 0;
            int late_i = tdr_find_late_open_peak(samples, n, launch_i, late_g);
            if (late_i >= 0) {
                out_g = late_g;
                return late_i;
            }
        }
    }

    int best_3m_i = -1;
    int best_3m_q = 0;
    int best_rel_i = -1;
    int best_rel_q = 0;

    auto consider = [&](int i, int g, bool relaxed) {
        if (i < 1 || tdr_is_pulse_off_sample(i))
            return;
        int delta = i - launch_i;
        if (delta < TDR_DIST_MIN_DELTA)
            return;
        if (relaxed) {
            if (delta > TDR_RELAXED_OPEN_DELTA_HI)
                return;
            if (!cable_pulse)
                return;
            if (delta > TDR_RELAXED_OPEN_DELTA_MID)
                return;
            if (!tdr_reflect_edge_relaxed(samples, n, i, g))
                return;
        } else {
            if (!tdr_reflect_edge_ok(samples, n, i, g))
                return;
        }
        if (!wide_cable && tdr_is_pcb_coupling(samples, n, launch_i, i, g))
            return;
        int q = tdr_reflection_quality(samples, n, launch_i, i, g);
        if (delta >= TDR_TYPICAL_3M_DELTA_LO && delta <= TDR_TYPICAL_3M_DELTA_HI) {
            if (q > best_3m_q) {
                best_3m_q = q;
                best_3m_i = i;
                out_g     = g;
            }
        } else if (delta >= TDR_RELAXED_OPEN_DELTA_LO &&
                   delta <= TDR_RELAXED_OPEN_DELTA_MID && q > best_rel_q) {
            best_rel_q = q;
            best_rel_i = i;
            out_g      = g;
        }
    };

    // Primær: launch+2..+10 med afslappet kant (delta 2-6, bred GP3-puls)
    int rel_hi = launch_i + TDR_RELAXED_OPEN_DELTA_HI;
    if (rel_hi >= n)
        rel_hi = n - 1;
    for (int i = launch_i + TDR_DIST_MIN_DELTA; i <= rel_hi; i++) {
        int g = (int)samples[i] - (int)samples[i - 1];
        if (g == 0)
            continue;
        consider(i, g, true);
    }

    if (best_3m_i >= 0)
        return best_3m_i;
    if (best_rel_i >= 0)
        return best_rel_i;

    // Mønster-hint (stille GP3 → sen stigning)
    bool pattern_3m = tdr_shot_reflection_after_quiet(samples, n, launch_i) ||
                      tdr_shot_delayed_open_rise(samples, n, launch_i) ||
                      tdr_shot_launch_refl_rise(samples, n, launch_i);
    if (pattern_3m) {
        int hint_lo = launch_i + TDR_STRONG_OPEN_DELTA_LO;
        int hint_hi = launch_i + TDR_OPEN_DELTA_HI;
        if (hint_hi >= n)
            hint_hi = n - 1;
        for (int i = hint_lo; i <= hint_hi; i++) {
            if (i < 1 || tdr_is_pulse_off_sample(i))
                continue;
            int g = (int)samples[i] - (int)samples[i - 1];
            if (g <= 0)
                continue;
            if (!tdr_reflect_edge_relaxed(samples, n, i, g))
                continue;
            consider(i, g, true);
        }
        if (best_3m_i >= 0)
            return best_3m_i;
        if (best_rel_i >= 0)
            return best_rel_i;
    }

    // Kun tidlig kant (d<=1) eller intet fund: søg peak sample 12-18, delta 2-5
    int zone_g = 0;
    int zone_i = tdr_find_open_zone_edge(samples, n, launch_i, zone_g);
    bool only_early = (zone_i < 0) ||
                      (zone_i >= 0 && (zone_i - launch_i) <= TDR_SHORT_DELTA_MAX);
    if (only_early || zone_i < 0) {
        int late_g = 0;
        int late_i = tdr_find_late_open_peak(samples, n, launch_i, late_g);
        if (late_i >= 0)
            return late_i;
    }
    if (only_early && cable_pulse) {
        int peak_lo = TDR_LATE_PEAK_SAMPLE_LO;
        int peak_hi = TDR_LATE_PEAK_SAMPLE_HI;
        if (peak_lo < launch_i + TDR_DIST_MIN_DELTA)
            peak_lo = launch_i + TDR_DIST_MIN_DELTA;
        if (peak_hi >= n)
            peak_hi = n - 1;
        int peak_best = -1;
        int peak_q    = 0;
        for (int i = peak_lo; i <= peak_hi; i++) {
            if (i < 1 || tdr_is_pulse_off_sample(i))
                continue;
            int g = (int)samples[i] - (int)samples[i - 1];
            if (g <= 0)
                continue;
            if (!tdr_reflect_edge_relaxed(samples, n, i, g))
                continue;
            int delta = i - launch_i;
            if (delta < TDR_RELAXED_OPEN_DELTA_LO ||
                delta > TDR_RELAXED_OPEN_DELTA_HI)
                continue;
            if (tdr_is_pcb_coupling(samples, n, launch_i, i, g))
                continue;
            int q = tdr_reflection_quality(samples, n, launch_i, i, g);
            if (q > peak_q) {
                peak_q    = q;
                peak_best = i;
                out_g     = g;
            }
        }
        if (peak_best >= 0)
            return peak_best;
    }

    // Standard kantcheck i OPEN-vindue
    int lo = launch_i + TDR_DIST_MIN_DELTA;
    int hi = launch_i + TDR_OPEN_DELTA_HI;
    if (hi >= n)
        hi = n - 1;
    for (int i = lo; i <= hi; i++) {
        if (i < 1 || tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        if (g == 0)
            continue;
        consider(i, g, false);
    }

    if (best_3m_i >= 0)
        return best_3m_i;
    if (best_rel_i >= 0)
        return best_rel_i;

    if (zone_i >= 0) {
        int zd = zone_i - launch_i;
        if (zd >= TDR_DIST_MIN_DELTA &&
            !tdr_is_pcb_coupling(samples, n, launch_i, zone_i, zone_g)) {
            out_g = zone_g;
            return zone_i;
        }
    }
    return -1;
}

static float tdr_shot_open_zone_distance_m(const uint8_t *samples, int n,
                                           int launch_i) {
    const bool wide_cable =
        tdr_gp3_pulse_width(samples, n) >= TDR_WIDE_CABLE_PULSE_MIN;
    if (tdr_shot_blocks_late_open(samples, n, launch_i) &&
        !tdr_shot_cable_end_reflection(samples, n, launch_i) &&
        !wide_cable)
        return -1.0f;
    int zone_g = 0;
    int zone_i = tdr_find_preferred_open_reflect(samples, n, launch_i, zone_g);
    if (zone_i < 0) {
        zone_i = tdr_find_late_open_peak(samples, n, launch_i, zone_g);
    }
    if (zone_i < 0)
        return -1.0f;
    TdrResult tmp{};
    tmp.launch_index  = launch_i;
    tmp.reflect_index = zone_i;
    tdr_fill_distance(tmp, launch_i, zone_i);
    return tmp.distance_m;
}

static float tdr_shot_strict_late_refl_distance_m(const uint8_t *samples, int n,
                                                  int launch_i) {
    if (launch_i < 0)
        return -1.0f;
    int late_g = 0;
    int late_i = tdr_find_late_open_peak_strict(samples, n, launch_i, late_g);
    if (late_i < 0)
        return -1.0f;
    int delta = late_i - launch_i;
    if (delta < TDR_DIST_MIN_DELTA)
        return -1.0f;
    return tdr_distance_m_from_indices(launch_i, late_i);
}

static float tdr_consensus_distance_m(const TdrResult *results, int median_delta,
                                        int reflect_median, int tries) {
    auto pick_dist_for_delta = [&](int target_d) -> float {
        if (target_d < TDR_DIST_MIN_DELTA || target_d >= 99)
            return -1.0f;
        float best = -1.0f;
        for (int i = 0; i < tries; i++) {
            if (!results[i].fault_found || results[i].launch_index < 0)
                continue;
            int d = results[i].reflect_index - results[i].launch_index;
            if (d < 0)
                d = 0;
            if (d != target_d)
                continue;
            float dm = tdr_distance_m_from_indices(results[i].launch_index,
                                                   results[i].reflect_index);
            if (dm < 0.0f)
                continue;
            if (dm > TDR_MAX_DISTANCE_M)
                dm = TDR_MAX_DISTANCE_M;
            if (best < 0.0f)
                best = dm;
        }
        return best;
    };

    if (reflect_median >= TDR_TYPICAL_3M_DELTA_LO &&
        reflect_median <= TDR_TYPICAL_3M_DELTA_HI) {
        float dm = pick_dist_for_delta(reflect_median);
        if (dm >= 0.0f)
            return dm;
    }
    if (reflect_median >= TDR_DIST_MIN_DELTA) {
        float dm = pick_dist_for_delta(reflect_median);
        if (dm >= 0.0f)
            return dm;
    }
    if (median_delta >= TDR_TYPICAL_3M_DELTA_LO &&
        median_delta <= TDR_TYPICAL_3M_DELTA_HI) {
        float dm = pick_dist_for_delta(median_delta);
        if (dm >= 0.0f)
            return dm;
    }
    if (median_delta >= TDR_DIST_MIN_DELTA && median_delta < 99) {
        float dm = pick_dist_for_delta(median_delta);
        if (dm >= 0.0f)
            return dm;
    }
    return -1.0f;
}

static bool tdr_distance_forces_open(float dist_m) {
    return dist_m > TDR_DIST_FORCE_OPEN_M;
}

static bool tdr_distance_allows_short(float dist_m) {
    return dist_m >= 0.0f && dist_m < TDR_DIST_SHORT_MAX_M;
}

// Stærkeste kant efter launch (fallback når solide kanter mangler — svag åben refleksion)
static int tdr_find_peak_edge_after_launch(const uint8_t *samples, int n,
                                           int launch_i, int &out_g) {
    if (launch_i < 0)
        return -1;

    int search_lo = launch_i + TDR_MIN_END_GAP;
    int search_hi = launch_i + TDR_MAX_END_DELTA;
    if (search_hi > launch_i + TDR_OPEN_DELTA_HI)
        search_hi = launch_i + TDR_OPEN_DELTA_HI;
    if (search_hi >= n)
        search_hi = n - 1;

    int best_i = -1;
    int best_abs = 0;
    int early_i = -1;
    int early_g = 0;
    for (int i = search_lo; i <= search_hi; i++) {
        if (tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        if (!tdr_reflect_edge_ok(samples, n, i, g))
            continue;
        int ag = tdr_abs_grad(g);
        int d  = i - launch_i;
        if (d <= TDR_SHORT_DELTA_MAX) {
            if (early_i < 0) {
                early_i = i;
                early_g = g;
            }
            continue;
        }
        if (d >= TDR_OPEN_DELTA_LO && d <= TDR_OPEN_DELTA_HI) {
            out_g = g;
            return i;
        }
        if (ag > best_abs) {
            best_abs = ag;
            best_i   = i;
            out_g    = g;
        }
    }
    if (best_i >= 0)
        return best_i;
    if (early_i >= 0) {
        out_g = early_g;
        return early_i;
    }
    return -1;
}

// Kalibrering: svag åben refleksion — udvidet vindue, kun aktiv puls-OFF udelades
static int tdr_find_calibrate_open_edge(const uint8_t *samples, int n,
                                        int launch_i, int &out_g) {
    if (launch_i < 0)
        return -1;

    int lo = launch_i + TDR_OPEN_DELTA_LO;
    int hi = tdr_calibrate_open_hi(launch_i);

    int best_i = -1;
    int best_score = 0;
    for (int i = lo; i <= hi; i++) {
        if (i < 1 || tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        if (g == 0)
            continue;
        int ag  = tdr_abs_grad(g);
        int run = tdr_edge_run(samples, n, i);
        int score = ag * 4 + run;
        if (score > best_score) {
            best_score = score;
            best_i     = i;
            out_g      = g;
        }
    }
    return best_i;
}

// Kalibrering: rå peak i udvidet vindue (grad>=1)
static int tdr_find_calibrate_raw_peak(const uint8_t *samples, int n,
                                       int launch_i, int &out_g) {
    if (launch_i < 0)
        return -1;

    int lo = launch_i + TDR_OPEN_DELTA_LO;
    int hi = tdr_calibrate_open_hi(launch_i);

    int best_i = -1;
    int best_score = 0;
    for (int i = lo; i <= hi; i++) {
        if (i < 1 || tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        if (g == 0)
            continue;
        int ag  = tdr_abs_grad(g);
        int run = tdr_edge_run(samples, n, i);
        int step = tdr_edge_level_step(samples, n, i);
        int score = ag * 8 + run * 3 + step;
        if (score > best_score) {
            best_score = score;
            best_i     = i;
            out_g      = g;
        }
    }
    return best_i;
}

// Kalibrering: største absolutte gradient i vinduet (max derivative)
static int tdr_find_calibrate_max_deriv(const uint8_t *samples, int n,
                                        int launch_i, int &out_g) {
    if (launch_i < 0)
        return -1;

    int lo = launch_i + TDR_OPEN_DELTA_LO;
    int hi = tdr_calibrate_open_hi(launch_i);

    int best_i = -1;
    int best_ag = 0;
    for (int i = lo; i <= hi; i++) {
        if (i < 1 || tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        int ag = tdr_abs_grad(g);
        if (ag >= 1 && ag >= best_ag) {
            best_ag = ag;
            best_i  = i;
            out_g   = g;
        }
    }
    return best_i;
}

// Kalibrering: første stigende kant efter launch (monoton rise)
static int tdr_find_calibrate_monotonic_rise(const uint8_t *samples, int n,
                                             int launch_i, int &out_g) {
    if (launch_i < 0)
        return -1;

    int lo = launch_i + TDR_OPEN_DELTA_LO;
    int hi = tdr_calibrate_open_hi(launch_i);

    for (int i = lo; i <= hi; i++) {
        if (i < 1 || tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        if (g > 0) {
            out_g = g;
            return i;
        }
    }
    return -1;
}

// Kalibrering: enhver kant (stigende eller faldende) i delta 1-8
static int tdr_find_calibrate_any_edge(const uint8_t *samples, int n,
                                       int launch_i, int &out_g) {
    if (launch_i < 0)
        return -1;

    int lo = launch_i + TDR_OPEN_DELTA_LO;
    int hi = launch_i + TDR_OPEN_DELTA_HI;
    if (hi >= n)
        hi = n - 1;

    for (int i = lo; i <= hi; i++) {
        if (i < 1 || tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        if (g != 0) {
            out_g = g;
            return i;
        }
    }
    return -1;
}

// Første solide kant efter launch — ignorer puls-fald ved TDR_PULSE_OFF
static int tdr_find_end_after_launch(const uint8_t *samples, int n,
                                     int launch_i, int &out_g) {
    if (launch_i < 0)
        return -1;

    int search_lo = launch_i + TDR_MIN_END_GAP;
    int search_hi = launch_i + TDR_OPEN_DELTA_HI;
    if (search_hi >= n)
        search_hi = n - 1;

    int early_i = -1;
    int early_g = 0;
    for (int i = search_lo; i <= search_hi; i++) {
        if (tdr_is_pulse_off_sample(i))
            continue;
        int g = (int)samples[i] - (int)samples[i - 1];
        if (!tdr_reflect_edge_ok(samples, n, i, g))
            continue;
        int d = i - launch_i;
        if (d <= TDR_SHORT_DELTA_MAX) {
            if (early_i < 0) {
                early_i = i;
                early_g = g;
            }
            continue;
        }
        out_g = g;
        return i;
    }

    if (early_i >= 0) {
        out_g = early_g;
        return early_i;
    }

    out_g = 0;
    return -1;
}

// ------------------------------------------------------------
// Sample-periode
// ------------------------------------------------------------
float tdr_get_sample_period_ns() {
    return g_capture_sample_period_ns;
}

// ------------------------------------------------------------
// Velocity factor
// ------------------------------------------------------------
void tdr_set_velocity_factor(float vf) {
    g_velocity_factor = vf;
}

float tdr_get_velocity_factor() {
    return g_velocity_factor;
}

void tdr_set_calibration(const TdrCalibState &cal) {
    g_cal_short_zero   = cal.short_zero_delta;
    g_cal_load_delta   = cal.load100_delta;
    g_cal_short_valid  = cal.short_valid;
    g_cal_load_valid   = cal.load_valid;
}

void tdr_get_calibration(TdrCalibState &cal) {
    cal.short_zero_delta = g_cal_short_zero;
    cal.load100_delta    = g_cal_load_delta;
    cal.short_valid      = g_cal_short_valid;
    cal.load_valid       = g_cal_load_valid;
}

// ------------------------------------------------------------
// Capture samples
// ------------------------------------------------------------
static uint32_t tdr_ns_to_cycles(uint32_t ns) {
    uint64_t cycles =
        ((uint64_t)ns * (uint64_t)clock_get_hz(clk_sys) + 999999999ull) /
        1000000000ull;
    if (cycles < 1)
        cycles = 1;
    return (uint32_t)cycles;
}

static void tdr_delay_cycles(uint32_t cycles) {
    while (cycles--) {
        __asm volatile("nop");
    }
}

static void tdr_delay_ns(uint32_t ns) {
    uint32_t cycles = tdr_ns_to_cycles(ns);
    if (cycles > TDR_SAMPLE_LOOP_OVERHEAD_CYCLES)
        cycles -= (uint32_t)TDR_SAMPLE_LOOP_OVERHEAD_CYCLES;
    else
        cycles = 1;
    tdr_delay_cycles(cycles);
}

static float tdr_cycles_to_ns(uint32_t cycles) {
    float hz = (float)clock_get_hz(clk_sys);
    return ((float)cycles * 1e9f) / hz;
}

static inline bool tdr_read_in_pin() {
    return (sio_hw->gpio_in >> TDR_IN_PIN) & 1u;
}

static void tdr_restore_pio_pins() {
    pio_gpio_init(g_pio, TDR_OUT_PIN);
    pio_gpio_init(g_pio, TDR_IN_PIN);
    pio_sm_set_pindirs_with_mask(
        g_pio, g_sm,
        (1u << TDR_OUT_PIN),
        (1u << TDR_OUT_PIN)
    );
    pio_sm_set_pindirs_with_mask(
        g_pio, g_sm,
        0,
        (1u << TDR_IN_PIN)
    );
}

enum class TdrPull {
    None = 0,
    Up   = 1,
    Down = 2,
};

static void tdr_set_in_pull(TdrPull pull) {
    gpio_set_dir(TDR_IN_PIN, GPIO_IN);
    gpio_disable_pulls(TDR_IN_PIN);
    if (pull == TdrPull::Up)
        gpio_pull_up(TDR_IN_PIN);
    else if (pull == TdrPull::Down)
        gpio_pull_down(TDR_IN_PIN);
}

// Sample GP3 mens GP2 pulser (GP3-check holder GP2 høj — TDR gjorde ikke det)
static bool tdr_capture_gpio_pull(TdrPull pull) {
    pio_sm_set_enabled(g_pio, g_sm, false);

    gpio_set_function(TDR_OUT_PIN, GPIO_FUNC_SIO);
    gpio_set_function(TDR_IN_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(TDR_OUT_PIN, GPIO_OUT);
    tdr_set_in_pull(pull);

    bool raw[128];

    gpio_put(TDR_OUT_PIN, 0);
    busy_wait_us(20);
    bool idle_hi = gpio_get(TDR_IN_PIN);

    // Grov delay (50 ns) gør ~2 m kabel til delta 0–1 → 0.0 m + Unstable
    uint32_t delay_ns = (uint32_t)tdr_get_sample_period_ns();
    if (delay_ns < 10)
        delay_ns = 10;
    g_capture_sample_period_ns = (float)delay_ns;

    const int pulse_on  = TDR_PULSE_ON;
    const int pulse_off = g_capture_pulse_off;

    bool saw_hi = false;
    bool saw_lo = false;

    const uint32_t fast_cycles =
        (uint32_t)TDR_SAMPLE_LOOP_OVERHEAD_CYCLES + (uint32_t)TDR_FAST_EXTRA_CYCLES;
    float fast_step_ns_f = tdr_cycles_to_ns(fast_cycles);

    // Mål faktisk hurtig sample-periode (4096 samples → ~60 µs, pålidelig bench_us)
    {
        absolute_time_t t0 = get_absolute_time();
        for (int k = 0; k < TDR_BENCH_FAST_SAMPLES; k++) {
            if (k == pulse_on)
                gpio_put(TDR_OUT_PIN, 1);
            else if (k == pulse_off)
                gpio_put(TDR_OUT_PIN, 0);
            (void)tdr_read_in_pin();
        }
        gpio_put(TDR_OUT_PIN, 0);
        int64_t bench_us = absolute_time_diff_us(t0, get_absolute_time());
        if (bench_us > 0) {
            float measured =
                (float)bench_us * 1000.0f / (float)TDR_BENCH_FAST_SAMPLES;
            if (measured < TDR_FAST_PERIOD_MIN_NS)
                measured = TDR_FAST_PERIOD_MIN_NS;
            if (measured > TDR_FAST_PERIOD_MAX_NS)
                measured = TDR_FAST_PERIOD_MAX_NS;
            fast_step_ns_f = measured;
        }
    }
    if (fast_step_ns_f < TDR_FAST_PERIOD_MIN_NS)
        fast_step_ns_f = TDR_FAST_PERIOD_MIN_NS;
    if (fast_step_ns_f > TDR_FAST_PERIOD_MAX_NS)
        fast_step_ns_f = TDR_FAST_PERIOD_MAX_NS;

    const uint32_t fast_step_ns = (uint32_t)fast_step_ns_f;
    uint32_t t_cum_ns           = 0;

    for (int i = 0; i < 128; i++) {
        if (i == pulse_on)
            gpio_put(TDR_OUT_PIN, 1);
        else if (i == pulse_off)
            gpio_put(TDR_OUT_PIN, 0);

        g_sample_time_ns[i] = t_cum_ns;

        bool hi = tdr_read_in_pin();
        raw[i] = hi;
        if (hi)
            saw_hi = true;
        else
            saw_lo = true;

        if (i < TDR_FAST_CAPTURE_END)
            t_cum_ns += fast_step_ns;
        else
            t_cum_ns += delay_ns;
    }

    g_capture_sample_period_ns = (float)fast_step_ns;

    gpio_put(TDR_OUT_PIN, 0);
    tdr_restore_pio_pins();

    if (!saw_hi && !saw_lo)
        return false;

    const bool try_inv[] = { idle_hi, !idle_hi };
    for (bool invert : try_inv) {
        for (int i = 0; i < 128; i++)
            g_samples[i] = (raw[i] != invert) ? 1 : 0;
        if (tdr_samples_vary(g_samples, 128)) {
            g_invert_in = invert;
            return true;
        }
    }

    // Kortslutning / aktiv refleksion kan give kun kort HI-puls
    if (saw_hi && saw_lo) {
        for (int i = 0; i < 128; i++)
            g_samples[i] = raw[i] ? 1 : 0;
        g_invert_in = false;
        return true;
    }

    return false;
}

// Samme timing som puls-capture, men GP2 holdes lav (divider-baseline)
static bool tdr_capture_baseline_gpio(TdrPull pull) {
    pio_sm_set_enabled(g_pio, g_sm, false);

    gpio_set_function(TDR_OUT_PIN, GPIO_FUNC_SIO);
    gpio_set_function(TDR_IN_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(TDR_OUT_PIN, GPIO_OUT);
    tdr_set_in_pull(pull);

    bool raw[128];

    gpio_put(TDR_OUT_PIN, 0);
    busy_wait_us(20);
    bool idle_hi = gpio_get(TDR_IN_PIN);

    const uint32_t fast_cycles =
        (uint32_t)TDR_SAMPLE_LOOP_OVERHEAD_CYCLES + (uint32_t)TDR_FAST_EXTRA_CYCLES;
    float fast_step_ns_f = tdr_cycles_to_ns(fast_cycles);
    if (fast_step_ns_f < TDR_FAST_PERIOD_MIN_NS)
        fast_step_ns_f = TDR_FAST_PERIOD_MIN_NS;
    if (fast_step_ns_f > TDR_FAST_PERIOD_MAX_NS)
        fast_step_ns_f = TDR_FAST_PERIOD_MAX_NS;

    const uint32_t fast_step_ns = (uint32_t)fast_step_ns_f;
    uint32_t delay_ns = (uint32_t)g_capture_sample_period_ns;
    if (delay_ns < 10)
        delay_ns = 10;

    bool saw_hi = false;
    bool saw_lo = false;

    for (int i = 0; i < 128; i++) {
        gpio_put(TDR_OUT_PIN, 0);
        bool hi = tdr_read_in_pin();
        raw[i] = hi;
        if (hi)
            saw_hi = true;
        else
            saw_lo = true;
        (void)fast_step_ns;
        (void)delay_ns;
    }

    gpio_put(TDR_OUT_PIN, 0);
    tdr_restore_pio_pins();

    if (!saw_hi && !saw_lo)
        return false;

    const bool try_inv[] = { g_invert_in, idle_hi, !idle_hi };
    for (bool invert : try_inv) {
        for (int i = 0; i < 128; i++)
            g_baseline_samples[i] = (raw[i] != invert) ? 1 : 0;
        if (tdr_samples_vary(g_baseline_samples, 128))
            return true;
    }

    if (saw_hi && saw_lo) {
        for (int i = 0; i < 128; i++)
            g_baseline_samples[i] = raw[i] ? 1 : 0;
        return true;
    }
    return false;
}

static void tdr_capture_ex(int pulse_off) {
    if (!g_tdr_active)
        return;

    g_capture_pulse_off = pulse_off;
    g_baseline_valid = false;

    const TdrPull pulls[] = {
        TdrPull::Up,
        TdrPull::None,
        TdrPull::Down,
    };

    for (TdrPull pull : pulls) {
        if (tdr_capture_gpio_pull(pull)) {
            if (tdr_capture_baseline_gpio(pull))
                g_baseline_valid = true;
            return;
        }
    }

    memset(g_samples, 0, sizeof(g_samples));
    memset(g_baseline_samples, 0, sizeof(g_baseline_samples));
    memset(g_sample_time_ns, 0, sizeof(g_sample_time_ns));
}

static void tdr_capture() {
    tdr_capture_ex(TDR_PULSE_OFF);
}

static void tdr_capture_for_calibrate() {
    tdr_capture();
}

static void tdr_gpio_sio_begin() {
    pio_sm_set_enabled(g_pio, g_sm, false);
    gpio_set_function(TDR_OUT_PIN, GPIO_FUNC_SIO);
    gpio_set_function(TDR_IN_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(TDR_OUT_PIN, GPIO_OUT);
    gpio_set_dir(TDR_IN_PIN, GPIO_IN);
}

// MIC 6-pin 1-3 (GP10-12) — ikke TDR; kortslutning (fx pin 2-3) kobler falsk refleksion på GP3
static const uint TDR_MIC_PIN1 = 10;
static const uint TDR_MIC_PIN2 = 11;
static const uint TDR_MIC_PIN3 = 12;

static void tdr_mic_pins_release() {
    gpio_init(TDR_MIC_PIN1);
    gpio_init(TDR_MIC_PIN2);
    gpio_init(TDR_MIC_PIN3);
    gpio_set_dir(TDR_MIC_PIN1, GPIO_IN);
    gpio_set_dir(TDR_MIC_PIN2, GPIO_IN);
    gpio_set_dir(TDR_MIC_PIN3, GPIO_IN);
    gpio_disable_pulls(TDR_MIC_PIN1);
    gpio_disable_pulls(TDR_MIC_PIN2);
    gpio_disable_pulls(TDR_MIC_PIN3);
}

// Aktiv drive-test (som mic_test::test_short_pair) — kun ved reel kortslutning
static bool tdr_mic_pair_shorted(uint a, uint b) {
    gpio_init(a);
    gpio_init(b);
    gpio_set_dir(a, GPIO_IN);
    gpio_set_dir(b, GPIO_IN);
    gpio_pull_down(a);
    gpio_pull_down(b);
    busy_wait_us(30);

    gpio_set_dir(a, GPIO_OUT);
    gpio_put(a, 1);
    busy_wait_us(100);
    bool ab = gpio_get(b);

    gpio_put(a, 0);
    gpio_set_dir(a, GPIO_IN);
    busy_wait_us(30);

    gpio_set_dir(b, GPIO_OUT);
    gpio_put(b, 1);
    busy_wait_us(100);
    bool ba = gpio_get(a);

    gpio_put(b, 0);
    gpio_set_dir(b, GPIO_IN);
    gpio_disable_pulls(a);
    gpio_disable_pulls(b);

    return ab || ba;
}

static bool tdr_mic_connector_short_once() {
    gpio_init(TDR_MIC_PIN1);
    gpio_init(TDR_MIC_PIN2);
    gpio_init(TDR_MIC_PIN3);
    gpio_set_dir(TDR_MIC_PIN1, GPIO_IN);
    gpio_set_dir(TDR_MIC_PIN2, GPIO_IN);
    gpio_set_dir(TDR_MIC_PIN3, GPIO_IN);
    gpio_pull_down(TDR_MIC_PIN1);
    gpio_pull_down(TDR_MIC_PIN2);
    gpio_pull_down(TDR_MIC_PIN3);
    busy_wait_us(30);

    if (tdr_mic_pair_shorted(TDR_MIC_PIN1, TDR_MIC_PIN2))
        return true;
    if (tdr_mic_pair_shorted(TDR_MIC_PIN1, TDR_MIC_PIN3))
        return true;
    if (tdr_mic_pair_shorted(TDR_MIC_PIN2, TDR_MIC_PIN3))
        return true;

    tdr_mic_pins_release();
    return false;
}

static bool tdr_mic_connector_short() {
    int hits = 0;
    for (int t = 0; t < 3; t++) {
        if (tdr_mic_connector_short_once())
            hits++;
        busy_wait_us(40);
    }
    tdr_mic_pins_release();
    return hits >= 2;
}

// Flydende GP3 (ingen 6-pin / intet stik) toggler ofte uden puls
static bool tdr_gp3_idle_stable() {
    tdr_gpio_sio_begin();
    gpio_disable_pulls(TDR_IN_PIN);
    gpio_put(TDR_OUT_PIN, 0);
    busy_wait_us(100);

    int edges = 0;
    bool last = gpio_get(TDR_IN_PIN);
    for (int i = 0; i < 24; i++) {
        busy_wait_us(20);
        bool v = gpio_get(TDR_IN_PIN);
        if (v != last) {
            edges++;
            last = v;
        }
    }

    gpio_put(TDR_OUT_PIN, 0);
    tdr_restore_pio_pins();
    return edges <= TDR_IDLE_STABLE_MAX_EDGES;
}

// Kort test: virker GP2→GP3 på printet? (kortslut med tang på Pico)
// Med 10k+10k divider er GP3 lav ved GP2=0 (lo=false) — det er normalt, ikke "stuck".
// Langt åbent kabel (RC) kan give GP23_OPEN ved hurtig test — brug slow_cable ved kalibrering.
static void tdr_run_hw_diag(TdrResult &r, bool slow_cable = false) {
    r.diag = TDR_DIAG_NONE;
    if (!g_tdr_active)
        return;

    const int settle_us = slow_cable ? TDR_DIAG_SLOW_SETTLE_US : TDR_DIAG_SETTLE_US;
    const int drive_us  = slow_cable ? TDR_DIAG_SLOW_DRIVE_US  : TDR_DIAG_DRIVE_US;

    tdr_gpio_sio_begin();
    gpio_disable_pulls(TDR_IN_PIN);

    gpio_put(TDR_OUT_PIN, 0);
    busy_wait_us(settle_us);
    bool lo = gpio_get(TDR_IN_PIN);

    gpio_put(TDR_OUT_PIN, 1);
    busy_wait_us(drive_us);
    bool hi = gpio_get(TDR_IN_PIN);
    if (!hi) {
        busy_wait_us(drive_us);
        hi = gpio_get(TDR_IN_PIN);
    }
    if (slow_cable && !hi) {
        busy_wait_us(drive_us);
        hi = gpio_get(TDR_IN_PIN);
    }

    gpio_put(TDR_OUT_PIN, 0);
    tdr_restore_pio_pins();

    if (hi && !lo)
        r.diag = TDR_DIAG_GP23_OK;
    else if (lo && hi)
        r.diag = TDR_DIAG_GP3_STUCK;   // GP3 høj selv når GP2 er lav
    else
        r.diag = TDR_DIAG_GP23_OPEN;   // følger ikke GP2 (åben sti / langsom kabel)
}

// Stik 5-6 kort: GP3 lav ved GP2=0 og høj ved GP2=1 (inden for ~2,5 ms drive)
static bool tdr_hw_connector_shorted() {
    if (!g_tdr_active)
        return false;

    tdr_gpio_sio_begin();
    gpio_disable_pulls(TDR_IN_PIN);

    gpio_put(TDR_OUT_PIN, 0);
    busy_wait_us(TDR_DIAG_SETTLE_US);
    bool lo = gpio_get(TDR_IN_PIN);

    gpio_put(TDR_OUT_PIN, 1);
    busy_wait_us(TDR_DIAG_DRIVE_US);
    bool hi = gpio_get(TDR_IN_PIN);

    gpio_put(TDR_OUT_PIN, 0);
    tdr_restore_pio_pins();

    return hi && !lo;
}

// Stik 4-5 (GND→puls) eller 4-6 (GND→sense): GP3 stabil lav, følger ikke GP2-puls.
// Ikke stik 5-6 kort (det giver GP3 høj ved GP2=1). Tomt stik har ofte flydende GP3.
static bool tdr_hw_ground_fault() {
    if (!g_tdr_active)
        return false;
    if (tdr_hw_connector_shorted())
        return false;
    if (tdr_mic_connector_short())
        return false;
    if (!tdr_gp3_idle_stable())
        return false;

    tdr_gpio_sio_begin();
    gpio_disable_pulls(TDR_IN_PIN);

    gpio_put(TDR_OUT_PIN, 0);
    busy_wait_us(TDR_DIAG_SETTLE_US);
    (void)gpio_get(TDR_IN_PIN);

    gpio_put(TDR_OUT_PIN, 1);
    busy_wait_us(TDR_DIAG_DRIVE_US);
    bool hi = gpio_get(TDR_IN_PIN);
    if (!hi) {
        busy_wait_us(TDR_DIAG_DRIVE_US);
        hi = gpio_get(TDR_IN_PIN);
    }

    gpio_put(TDR_OUT_PIN, 0);
    tdr_restore_pio_pins();

    return !hi;
}

// ------------------------------------------------------------
// Gradient-refleksionsdetektor
// ------------------------------------------------------------
static TdrResult tdr_detect_reflection(const uint8_t *samples, int n,
                                       bool for_calibrate = false,
                                       TdrCalibType cal_type = TdrCalibType::Open)
{
    TdrResult r{};
    r.reflect_index = -1;
    r.launch_index  = -1;
    const bool allow_short =
        !for_calibrate || tdr_short_cal_active(for_calibrate, cal_type);

    if (!tdr_samples_vary(samples, n))
        return r;

    int edges = tdr_count_edges(samples, n);
    if (edges > (for_calibrate ? TDR_CALIBRATE_MAX_EDGES : 12))
        return r;   // for meget støj (typisk berøring)

    int launch_g = 0;
    int launch_i = tdr_find_launch(samples, n, launch_g);
    if (launch_i < 0)
        return r;

    if (allow_short &&
        !tdr_open_reflection_blocks_short(samples, n, launch_i)) {
        int short_refl = launch_i;
        if (tdr_is_connector_short(samples, n, launch_i, &short_refl) &&
            !tdr_shot_forbids_connector_short(samples, n, launch_i, -1.0f)) {
            r.fault_found   = true;
            r.launch_index  = launch_i;
            r.reflect_index = short_refl;
            r.is_short      = true;
            r.distance_m    = 0.0f;
            return r;
        }
    }

    int best_i = -1;
    int best_g = 0;
    bool launch_only = false;

    // Åben kabel: foretræk delta 3-5 (~3 m), ikke delta-1 kobling
    {
        int zone_g = 0;
        int zone_i = tdr_find_preferred_open_reflect(samples, n, launch_i, zone_g);
        if (zone_i >= 0) {
            best_i = zone_i;
            best_g = zone_g;
        }
    }

    if (best_i < 0) {
        int end_g = 0;
        int end_i = tdr_find_end_after_launch(samples, n, launch_i, end_g);
        if (end_i >= 0) {
            best_i = end_i;
            best_g = end_g;
        }
    }

    if (best_i < 0) {
        int peak_g = 0;
        int peak_i = tdr_find_peak_edge_after_launch(samples, n, launch_i, peak_g);
        if (peak_i >= 0) {
            best_i = peak_i;
            best_g = peak_g;
        }
    }

    if (best_i < 0 && for_calibrate) {
        int cal_g = 0;
        int cal_i = tdr_find_calibrate_open_edge(samples, n, launch_i, cal_g);
        if (cal_i >= 0) {
            best_i = cal_i;
            best_g = cal_g;
        }
    }

    if (best_i < 0 && for_calibrate) {
        int raw_g = 0;
        int raw_i = tdr_find_calibrate_raw_peak(samples, n, launch_i, raw_g);
        if (raw_i >= 0) {
            best_i = raw_i;
            best_g = raw_g;
        }
    }

    if (best_i < 0 && for_calibrate) {
        int md_g = 0;
        int md_i = tdr_find_calibrate_max_deriv(samples, n, launch_i, md_g);
        if (md_i >= 0) {
            best_i = md_i;
            best_g = md_g;
        }
    }

    if (best_i < 0 && for_calibrate) {
        int mr_g = 0;
        int mr_i = tdr_find_calibrate_monotonic_rise(samples, n, launch_i, mr_g);
        if (mr_i >= 0) {
            best_i = mr_i;
            best_g = mr_g;
        }
    }

    if (best_i < 0 && for_calibrate) {
        int any_g = 0;
        int any_i = tdr_find_calibrate_any_edge(samples, n, launch_i, any_g);
        if (any_i >= 0) {
            best_i = any_i;
            best_g = any_g;
        }
    }

    // Kun launch-kant synlig → typisk kortslutning ved stik (ikke åben 2 m kabel)
    if (best_i < 0) {
        if (launch_i >= TDR_LAUNCH_SEARCH_LO && launch_i <= TDR_LAUNCH_SEARCH_HI) {
            best_i = launch_i;
            best_g = launch_g;
            launch_only = true;
        } else {
            return r;
        }
    }

    int delta = best_i - launch_i;
    if (delta < 0)
        delta = 0;

    if (best_g == 0)
        return r;

    // Afvis puls-kobling / divider-støj (kun tidlig delta 1-3)
    bool reject_edge = false;
    if (!for_calibrate) {
        bool cable_pulse =
            tdr_gp3_pulse_width(samples, n) >= TDR_CABLE_MIN_PULSE_WIDTH;
        bool edge_ok = tdr_reflect_edge_ok(samples, n, best_i, best_g) ||
                       (cable_pulse && delta >= TDR_RELAXED_OPEN_DELTA_LO &&
                        delta <= TDR_RELAXED_OPEN_DELTA_MID &&
                        tdr_reflect_edge_relaxed(samples, n, best_i, best_g));
        reject_edge = !edge_ok ||
                      tdr_is_pcb_coupling(samples, n, launch_i, best_i, best_g);
    } else if (tdr_is_pulse_off_sample(best_i)) {
        reject_edge = true;
    } else if (best_i <= g_capture_pulse_off + 1 && best_g != 0) {
        int run = tdr_edge_run(samples, n, best_i);
        reject_edge = (run < 1 && tdr_abs_grad(best_g) < 1);
    }
    if (reject_edge) {
        if (!tdr_gp3_idle_stable())
            r.unstable = true;
        return r;
    }

    int delta_pre = best_i - launch_i;
    if (delta_pre < 0)
        delta_pre = 0;

    r.fault_found   = true;
    r.launch_index  = launch_i;
    r.reflect_index = best_i;
    tdr_fill_distance(r, launch_i, best_i);

    // Afvis sen støj-kant / urealistisk længde for 10k-TDR (typisk <5 m)
    if (r.distance_m > TDR_MAX_DISTANCE_M) {
        int zone_g = 0;
        int zone_i = tdr_find_open_zone_edge(samples, n, launch_i, zone_g);
        if (zone_i >= 0) {
            r.reflect_index = zone_i;
            tdr_fill_distance(r, launch_i, zone_i);
        }
    }
    if (r.distance_m > TDR_MAX_DISTANCE_M) {
        r.fault_found   = false;
        r.reflect_index = -1;
        r.distance_m    = 0.0f;
        return r;
    }

    // Resistor-TDR: åben ende kan give faldende kant — brug delta, ikke polaritet alene
    if (launch_only) {
        // Kun sendepuls synlig: kun reel stik 5-6 kort (GP3 følger GP2 under puls)
        r.is_short = tdr_is_connector_short(samples, n, launch_i, nullptr);
        if (r.is_short) {
            r.distance_m = 0.0f;
        } else {
            r.is_short = false;
            int zone_g = 0;
            int zone_i = -1;
            if (for_calibrate) {
                zone_i = tdr_find_calibrate_open_edge(samples, n, launch_i, zone_g);
                if (zone_i < 0)
                    zone_i = tdr_find_calibrate_raw_peak(samples, n, launch_i, zone_g);
                if (zone_i < 0)
                    zone_i = tdr_find_calibrate_max_deriv(samples, n, launch_i, zone_g);
                if (zone_i < 0)
                    zone_i = tdr_find_calibrate_monotonic_rise(samples, n, launch_i, zone_g);
                if (zone_i < 0)
                    zone_i = tdr_find_calibrate_any_edge(samples, n, launch_i, zone_g);
            }
            if (zone_i < 0)
                zone_i = tdr_find_open_zone_edge(samples, n, launch_i, zone_g);
            if (zone_i >= 0 &&
                (for_calibrate ||
                 (tdr_reflect_edge_ok(samples, n, zone_i, zone_g) &&
                  !tdr_is_pcb_coupling(samples, n, launch_i, zone_i, zone_g)))) {
                r.reflect_index = zone_i;
                delta = zone_i - launch_i;
                tdr_fill_distance(r, launch_i, zone_i);
            } else {
                // Ingen tydelig refleksion — rapporter ikke 0.0 m som åben længde
                r.fault_found = false;
                r.reflect_index = -1;
                r.distance_m    = 0.0f;
            }
        }
    } else if (delta >= TDR_OPEN_DELTA_LO &&
               delta <= (for_calibrate ? TDR_CALIBRATE_OPEN_DELTA_HI
                                       : TDR_OPEN_DELTA_HI)) {
        // Kort kabel (≈1-4 m): tvang OPEN — polaritet upålidelig på 100Ω/10k TDR
        r.is_short = false;
        if (!for_calibrate &&
            tdr_is_pcb_coupling(samples, n, launch_i, best_i, best_g)) {
            r.fault_found   = false;
            r.reflect_index = -1;
            r.distance_m    = 0.0f;
            if (!tdr_gp3_idle_stable())
                r.unstable = true;
            return r;
        }
        r.unstable = false;
    } else if (delta <= tdr_get_short_delta_max()) {
        r.is_short = tdr_is_connector_short(samples, n, launch_i, nullptr);
        if (r.is_short)
            r.distance_m = 0.0f;
    } else {
        r.is_short = false;
    }

    if (!for_calibrate && r.fault_found && !r.is_short) {
        int d_chk = r.reflect_index - r.launch_index;
        if (tdr_reflect_matches_load100(d_chk) &&
            tdr_gp3_pulse_width(samples, n) < TDR_WIDE_CABLE_PULSE_MIN) {
            r.fault_found   = false;
            r.reflect_index = -1;
            r.distance_m    = 0.0f;
            r.weak_signal   = false;
            return r;
        }
    }

    if (!for_calibrate && r.fault_found && !r.is_short) {
        int d = r.reflect_index - r.launch_index;
        if (d >= TDR_OPEN_DELTA_LO && d <= TDR_OPEN_DELTA_HI) {
            int q = tdr_reflection_quality(samples, n, launch_i, best_i, best_g);
            if (q < TDR_OPEN_MIN_QUALITY ||
                tdr_is_pcb_coupling(samples, n, launch_i, best_i, best_g)) {
                if (d <= TDR_SHORT_DELTA_MAX ||
                    tdr_is_pcb_coupling(samples, n, launch_i, best_i, best_g)) {
                    int open_g = 0;
                    int open_i = tdr_find_preferred_open_reflect(samples, n,
                                                                 launch_i, open_g);
                    if (open_i >= 0) {
                        best_i          = open_i;
                        best_g          = open_g;
                        r.reflect_index = open_i;
                        tdr_fill_distance(r, launch_i, open_i);
                        d = open_i - launch_i;
                        q = tdr_reflection_quality(samples, n, launch_i,
                                                   open_i, open_g);
                    }
                }
                if (q < TDR_OPEN_MIN_QUALITY ||
                    tdr_is_pcb_coupling(samples, n, launch_i, best_i, best_g)) {
                    r.fault_found   = false;
                    r.reflect_index = -1;
                    r.distance_m    = 0.0f;
                    if (!tdr_gp3_idle_stable())
                        r.unstable = true;
                    return r;
                }
            }
        }
    }

    // Stik 5-6 kort: kun hvis refleksion ligger ved launch (delta 0-1)
    if (!for_calibrate && r.fault_found && !r.is_short &&
        !tdr_open_reflection_blocks_short(samples, n, launch_i)) {
        int rd = r.reflect_index - launch_i;
        if (rd < 0)
            rd = 0;
        if (rd <= TDR_SHORT_DELTA_MAX) {
            int short_refl = launch_i;
            if (tdr_is_connector_short(samples, n, launch_i, &short_refl) &&
                !tdr_shot_forbids_connector_short(samples, n, launch_i, -1.0f)) {
                r.reflect_index = short_refl;
                r.launch_index  = launch_i;
                r.is_short      = true;
                r.distance_m    = 0.0f;
            }
        }
    }

    if (!for_calibrate && r.fault_found && !r.is_short) {
        int rd = r.reflect_index - launch_i;
        if (rd < 0)
            rd = 0;
        if (rd < TDR_DIST_MIN_DELTA) {
            int open_g = 0;
            int open_i = tdr_find_preferred_open_reflect(samples, n, launch_i,
                                                           open_g);
            if (open_i >= 0) {
                r.reflect_index = open_i;
                tdr_fill_distance(r, launch_i, open_i);
            } else {
                r.distance_m = 0.0f;
            }
        }
    }

    return r;
}

// ------------------------------------------------------------
// Raw TDR
// ------------------------------------------------------------
TdrResult tdr_measure() {
    tdr_capture();
    if (!tdr_samples_vary(g_samples, 128))
        return TdrResult{};
    return tdr_detect_reflection(g_samples, 128, false);
}

// ------------------------------------------------------------
// Filter
// ------------------------------------------------------------
static void tdr_filter_majority() {
    g_filtered[0]   = g_samples[0];
    g_filtered[127] = g_samples[127];

    for (int i = 1; i < 127; i++) {
        int sum = g_samples[i-1] + g_samples[i] + g_samples[i+1];
        g_filtered[i] = (sum >= 2) ? 1 : 0;
    }
}

// ------------------------------------------------------------
// Filtered TDR
// ------------------------------------------------------------
static TdrResult tdr_measure_filtered_ex(bool for_calibrate,
                                         TdrCalibType cal_type = TdrCalibType::Open) {
    if (for_calibrate)
        tdr_capture_for_calibrate();
    else
        tdr_capture();
    tdr_filter_majority();
    TdrResult r = tdr_detect_reflection(g_filtered, 128, for_calibrate, cal_type);
    if (!r.fault_found)
        r = tdr_detect_reflection(g_samples, 128, for_calibrate, cal_type);
    if (!r.fault_found) {
        const bool capture_ok = tdr_samples_vary(g_samples, 128);
        if (!for_calibrate || !capture_ok)
            tdr_run_hw_diag(r);
        else if (r.diag == TDR_DIAG_GP23_OK && !tdr_gp3_idle_stable())
            r.unstable = true;
    }
    if (for_calibrate)
        tdr_fill_calibrate_meta(r);
    return r;
}

TdrResult tdr_measure_filtered() {
    return tdr_measure_filtered_ex(false);
}

// ------------------------------------------------------------
// Stabil måling (ignorerer fingre / kapacitiv støj)
// ------------------------------------------------------------
static int tdr_shot_quality(const TdrResult &r, const uint8_t *samples, int n) {
    if (!r.fault_found || r.reflect_index < 1 || r.launch_index < 0)
        return 0;
    int g = (int)samples[r.reflect_index] -
            (int)samples[r.reflect_index - 1];
    return tdr_reflection_quality(samples, n, r.launch_index, r.reflect_index, g);
}

// Antal HI-samples i OPEN-vinduet — shot-to-shot spredning skelner kabel fra tom stik
static int tdr_open_window_popcount(const uint8_t *samples, int n, int launch_i) {
    if (launch_i < 0)
        return 0;
    int lo = launch_i + TDR_OPEN_DELTA_LO;
    int hi = launch_i + TDR_OPEN_DELTA_HI;
    if (hi >= n)
        hi = n - 1;
    int c = 0;
    for (int i = lo; i <= hi; i++) {
        if (samples[i])
            c++;
    }
    return c;
}

static int tdr_int_spread(const int *vals, int n) {
    if (n <= 0)
        return 0;
    int vmin = vals[0];
    int vmax = vals[0];
    for (int i = 1; i < n; i++) {
        if (vals[i] < vmin)
            vmin = vals[i];
        if (vals[i] > vmax)
            vmax = vals[i];
    }
    return vmax - vmin;
}

static int tdr_int_median(int *vals, int n) {
    if (n <= 0)
        return 0;
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (vals[j] < vals[i]) {
                int t = vals[i];
                vals[i] = vals[j];
                vals[j] = t;
            }
        }
    }
    return vals[n / 2];
}

// Længste sammenhængende HI på GP3 mens sendepuls (GP2) er aktiv
static int tdr_gp3_pulse_width(const uint8_t *samples, int n) {
    int lo = TDR_PULSE_ON;
    int hi = g_capture_pulse_off - 1;
    if (lo < 0)
        lo = 0;
    if (hi >= n)
        hi = n - 1;
    if (hi < lo)
        return 0;

    int best = 0;
    int run  = 0;
    for (int i = lo; i <= hi; i++) {
        if (samples[i]) {
            run++;
            if (run > best)
                best = run;
        } else {
            run = 0;
        }
    }
    return best;
}

static bool tdr_has_open_cable_reflection(const uint8_t *samples, int n,
                                          int launch_i) {
    if (launch_i < 0)
        return false;

    if (tdr_shot_gp3_immediate_follow(samples, n, launch_i) &&
        !tdr_shot_delayed_open_rise(samples, n, launch_i) &&
        !tdr_shot_reflection_after_quiet(samples, n, launch_i))
        return false;

    int zone_g = 0;
    int zone_i = tdr_find_preferred_open_reflect(samples, n, launch_i, zone_g);
    if (zone_i < 0) {
        if ((tdr_shot_reflection_after_quiet(samples, n, launch_i) ||
             tdr_shot_delayed_open_rise(samples, n, launch_i) ||
             tdr_shot_launch_refl_rise(samples, n, launch_i)) &&
            tdr_gp3_pulse_width(samples, n) >= TDR_CABLE_MIN_PULSE_WIDTH)
            return true;
        if (tdr_gp3_pulse_width(samples, n) >= TDR_WIDE_CABLE_PULSE_MIN) {
            int late_g = 0;
            int late_i = tdr_find_late_open_peak(samples, n, launch_i, late_g);
            if (late_i >= 0)
                return true;
        }
        return false;
    }

    int delta = zone_i - launch_i;
    if (delta < TDR_OPEN_DELTA_LO || delta > TDR_OPEN_DELTA_HI)
        return false;
    bool cable_pulse =
        tdr_gp3_pulse_width(samples, n) >= TDR_CABLE_MIN_PULSE_WIDTH;
    bool edge_ok = tdr_reflect_edge_ok(samples, n, zone_i, zone_g) ||
                   (cable_pulse && delta >= TDR_RELAXED_OPEN_DELTA_LO &&
                    delta <= TDR_RELAXED_OPEN_DELTA_MID &&
                    tdr_reflect_edge_relaxed(samples, n, zone_i, zone_g));
    if (!edge_ok)
        return false;
    if (tdr_is_pcb_coupling(samples, n, launch_i, zone_i, zone_g))
        return false;

    int q = tdr_reflection_quality(samples, n, launch_i, zone_i, zone_g);
    if (q < TDR_OPEN_MIN_QUALITY) {
        if (!(cable_pulse && delta >= TDR_RELAXED_OPEN_DELTA_LO &&
              delta <= TDR_RELAXED_OPEN_DELTA_MID &&
              tdr_reflect_edge_relaxed(samples, n, zone_i, zone_g)))
            return false;
    }
    if (!cable_pulse)
        return false;
    return true;
}

// Stærk OPEN-signatur (typisk ~3 m): delta 3-8, inkl. forsinket stigning trods bred GP3-puls
static bool tdr_strong_open_cable_sig(const uint8_t *samples, int n, int launch_i) {
    if (launch_i < 0)
        return false;

    if (tdr_shot_reflection_after_quiet(samples, n, launch_i))
        return true;

    if (tdr_shot_delayed_open_rise(samples, n, launch_i) ||
        tdr_shot_launch_refl_rise(samples, n, launch_i)) {
        if (tdr_gp3_pulse_width(samples, n) < TDR_CABLE_MIN_PULSE_WIDTH)
            return false;
        int zone_g = 0;
        int zone_i = tdr_find_open_zone_edge(samples, n, launch_i, zone_g);
        if (zone_i < 0)
            return true;
        int delta = zone_i - launch_i;
        return delta >= TDR_STRONG_OPEN_DELTA_LO && delta <= TDR_OPEN_DELTA_HI;
    }

    if (!tdr_has_open_cable_reflection(samples, n, launch_i))
        return false;

    int zone_g = 0;
    int zone_i = tdr_find_open_zone_edge(samples, n, launch_i, zone_g);
    if (zone_i < 0)
        return false;
    int delta = zone_i - launch_i;
    return delta >= TDR_STRONG_OPEN_DELTA_LO && delta <= TDR_OPEN_DELTA_HI;
}

// Stærkeste HI-hold i OPEN-refleksionsvinduet (amplitude-proxy)
static int tdr_open_window_max_hi_run(const uint8_t *samples, int n, int launch_i) {
    if (launch_i < 0)
        return 0;
    int lo = launch_i + TDR_OPEN_DELTA_LO;
    int hi = launch_i + TDR_OPEN_DELTA_HI;
    if (lo < 0)
        lo = 0;
    if (hi >= n)
        hi = n - 1;
    if (hi < lo)
        return 0;

    int best = 0;
    int run  = 0;
    for (int i = lo; i <= hi; i++) {
        if (samples[i]) {
            run++;
            if (run > best)
                best = run;
        } else {
            run = 0;
        }
    }
    return best;
}

static void tdr_clear_fault(TdrResult &r) {
    r.fault_found   = false;
    r.reflect_index = -1;
    r.distance_m    = 0.0f;
    r.is_short      = false;
    r.weak_signal   = false;
    r.no_cable      = false;
    r.consensus_strong = false;
}

static int tdr_shot_reflection_delta(const TdrResult &r, const uint8_t *samples,
                                   int n, int launch_i) {
    if (launch_i < 0)
        launch_i = TDR_PULSE_ON;

    if (r.fault_found && r.launch_index >= 0) {
        int d = r.reflect_index - r.launch_index;
        if (d < 0)
            d = 0;
        if (d > TDR_SHORT_DELTA_MAX)
            return d;
    }

    int zone_g = 0;
    int zone_i = tdr_find_preferred_open_reflect(samples, n, launch_i, zone_g);
    if (zone_i >= 0) {
        int zd = zone_i - launch_i;
        if (zd >= TDR_DIST_MIN_DELTA)
            return zd;
    }

    if (tdr_gp3_pulse_width(samples, n) >= TDR_WIDE_CABLE_PULSE_MIN) {
        int late_g = 0;
        int late_i = tdr_find_late_open_peak(samples, n, launch_i, late_g);
        if (late_i >= 0) {
            int ld = late_i - launch_i;
            if (ld >= TDR_DIST_MIN_DELTA)
                return ld;
        }
    }

    if (tdr_shot_reflection_after_quiet(samples, n, launch_i))
        return TDR_TYPICAL_3M_MEDIAN_LO + 1;
    if (tdr_shot_delayed_open_rise(samples, n, launch_i) ||
        tdr_shot_launch_refl_rise(samples, n, launch_i))
        return TDR_STRONG_OPEN_DELTA_LO;

    if (r.fault_found && r.launch_index >= 0) {
        int d = r.reflect_index - r.launch_index;
        if (d < 0)
            d = 0;
        return d;
    }

    int rise_d = tdr_gp3_rise_delta_after_launch(samples, n, launch_i);
    return (rise_d > TDR_SHORT_DELTA_MAX) ? 99 : rise_d;
}

static int tdr_shot_delta_max(const int *shot_delta, int tries) {
    int vmax = 0;
    for (int i = 0; i < tries; i++) {
        if (shot_delta[i] >= 99)
            continue;
        if (shot_delta[i] > vmax)
            vmax = shot_delta[i];
    }
    return vmax;
}

static bool tdr_any_shot_delta_ge3(const int *shot_delta, int tries) {
    for (int i = 0; i < tries; i++) {
        if (shot_delta[i] >= 3 && shot_delta[i] < 99)
            return true;
    }
    return false;
}

static bool tdr_median_typical_3m_open(int median_delta) {
    return median_delta >= TDR_TYPICAL_3M_MEDIAN_LO &&
           median_delta <= TDR_TYPICAL_3M_MEDIAN_HI;
}

static int tdr_count_shots_delta_at_least(const int *shot_delta, int tries,
                                           int min_d) {
    int c = 0;
    for (int i = 0; i < tries; i++) {
        if (shot_delta[i] >= min_d && shot_delta[i] < 99)
            c++;
    }
    return c;
}

static int tdr_count_shots_delta_in_range(const int *shot_delta, int tries,
                                           int d_lo, int d_hi) {
    int c = 0;
    for (int i = 0; i < tries; i++) {
        if (shot_delta[i] >= d_lo && shot_delta[i] <= d_hi && shot_delta[i] < 99)
            c++;
    }
    return c;
}

static int tdr_count_shots_delta24_open_band(const int *shot_delta, int tries) {
    int c = 0;
    for (int i = 0; i < tries; i++) {
        int d = shot_delta[i];
        if (d < TDR_OPEN_DELTA24_LO || d > TDR_OPEN_DELTA24_HI || d >= 99)
            continue;
        float dm = tdr_distance_m_from_indices(TDR_PULSE_ON, TDR_PULSE_ON + d);
        if (dm >= TDR_OPEN_DISPLAY_DIST_LO && dm <= TDR_OPEN_DISPLAY_DIST_HI)
            c++;
    }
    return c;
}

// Fallback OPEN: >=3/6 shots med kant (ri>=launch+1), pw>=4, bedste delta 2-6
static bool tdr_open_fallback_consensus_ok(const int *shot_raw_i,
                                           const int *shot_pref_delta,
                                           const int *pulse_w, int tries,
                                           int launch_i, bool hw_short) {
    if (hw_short)
        return false;
    int edge_shots = 0;
    int best_delta = -1;
    for (int i = 0; i < tries; i++) {
        if (pulse_w[i] < TDR_CABLE_MIN_PULSE_WIDTH)
            continue;
        if (shot_raw_i[i] < launch_i + 1)
            continue;
        edge_shots++;
        int rd = shot_pref_delta[i];
        if (rd >= TDR_RELAXED_OPEN_DELTA_LO && rd <= TDR_RELAXED_OPEN_DELTA_MID &&
            (best_delta < 0 || rd > best_delta))
            best_delta = rd;
    }
    return edge_shots >= TDR_OPEN_FALLBACK_MIN_SHOTS &&
           best_delta >= TDR_RELAXED_OPEN_DELTA_LO;
}

static bool tdr_any_shot_has_post_launch_edge(const int *shot_raw_i,
                                              const int *shot_pref_delta,
                                              int tries, int launch_i) {
    for (int i = 0; i < tries; i++) {
        if (shot_raw_i[i] >= launch_i + 1)
            return true;
        if (shot_pref_delta[i] >= TDR_RELAXED_OPEN_DELTA_LO)
            return true;
    }
    return false;
}

// OPEN-konsensus: median 3-6, >=3× d>=3, >=3× delta 2-4 m. afstand 1.5-4.5 m
static bool tdr_open_cable_consensus_ok(int median_delta, const int *shot_delta,
                                       int tries) {
    if (tdr_median_typical_3m_open(median_delta))
        return true;
    if (tdr_count_shots_delta_at_least(shot_delta, tries,
                                       TDR_TYPICAL_3M_DELTA_LO) >=
        TDR_OPEN_CONSENSUS_MIN_D3)
        return true;
    return tdr_count_shots_delta24_open_band(shot_delta, tries) >=
           TDR_OPEN_DELTA24_MIN_AGREE;
}

static void tdr_normalize_open_distance(TdrResult &r) {
    if (!r.fault_found || r.is_short || r.launch_index < 0)
        return;

    if (r.distance_m > TDR_MAX_DISTANCE_M)
        r.distance_m = TDR_MAX_DISTANCE_M;

    int li = r.launch_index;
    int d  = r.reflect_index - li;
    if (d < 0)
        d = 0;

    bool dist_ok = r.distance_m >= TDR_OPEN_DISPLAY_DIST_LO &&
                   r.distance_m <= TDR_OPEN_DISPLAY_DIST_HI;
    bool delta_ok = d >= TDR_TYPICAL_3M_DELTA_LO && d <= TDR_TYPICAL_3M_DELTA_HI;

    if (delta_ok && dist_ok)
        return;

    int fn = 128;
    const uint8_t *f = tdr_get_filtered(fn);
    int zone_g = 0;
    int zone_i = tdr_find_preferred_open_reflect(f, fn, li, zone_g);
    if (zone_i < 0)
        return;

    int zd = zone_i - li;
    if (zd < TDR_TYPICAL_3M_DELTA_LO || zd > TDR_TYPICAL_3M_DELTA_HI)
        return;

    r.reflect_index = zone_i;
    tdr_fill_distance(r, li, zone_i);
    if (r.distance_m > TDR_MAX_DISTANCE_M)
        r.distance_m = TDR_MAX_DISTANCE_M;
}

static bool tdr_short_consensus_allowed(int median_delta, int max_shot_delta,
                                        int strong_open_votes,
                                        float max_zone_dist_m) {
    if (max_zone_dist_m >= TDR_SHORT_BLOCK_ZONE_DIST_M)
        return false;
    return median_delta <= tdr_get_short_delta_max() &&
           max_shot_delta <= TDR_SHORT_MAX_SHOT_DELTA &&
           strong_open_votes == 0;
}

static void tdr_set_vote_debug(TdrResult &r, uint8_t md, uint8_t so, uint8_t sh) {
    r.vote_median_delta = md;
    r.vote_strong_open  = so;
    r.vote_short        = sh;
}

// Median af reflect_index - launch_index (kun tydelig refleksion efter launch)
static int tdr_median_reflect_delta(const TdrResult *results, int tries) {
    int vals[6];
    int n = 0;
    for (int i = 0; i < tries; i++) {
        if (!results[i].fault_found || results[i].launch_index < 0)
            continue;
        if (results[i].reflect_index <= results[i].launch_index)
            continue;
        vals[n++] = results[i].reflect_index - results[i].launch_index;
    }
    if (n <= 0)
        return -1;
    return tdr_int_median(vals, n);
}

static int tdr_count_short_shots(const TdrResult *results, const int *shot_delta,
                                 const bool *strong_open_sig, int tries) {
    int v = 0;
    for (int i = 0; i < tries; i++) {
        if (shot_delta[i] >= 99 || shot_delta[i] > TDR_SHORT_DELTA_MAX)
            continue;
        if (shot_delta[i] >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO)
            continue;
        if (strong_open_sig[i])
            continue;
        if (!results[i].fault_found || !results[i].is_short)
            continue;
        v++;
    }
    return v;
}

static int tdr_count_short_gp3_shots(const int *shot_delta, const bool *gp3_follow,
                                     int tries) {
    int v = 0;
    for (int i = 0; i < tries; i++) {
        if (shot_delta[i] >= 99 || shot_delta[i] > TDR_SHORT_DELTA_MAX)
            continue;
        if (shot_delta[i] >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO)
            continue;
        if (gp3_follow[i])
            v++;
    }
    return v;
}

static bool tdr_shot_pulse_short_sig(const uint8_t *samples, int n, int launch_i,
                                   int pulse_w, int shot_delta) {
    if (shot_delta > TDR_SHORT_DELTA_MAX || shot_delta >= 99)
        return false;
    if (pulse_w < TDR_CABLE_MIN_PULSE_WIDTH)
        return false;
    if (tdr_has_later_reflection_peak(samples, n, launch_i))
        return false;
    if (tdr_shot_delayed_open_rise(samples, n, launch_i) ||
        tdr_shot_reflection_after_quiet(samples, n, launch_i))
        return false;
    return tdr_shot_gp3_immediate_follow(samples, n, launch_i) ||
           tdr_is_connector_short(samples, n, launch_i, nullptr);
}

static void tdr_force_shot_connector_short(TdrResult &r, const uint8_t *samples,
                                          int n, int launch_i) {
    int fn = 128;
    const uint8_t *f = tdr_get_filtered(fn);
    if (tdr_shot_forbids_connector_short(f, fn, launch_i, -1.0f))
        return;
    int short_refl = launch_i;
    if (!tdr_is_connector_short(samples, n, launch_i, &short_refl))
        return;
    r.fault_found   = true;
    r.is_short      = true;
    r.launch_index  = launch_i;
    r.reflect_index = short_refl;
    r.distance_m    = 0.0f;
}

// Per-shot stemmer til majoritetsbeslutning i tdr_measure_stable_impl
static void tdr_count_shot_votes(const TdrResult *results, const bool *open_cable_sig,
                                 const bool *strong_open_sig, const bool *gp3_follow,
                                 const int *shot_q, const int *pulse_w, int tries,
                                 int open_delta_hi, int &open_stable_votes,
                                 int &no_cable_votes, int &short_strict_votes,
                                 int &gp3_follow_votes) {
    open_stable_votes  = 0;
    no_cable_votes     = 0;
    short_strict_votes = 0;
    gp3_follow_votes   = 0;

    for (int i = 0; i < tries; i++) {
        const TdrResult &ri = results[i];
        int d = 0;
        if (ri.fault_found && ri.launch_index >= 0) {
            d = ri.reflect_index - ri.launch_index;
            if (d < 0)
                d = 0;
        }

        if (gp3_follow[i])
            gp3_follow_votes++;

        bool open_stable = open_cable_sig[i];
        if (!open_stable && ri.fault_found && !ri.is_short &&
            d >= TDR_OPEN_DELTA_LO && d <= open_delta_hi &&
            shot_q[i] >= TDR_OPEN_MIN_QUALITY &&
            pulse_w[i] >= TDR_CABLE_MIN_PULSE_WIDTH)
            open_stable = true;
        if (open_stable)
            open_stable_votes++;

        if (!ri.fault_found) {
            if (!open_cable_sig[i] && !strong_open_sig[i])
                no_cable_votes++;
        } else if (!ri.is_short && !open_cable_sig[i] &&
                 shot_q[i] < TDR_COUPLING_MAX_QUALITY)
            no_cable_votes++;

        bool shot_open_band = d >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO &&
                              d <= open_delta_hi;
        (void)gp3_follow;
        bool strict_short = ri.is_short && ri.fault_found &&
                            d <= TDR_SHORT_DELTA_MAX && !shot_open_band;
        if (strict_short && (strong_open_sig[i] || open_cable_sig[i]))
            strict_short = false;
        if (strict_short && tdr_median_typical_3m_open(d))
            strict_short = false;
        if (strict_short)
            short_strict_votes++;
    }
}

static TdrResult tdr_return_short_consensus(const TdrResult *results,
                                            const bool *strong_open_sig, int tries) {
    for (int i = 0; i < tries; i++) {
        if (strong_open_sig[i])
            continue;
        if (results[i].is_short && results[i].fault_found) {
            TdrResult out = results[i];
            out.fault_found      = true;
            out.unstable         = false;
            out.distance_m       = 0.0f;
            out.is_short         = true;
            out.consensus_strong = true;
            if (out.launch_index < 0)
                out.launch_index = TDR_PULSE_ON;
            if (out.reflect_index < 0)
                out.reflect_index = out.launch_index;
            return out;
        }
    }
    TdrResult out{};
    out.fault_found      = true;
    out.launch_index     = TDR_PULSE_ON;
    out.reflect_index    = TDR_PULSE_ON;
    out.is_short         = true;
    out.distance_m       = 0.0f;
    out.consensus_strong = true;
    return out;
}

static TdrResult tdr_pick_open_stable_shot(const TdrResult *results,
                                           const bool *open_cable_sig,
                                           const int *shot_q, int tries,
                                           int open_delta_hi,
                                           float median_open_dist_m = -1.0f) {
    int best_q     = -1;
    int best_delta = -1;
    int best_i     = tries - 1;
    for (int i = 0; i < tries; i++) {
        const TdrResult &c = results[i];
        int launch_i = c.launch_index >= 0 ? c.launch_index : TDR_PULSE_ON;
        int fn = 128;
        const uint8_t *f = tdr_get_filtered(fn);
        int pref_g = 0;
        int pref_i = tdr_find_preferred_open_reflect(f, fn, launch_i, pref_g);
        int cdelta = 0;
        if (pref_i >= 0) {
            cdelta = pref_i - launch_i;
        } else if (c.fault_found && c.launch_index >= 0) {
            cdelta = c.reflect_index - c.launch_index;
            if (cdelta < 0)
                cdelta = 0;
        }

        bool ok = open_cable_sig[i] ||
                  (c.fault_found && !c.is_short &&
                   cdelta >= TDR_OPEN_DELTA_LO && cdelta <= open_delta_hi &&
                   shot_q[i] >= TDR_OPEN_MIN_QUALITY);
        if (!ok &&
            cdelta >= TDR_OPEN_DELTA24_LO && cdelta <= TDR_OPEN_DELTA24_HI &&
            shot_q[i] >= TDR_OPEN_MIN_QUALITY - 2)
            ok = true;
        if (!ok)
            continue;

        bool band_25 = cdelta >= TDR_OPEN_DELTA24_LO && cdelta <= 5;
        if (band_25) {
            if (cdelta > best_delta ||
                (cdelta == best_delta && shot_q[i] > best_q)) {
                best_delta = cdelta;
                best_q     = shot_q[i];
                best_i     = i;
            }
        } else if (best_delta < TDR_OPEN_DELTA24_LO) {
            if (shot_q[i] > best_q) {
                best_q = shot_q[i];
                best_i = i;
            }
        }
    }
    TdrResult out = results[best_i];
    out.is_short         = false;
    out.unstable         = false;
    out.no_cable         = false;
    out.weak_signal      = false;
    out.consensus_strong = true;
    int n = 128;
    const uint8_t *f = tdr_get_filtered(n);
    int launch_i = out.launch_index >= 0 ? out.launch_index : TDR_PULSE_ON;
    int zone_g = 0;
    int zone_i = -1;
    if (!tdr_shot_blocks_late_open(f, n, launch_i) ||
        tdr_shot_cable_end_reflection(f, n, launch_i)) {
        zone_i = tdr_find_preferred_open_reflect(f, n, launch_i, zone_g);
        if (zone_i < 0)
            zone_i = tdr_find_late_open_peak(f, n, launch_i, zone_g);
    }
    if (zone_i >= 0) {
        out.fault_found   = true;
        out.reflect_index = zone_i;
        out.launch_index  = launch_i;
        tdr_fill_distance(out, launch_i, zone_i);
    } else if (!out.fault_found && open_cable_sig[best_i]) {
        tdr_apply_late_open_reflect(out, f, n);
    } else if (out.fault_found && !out.is_short && out.launch_index >= 0) {
        int rd = out.reflect_index - out.launch_index;
        if (rd <= TDR_SHORT_DELTA_MAX)
            tdr_apply_late_open_reflect(out, f, n);
    }
    tdr_normalize_open_distance(out);
    tdr_finalize_stable_open(out, results, tries, median_open_dist_m);
    if (tdr_result_bogus_open_cycle(out) && g_stable_cycle.valid &&
        tdr_count_shots_delta_in_range(g_stable_cycle.shot_pref_delta, tries,
                                       TDR_OPEN_DELTA24_LO,
                                       TDR_OPEN_DELTA24_HI) >= 1) {
        for (int i = 0; i < tries; i++) {
            int pd = g_stable_cycle.shot_pref_delta[i];
            if (pd < TDR_OPEN_DELTA24_LO || pd > TDR_OPEN_DELTA24_HI ||
                results[i].is_short)
                continue;
            out = results[i];
            out.is_short         = false;
            out.no_cable         = false;
            out.weak_signal      = false;
            out.unstable         = false;
            out.consensus_strong = true;
            tdr_apply_open_shot_indices(out);
            tdr_finalize_stable_open(out, results, tries, median_open_dist_m);
            if (!tdr_result_bogus_open_cycle(out))
                break;
        }
    }
    return out;
}

static TdrResult tdr_return_cable_end_short(const TdrResult *results,
                                            const bool *open_cable_sig,
                                            const int *shot_q, int tries,
                                            int open_delta_hi,
                                            float median_open_dist_m,
                                            float max_open_zone_dist) {
    TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                              tries, open_delta_hi,
                                              median_open_dist_m);
    out.fault_found      = true;
    out.is_short         = true;
    out.no_cable         = false;
    out.weak_signal      = false;
    out.unstable         = false;
    out.consensus_strong = true;

    int li = out.launch_index >= 0 ? out.launch_index : TDR_PULSE_ON;
    int fn = 128;
    const uint8_t *f = tdr_get_filtered(fn);
    int zone_g = 0;
    int zone_i = tdr_find_preferred_open_reflect(f, fn, li, zone_g);
    if (zone_i < 0)
        zone_i = tdr_find_late_open_peak(f, fn, li, zone_g);
    if (zone_i < 0) {
        int strict_g = 0;
        zone_i = tdr_find_late_open_peak_strict(f, fn, li, strict_g);
    }
    if (zone_i >= 0) {
        out.launch_index  = li;
        out.reflect_index = zone_i;
        tdr_fill_distance(out, li, zone_i);
    }
    if (out.distance_m < TDR_OPEN_DISPLAY_DIST_LO) {
        if (median_open_dist_m >= TDR_OPEN_DISPLAY_DIST_LO)
            out.distance_m = median_open_dist_m;
        else if (max_open_zone_dist >= TDR_OPEN_DISPLAY_DIST_LO)
            out.distance_m = max_open_zone_dist;
    }
    if (out.distance_m > TDR_MAX_DISTANCE_M)
        out.distance_m = TDR_MAX_DISTANCE_M;
    return out;
}

static void tdr_set_no_signal(TdrResult &r, bool weak) {
    if (g_stable_cycle.valid && tdr_cycle_has_open_evidence(g_stable_cycle)) {
        if (tdr_force_open_from_cycle_evidence(r))
            return;
    }
    tdr_clear_fault(r);
    r.unstable = false;
    if (weak) {
        r.weak_signal = true;
    } else {
        r.no_cable = true;
        if (tdr_hw_ground_fault())
            r.diag = TDR_DIAG_GROUND_FAULT;
    }
}

static TdrResult tdr_measure_stable_impl(int open_delta_hi, bool for_calibrate,
                                         TdrCalibType cal_type = TdrCalibType::Open) {
    const int tries = 6;
    const bool allow_short =
        !for_calibrate || tdr_short_cal_active(for_calibrate, cal_type);
    // hw_continuity: GP3 følger GP2 (stik 5-6 kort eller forbundet kabel)
    // hw_connector_short (bare): kun tom stik-SHORT — blokerer ikke OPEN ved kabelende
    const bool hw_continuity =
        allow_short && tdr_hw_connector_shorted();
    bool hw_connector_short = hw_continuity;

    // HW stik 5-6 kort: kun 0.0 m når ingen refleksion ved kabelende
    if (hw_continuity) {
        TdrResult snap = for_calibrate ? tdr_measure_filtered_ex(true, cal_type)
                                       : tdr_measure_filtered();
        int n = 128;
        const uint8_t *s = tdr_get_samples(n);
        int fn = 128;
        const uint8_t *f = tdr_get_filtered(fn);
        int launch_i = snap.launch_index >= 0 ? snap.launch_index : TDR_PULSE_ON;
        float probe_zone = tdr_shot_open_zone_distance_m(f, fn, launch_i);
        if (probe_zone < 0.0f) {
            int sd = tdr_shot_reflection_delta(snap, f, fn, launch_i);
            if (sd >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO)
                probe_zone = tdr_distance_m_from_indices(launch_i, launch_i + sd);
        }
        const float probe_late =
            tdr_shot_late_refl_distance_m(f, fn, launch_i);
        const bool cable_end_probe =
            tdr_shot_cable_end_reflection(f, fn, launch_i) ||
            probe_zone >= TDR_SHORT_BLOCK_ZONE_DIST_M ||
            probe_late >= TDR_SHORT_BLOCK_ZONE_DIST_M;

        if (!cable_end_probe) {
            TdrResult out = snap;
            out.fault_found      = true;
            out.is_short         = true;
            out.no_cable         = false;
            out.weak_signal      = false;
            out.unstable         = false;
            out.consensus_strong = true;
            out.distance_m       = 0.0f;
            if (out.launch_index < 0)
                out.launch_index = TDR_PULSE_ON;
            if (out.reflect_index < 0)
                out.reflect_index = out.launch_index;
            int pw = tdr_gp3_pulse_width(g_samples, 128);
            tdr_save_stable_cycle(&snap, 1, -1.0f, -1.0f, pw, 0, nullptr,
                                  for_calibrate, true, true);
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
            g_stable_dbg.hw_short             = true;
            g_stable_dbg.max_open_zone_dist_m = -1.0f;
            g_stable_dbg.pulse_width_median   = pw;
            g_stable_dbg.rule                 = "HW";
            g_stable_dbg.fix_applied          = false;
#endif
#ifdef TDR_DEBUG
            TDR_DBG("TDR path=0.0m SHORT rule=HW probe_zone=%.2f probe_late=%.2f\n",
                    probe_zone, probe_late);
#endif
            return out;
        }
        hw_connector_short = false;
#ifdef TDR_DEBUG
        TDR_DBG("TDR path=cable-end probe (6-shot) probe_zone=%.2f probe_late=%.2f\n",
                probe_zone, probe_late);
#endif
    }

    TdrResult results[tries];
    int       shot_q[tries];
    int       open_pop[tries];
    int       pulse_w[tries];
    int       open_amp[tries];
    int       shot_delta[tries];
    int       shot_raw_i[tries];
    int       shot_pref_delta[tries];
    bool      open_cable_sig[tries];
    bool      strong_open_sig[tries];
    bool      gp3_follow[tries];
    float     max_open_zone_dist = -1.0f;
    float     max_strict_late_dist = -1.0f;
    int       cable_end_refl_votes = 0;

    for (int i = 0; i < tries; i++) {
        results[i] = for_calibrate ? tdr_measure_filtered_ex(true, cal_type)
                                   : tdr_measure_filtered();
        int n = 128;
        const uint8_t *s = tdr_get_samples(n);
        int fn = 128;
        const uint8_t *f = tdr_get_filtered(fn);
        shot_q[i] = tdr_shot_quality(results[i], s, n);
        pulse_w[i] = tdr_gp3_pulse_width(s, n);
        int launch_i = results[i].launch_index;
        if (launch_i < 0)
            launch_i = TDR_PULSE_ON;
        gp3_follow[i] = tdr_shot_gp3_immediate_follow(s, n, launch_i);
        {
            int raw_g = 0;
            shot_raw_i[i] = tdr_find_open_zone_edge(f, fn, launch_i, raw_g);
        }
        if (!tdr_short_cal_active(for_calibrate, cal_type) &&
            pulse_w[i] >= TDR_WIDE_CABLE_PULSE_MIN)
            tdr_apply_shot_late_open_upgrade(results[i], f, fn, pulse_w[i]);
        shot_delta[i] = tdr_shot_reflection_delta(results[i], f, fn, launch_i);
        {
            int pref_g = 0;
            int pref_i = tdr_find_preferred_open_reflect(f, fn, launch_i, pref_g);
            shot_pref_delta[i] =
                (pref_i >= 0) ? (pref_i - launch_i) : -1;
            if (shot_pref_delta[i] < TDR_DIST_MIN_DELTA &&
                pulse_w[i] >= TDR_WIDE_CABLE_PULSE_MIN) {
                int late_g = 0;
                int late_i = tdr_find_late_open_peak(f, fn, launch_i, late_g);
                if (late_i >= 0)
                    shot_pref_delta[i] = late_i - launch_i;
            }
            if (shot_pref_delta[i] < TDR_DIST_MIN_DELTA &&
                results[i].fault_found && results[i].launch_index >= 0) {
                int ud = results[i].reflect_index - results[i].launch_index;
                if (ud >= TDR_DIST_MIN_DELTA)
                    shot_pref_delta[i] = ud;
            }
        }
        open_amp[i] = tdr_open_window_max_hi_run(s, n, launch_i);
        open_cable_sig[i] = tdr_has_open_cable_reflection(f, fn, launch_i);
        strong_open_sig[i] = tdr_strong_open_cable_sig(f, fn, launch_i);
        float zone_dist = tdr_shot_open_zone_distance_m(f, fn, launch_i);
        if (zone_dist < 0.0f && shot_delta[i] >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO)
            zone_dist = tdr_distance_m_from_indices(launch_i, launch_i + shot_delta[i]);
        if (zone_dist < 0.0f &&
            shot_pref_delta[i] >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO)
            zone_dist = tdr_distance_m_from_indices(launch_i,
                                                    launch_i + shot_pref_delta[i]);
        const float strict_late =
            tdr_shot_late_refl_distance_m(f, fn, launch_i);
        if (strict_late > max_strict_late_dist)
            max_strict_late_dist = strict_late;
        if (zone_dist < 0.0f && strict_late >= TDR_SHORT_BLOCK_ZONE_DIST_M)
            zone_dist = strict_late;
        if (hw_continuity &&
            tdr_shot_cable_end_reflection(f, fn, launch_i))
            cable_end_refl_votes++;
        if (zone_dist > max_open_zone_dist)
            max_open_zone_dist = zone_dist;
        if (results[i].fault_found && results[i].launch_index >= 0)
            open_pop[i] = tdr_open_window_popcount(s, n, results[i].launch_index);
        else
            open_pop[i] = -1;

        if (zone_dist > TDR_MAX_DISTANCE_M)
            zone_dist = TDR_MAX_DISTANCE_M;

        // Per-shot SHORT: kun stik 5-6 (0 m), ikke kortslutning ved kabelende
        if (allow_short && shot_delta[i] < TDR_MEDIAN_OPEN_BLOCK_SHORT_LO &&
            !strong_open_sig[i] &&
            !tdr_shot_forbids_connector_short(f, fn, launch_i, zone_dist)) {
            float shot_dist = zone_dist;
            if (shot_dist < 0.0f && results[i].fault_found &&
                results[i].launch_index >= 0 && results[i].reflect_index >= 0) {
                int rd = results[i].reflect_index - results[i].launch_index;
                if (rd >= TDR_DIST_MIN_DELTA)
                    shot_dist = results[i].distance_m;
                else
                    shot_dist = -1.0f;
            }
            if (tdr_shot_pulse_short_sig(f, fn, launch_i, pulse_w[i],
                                         shot_delta[i]) ||
                (!tdr_distance_forces_open(shot_dist) &&
                 tdr_is_connector_short(s, n, launch_i, nullptr)))
                tdr_force_shot_connector_short(results[i], s, n, launch_i);
        }

        if (shot_delta[i] >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO)
            results[i].is_short = false;

        if (allow_short && gp3_follow[i] &&
            shot_delta[i] <= TDR_SHORT_DELTA_MAX && !strong_open_sig[i] &&
            !tdr_shot_forbids_connector_short(f, fn, launch_i, zone_dist)) {
            results[i].fault_found   = true;
            results[i].is_short      = true;
            results[i].no_cable      = false;
            results[i].weak_signal   = false;
            results[i].distance_m    = 0.0f;
            if (results[i].launch_index < 0)
                results[i].launch_index = launch_i;
            if (results[i].reflect_index < 0)
                results[i].reflect_index = results[i].launch_index;
        }

        sleep_ms(8);
    }

    int open_stable_votes  = 0;
    int no_cable_votes     = 0;
    int short_strict_votes = 0;
    int gp3_follow_votes   = 0;
    tdr_count_shot_votes(results, open_cable_sig, strong_open_sig, gp3_follow,
                         shot_q, pulse_w, tries, open_delta_hi, open_stable_votes,
                         no_cable_votes, short_strict_votes, gp3_follow_votes);

    int open_sig_votes = 0;
    int strong_open_votes = 0;
    for (int i = 0; i < tries; i++) {
        if (open_cable_sig[i])
            open_sig_votes++;
        if (strong_open_sig[i])
            strong_open_votes++;
    }

    int all_delta_sorted[tries];
    for (int i = 0; i < tries; i++) {
        all_delta_sorted[i] = shot_delta[i];
        if (shot_pref_delta[i] >= TDR_RELAXED_OPEN_DELTA_LO &&
            shot_delta[i] <= TDR_SHORT_DELTA_MAX &&
            (!gp3_follow[i] ||
             (hw_continuity &&
              shot_pref_delta[i] >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO)))
            all_delta_sorted[i] = shot_pref_delta[i];
    }
    int shot_median_delta = tdr_int_median(all_delta_sorted, tries);
    int reflect_median    = tdr_median_reflect_delta(results, tries);
    int median_delta      = shot_median_delta;
    if (reflect_median >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO &&
        reflect_median > median_delta)
        median_delta = reflect_median;
    int max_shot_delta    = tdr_shot_delta_max(shot_delta, tries);
    int short_shot_votes  = tdr_count_short_shots(results, shot_delta,
                                                  strong_open_sig, tries);
    int short_gp3_votes   = tdr_count_short_gp3_shots(shot_delta, gp3_follow, tries);

    const uint8_t dbg_md = (uint8_t)(median_delta < 0 ? 0 : median_delta);
    const uint8_t dbg_so = (uint8_t)strong_open_votes;
    const uint8_t dbg_sh = (uint8_t)short_shot_votes;

    const float median_open_dist_m = tdr_median_open_distance_shots(results, tries);

    int width_sorted_early[tries];
    for (int i = 0; i < tries; i++)
        width_sorted_early[i] = pulse_w[i];
    int width_median_early = tdr_int_median(width_sorted_early, tries);
    bool pulse_cable_shape = width_median_early >= TDR_CABLE_MIN_PULSE_WIDTH;

    if (for_calibrate)
        g_calib_zmax_m = max_open_zone_dist;

    const bool all_gp3_follow = (gp3_follow_votes == tries);
    const bool consensus_median_short_early =
        median_delta <= TDR_SHORT_DELTA_MAX;
    const bool gp3_median_short =
        consensus_median_short_early && all_gp3_follow;
    const bool gp3_short_cal_consensus =
        tdr_short_cal_active(for_calibrate, cal_type) &&
        gp3_follow_votes >= TDR_STABLE_SHORT_MIN_AGREE;

    tdr_save_stable_cycle(results, tries, median_open_dist_m, max_open_zone_dist,
                          width_median_early, median_delta, shot_pref_delta,
                          for_calibrate, hw_connector_short, gp3_median_short);

    const int stable_launch_i = TDR_PULSE_ON;
    const bool fallback_open =
        !for_calibrate && !gp3_median_short &&
        tdr_open_fallback_consensus_ok(shot_raw_i, shot_pref_delta, pulse_w,
                                       tries, stable_launch_i,
                                       hw_connector_short);
    const bool any_post_edges = tdr_any_shot_has_post_launch_edge(
        shot_raw_i, shot_pref_delta, tries, stable_launch_i);

#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
    g_stable_dbg.hw_short             = hw_continuity;
    g_stable_dbg.max_open_zone_dist_m = max_open_zone_dist;
    g_stable_dbg.pulse_width_median   = width_median_early;
    g_stable_dbg.rule                 = for_calibrate ? "CAL" : "-";
    g_stable_dbg.fix_applied          = false;
    g_stable_dbg.vf_calc              = 0.0f;
    g_stable_dbg.vf_new               = 0.0f;
#endif

    float cable_end_zone_dist = max_open_zone_dist;
    if (max_strict_late_dist > cable_end_zone_dist)
        cable_end_zone_dist = max_strict_late_dist;

    const bool cable_end_short =
        !for_calibrate && allow_short && hw_continuity &&
        (tdr_cable_end_short_evidence(cable_end_zone_dist, median_delta,
                                      hw_continuity) ||
         cable_end_refl_votes >= TDR_STABLE_MIN_AGREE ||
         tdr_cycle_forbids_connector_short(cable_end_zone_dist,
                                           max_strict_late_dist, median_delta));

    // Kortslutning ved kabelende: HW kontinuitet + refleksion ved ~3 m
    if (cable_end_short) {
        TdrResult out = tdr_return_cable_end_short(results, open_cable_sig, shot_q,
                                                   tries, open_delta_hi,
                                                   median_open_dist_m,
                                                   cable_end_zone_dist);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
        g_stable_dbg.rule = "SH3";
        TDR_DBG("TDR path=cable-end SHORT dist=%.2f zmax=%.2f late=%.2f md=%d\n",
                out.distance_m, cable_end_zone_dist, max_strict_late_dist,
                median_delta);
#endif
        return out;
    }

    // GP3 følger alle shots + median delta<=1: stik-SHORT før zmax/fallback OPEN
    if (allow_short && (gp3_median_short || gp3_short_cal_consensus) &&
        !tdr_cycle_forbids_connector_short(cable_end_zone_dist,
                                           max_strict_late_dist, median_delta)) {
        TdrResult out = tdr_return_short_consensus(results, strong_open_sig, tries);
        out.fault_found      = true;
        out.is_short         = true;
        out.distance_m       = 0.0f;
        out.unstable         = false;
        out.consensus_strong = true;
        if (out.launch_index < 0)
            out.launch_index = TDR_PULSE_ON;
        if (out.reflect_index < 0)
            out.reflect_index = out.launch_index;
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
        g_stable_dbg.rule = "SHORT";
        TDR_DBG("TDR path=0.0m SHORT rule=SHORT gp3_median md=%d\n", median_delta);
#endif
        return out;
    }

    // Bred puls + zmax>=1.5 m: OPEN med zmax-afstand (fx ~3 m efter kalibrering)
    if (!for_calibrate &&
        !gp3_median_short &&
        !hw_connector_short &&
        max_open_zone_dist >= TDR_OPEN_DISPLAY_DIST_LO &&
        width_median_early >= TDR_WIDE_CABLE_PULSE_MIN) {
        TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                  tries, open_delta_hi, median_open_dist_m);
        if (out.distance_m < TDR_OPEN_DISPLAY_DIST_LO ||
            max_open_zone_dist > out.distance_m)
            out.distance_m = max_open_zone_dist;
        tdr_normalize_open_distance(out);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
        g_stable_dbg.rule = "OPEN";
#endif
        return out;
    }

    // Fallback OPEN: >=3/6 shots med kant + bred puls, bedste delta 2-6
    if (!for_calibrate && fallback_open) {
        TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                  tries, open_delta_hi, median_open_dist_m);
        tdr_normalize_open_distance(out);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
        g_stable_dbg.rule = "OPEN";
#endif
        return out;
    }

    bool short_consensus     = false;
    bool short_blocks_open   = false;
    bool open_cable_wins     = false;

    if (max_open_zone_dist > TDR_MAX_DISTANCE_M)
        max_open_zone_dist = TDR_MAX_DISTANCE_M;

    float consensus_dist_m = tdr_consensus_distance_m(results, median_delta,
                                                      reflect_median, tries);
    if (max_open_zone_dist > TDR_MAX_DISTANCE_M)
        max_open_zone_dist = TDR_MAX_DISTANCE_M;
    if (tdr_open_cable_consensus_ok(median_delta, shot_delta, tries) &&
        max_open_zone_dist > consensus_dist_m &&
        max_open_zone_dist <= TDR_OPEN_DISPLAY_DIST_HI)
        consensus_dist_m = max_open_zone_dist;

    const bool open_consensus =
        !for_calibrate &&
        (tdr_open_cable_consensus_ok(median_delta, shot_delta, tries) ||
         fallback_open);
    const bool dist_forces_open =
        !for_calibrate && open_consensus &&
        (tdr_distance_forces_open(consensus_dist_m) ||
         (max_open_zone_dist >= TDR_SHORT_BLOCK_ZONE_DIST_M &&
          max_open_zone_dist <= TDR_OPEN_DISPLAY_DIST_HI));
    const bool dist_allows_short =
        !for_calibrate &&
        tdr_distance_allows_short(consensus_dist_m) &&
        max_open_zone_dist < TDR_SHORT_BLOCK_ZONE_DIST_M;

    // Afstand + OPEN-konsensus (median 3-5 eller >=4× d>=3) — ikke kun d=2 / zmax alene
    if (dist_forces_open && !cable_end_short) {
        for (int i = 0; i < tries; i++)
            results[i].is_short = false;
        TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                  tries, open_delta_hi, median_open_dist_m);
        tdr_normalize_open_distance(out);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
        g_stable_dbg.rule = "OPEN";
#endif
        return out;
    }

    if (median_delta >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO) {
        for (int i = 0; i < tries; i++)
            results[i].is_short = false;
    }

    bool consensus_median_short = median_delta <= TDR_SHORT_DELTA_MAX;
    bool consensus_median_open = median_delta >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO &&
                                 median_delta <= open_delta_hi;
    bool force_typical_3m_open = tdr_median_typical_3m_open(median_delta);
    bool short_allowed = tdr_short_consensus_allowed(median_delta,
                                                     max_shot_delta,
                                                     strong_open_votes,
                                                     max_open_zone_dist);
    bool hw_short_contrib =
        hw_connector_short &&
        median_delta <= TDR_SHORT_DELTA_MAX &&
        reflect_median < TDR_MEDIAN_OPEN_BLOCK_SHORT_LO &&
        dist_allows_short &&
        strong_open_votes < TDR_STABLE_STRONG_OPEN_MAX_FOR_SHORT &&
        max_shot_delta <= TDR_SHORT_MAX_SHOT_DELTA;
    bool gp3_short_contrib =
        short_gp3_votes >= TDR_STABLE_SHORT_MIN_AGREE &&
        max_shot_delta <= TDR_SHORT_MAX_SHOT_DELTA &&
        !tdr_any_shot_delta_ge3(shot_delta, tries);
    bool short_gp3_consensus =
        !for_calibrate && median_delta <= TDR_SHORT_DELTA_MAX &&
        dist_allows_short &&
        strong_open_votes < TDR_STABLE_STRONG_OPEN_MAX_FOR_SHORT &&
        (hw_short_contrib || gp3_short_contrib);

    if (short_gp3_consensus) {
        if (hw_continuity &&
            tdr_cycle_forbids_connector_short(cable_end_zone_dist,
                                            max_strict_late_dist, median_delta)) {
            TdrResult out = tdr_return_cable_end_short(results, open_cable_sig, shot_q,
                                                       tries, open_delta_hi,
                                                       median_open_dist_m,
                                                       cable_end_zone_dist);
            tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
            g_stable_dbg.rule = "SH3";
            TDR_DBG("TDR path=cable-end SHORT (blocked short_gp3) dist=%.2f\n",
                    out.distance_m);
#endif
            return out;
        }
        TdrResult out = tdr_return_short_consensus(results, strong_open_sig, tries);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
        g_stable_dbg.rule = "SHORT";
        TDR_DBG("TDR path=0.0m SHORT rule=SHORT short_gp3 md=%d\n", median_delta);
#endif
        return out;
    }

    if (!for_calibrate && consensus_median_short && dist_allows_short &&
        max_shot_delta <= TDR_SHORT_MAX_SHOT_DELTA &&
        short_shot_votes >= TDR_STABLE_SHORT_MIN_AGREE && short_allowed) {
        if (hw_continuity &&
            tdr_cycle_forbids_connector_short(cable_end_zone_dist,
                                            max_strict_late_dist, median_delta)) {
            TdrResult out = tdr_return_cable_end_short(results, open_cable_sig, shot_q,
                                                       tries, open_delta_hi,
                                                       median_open_dist_m,
                                                       cable_end_zone_dist);
            tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
            g_stable_dbg.rule = "SH3";
#endif
            return out;
        }
        TdrResult out = tdr_return_short_consensus(results, strong_open_sig, tries);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
        g_stable_dbg.rule = "SHORT";
#endif
        return out;
    }

    int found_slots[6];
    int nfound = 0;
    for (int i = 0; i < tries; i++) {
        if (results[i].fault_found)
            found_slots[nfound++] = i;
    }

    if (nfound == 0) {
        short_consensus = short_strict_votes >= TDR_STABLE_SHORT_MIN_AGREE &&
                          short_allowed && consensus_median_short;
        short_blocks_open = short_strict_votes >= TDR_STABLE_SHORT_BLOCK_OPEN &&
                            short_allowed && consensus_median_short;
        open_cable_wins = open_consensus &&
                          (strong_open_votes >= TDR_STABLE_MIN_AGREE ||
                           force_typical_3m_open) &&
                          (consensus_median_open || force_typical_3m_open) &&
                          pulse_cable_shape;

        if (!for_calibrate && force_typical_3m_open) {
            TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                      tries, open_delta_hi, median_open_dist_m);
            tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
            return out;
        }
        if (!for_calibrate && open_cable_wins) {
            TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                      tries, open_delta_hi, median_open_dist_m);
            tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
            return out;
        }
        if (allow_short && short_consensus &&
            (dist_allows_short || tdr_short_cal_active(for_calibrate, cal_type))) {
            if (hw_continuity &&
                tdr_cycle_forbids_connector_short(cable_end_zone_dist,
                                                max_strict_late_dist, median_delta)) {
                TdrResult out = tdr_return_cable_end_short(results, open_cable_sig, shot_q,
                                                           tries, open_delta_hi,
                                                           median_open_dist_m,
                                                           cable_end_zone_dist);
                tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
                g_stable_dbg.rule = "SH3";
#endif
                return out;
            }
            TdrResult out = tdr_return_short_consensus(results, strong_open_sig, tries);
            tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
            return out;
        }
        if (!for_calibrate && open_stable_votes >= TDR_STABLE_MIN_AGREE &&
            !short_blocks_open) {
            TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                      tries, open_delta_hi, median_open_dist_m);
            tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
            return out;
        }
        TdrResult r = results[tries - 1];
        if (r.diag == TDR_DIAG_NONE)
            tdr_run_hw_diag(r);
        if (tdr_short_cal_active(for_calibrate, cal_type)) {
            if (tdr_hw_connector_shorted()) {
                r.fault_found      = true;
                r.is_short         = true;
                r.no_cable         = false;
                r.weak_signal      = false;
                r.unstable         = false;
                r.consensus_strong = true;
                r.distance_m       = 0.0f;
                if (r.launch_index < 0)
                    r.launch_index = TDR_PULSE_ON;
                if (r.reflect_index < 0)
                    r.reflect_index = r.launch_index;
                tdr_set_vote_debug(r, dbg_md, dbg_so, dbg_sh);
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
                g_stable_dbg.rule = "HW";
#endif
                return r;
            }
            if (gp3_short_cal_consensus) {
                TdrResult out = tdr_return_short_consensus(results, strong_open_sig, tries);
                tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
                g_stable_dbg.rule = "SHORT";
#endif
                return out;
            }
        }
        if (!for_calibrate && no_cable_votes >= TDR_STABLE_NO_CABLE_MIN_AGREE) {
            if (pulse_cable_shape && any_post_edges) {
                TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig,
                                                          shot_q, tries, open_delta_hi, median_open_dist_m);
                tdr_normalize_open_distance(out);
                if (out.fault_found) {
                    tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
                    g_stable_dbg.rule = "OPEN";
#endif
                    return out;
                }
                tdr_set_no_signal(r, true);
            } else {
                tdr_set_no_signal(r, false);
            }
            r.consensus_strong = true;
            tdr_set_vote_debug(r, dbg_md, dbg_so, dbg_sh);
            return r;
        }
        if (pulse_cable_shape && any_post_edges)
            tdr_set_no_signal(r, true);
        else if (tdr_samples_vary(g_samples, 128))
            tdr_set_no_signal(r, true);
        else
            tdr_set_no_signal(r, false);
        tdr_set_vote_debug(r, dbg_md, dbg_so, dbg_sh);
        return r;
    }

    int idxs[6];
    for (int i = 0; i < nfound; i++) {
        TdrResult &t = results[found_slots[i]];
        idxs[i] = t.reflect_index - t.launch_index;
        if (idxs[i] < 0)
            idxs[i] = 0;
    }

    for (int i = 0; i < nfound - 1; i++) {
        for (int j = i + 1; j < nfound; j++) {
            if (idxs[j] < idxs[i]) {
                int t = idxs[i];
                idxs[i] = idxs[j];
                idxs[j] = t;
            }
        }
    }

    int spread = idxs[nfound - 1] - idxs[0];
    int pick   = idxs[nfound / 2];

    bool median_short = pick <= TDR_SHORT_DELTA_MAX;
    bool median_open_cable = pick >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO &&
                             pick <= open_delta_hi;
    if (tdr_median_typical_3m_open(pick))
        force_typical_3m_open = true;
    short_allowed = tdr_short_consensus_allowed(pick, max_shot_delta,
                                                strong_open_votes,
                                                max_open_zone_dist);
    short_consensus = short_strict_votes >= TDR_STABLE_SHORT_MIN_AGREE &&
                      short_allowed && median_short;
    short_blocks_open = short_strict_votes >= TDR_STABLE_SHORT_BLOCK_OPEN &&
                        short_allowed && median_short;
    open_cable_wins = open_consensus &&
                      (strong_open_votes >= TDR_STABLE_MIN_AGREE ||
                       force_typical_3m_open) &&
                      (median_open_cable || force_typical_3m_open) &&
                      pulse_cable_shape;

    if (!for_calibrate && force_typical_3m_open)
        return tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                         tries, open_delta_hi, median_open_dist_m);

    if (!for_calibrate && short_blocks_open) {
        int short_d[6];
        int nshort = 0;
        for (int i = 0; i < tries; i++) {
            if (!results[i].fault_found || results[i].launch_index < 0)
                continue;
            int d = results[i].reflect_index - results[i].launch_index;
            if (d < 0)
                d = 0;
            if (results[i].is_short && d <= TDR_SHORT_DELTA_MAX)
                short_d[nshort++] = d;
        }
        if (nshort > 0) {
            for (int i = 0; i < nshort - 1; i++) {
                for (int j = i + 1; j < nshort; j++) {
                    if (short_d[j] < short_d[i]) {
                        int t = short_d[i];
                        short_d[i] = short_d[j];
                        short_d[j] = t;
                    }
                }
            }
            pick = short_d[nshort / 2];
        }
    }

    TdrResult best = results[found_slots[0]];
    int best_dist  = std::abs((best.reflect_index - best.launch_index) - pick);
    for (int i = 1; i < nfound; i++) {
        TdrResult &c = results[found_slots[i]];
        int cdelta   = c.reflect_index - c.launch_index;
        if (cdelta < 0)
            cdelta = 0;
        int d = std::abs(cdelta - pick);
        if (d < best_dist ||
            (d == best_dist && cdelta < (best.reflect_index - best.launch_index))) {
            best      = c;
            best_dist = d;
        }
    }

    int best_delta = best.reflect_index - best.launch_index;
    if (best_delta < 0)
        best_delta = 0;

    int max_q = 0;
    int n_agree = 0;
    for (int i = 0; i < nfound; i++) {
        int slot = found_slots[i];
        int d    = results[slot].reflect_index - results[slot].launch_index;
        if (d < 0)
            d = 0;
        if (std::abs(d - pick) <= 1) {
            n_agree++;
            if (shot_q[slot] > max_q)
                max_q = shot_q[slot];
        }
    }

    int pop_vals[6];
    int npop = 0;
    for (int i = 0; i < tries; i++) {
        if (open_pop[i] >= 0)
            pop_vals[npop++] = open_pop[i];
    }
    int pop_spread = tdr_int_spread(pop_vals, npop);

    int width_median = width_median_early;
    int width_spread = tdr_int_spread(pulse_w, tries);
    int amp_spread   = tdr_int_spread(open_amp, tries);

    bool median_open_band = pick >= TDR_OPEN_DELTA_LO && pick <= open_delta_hi;
    bool median_blocks_short = !short_blocks_open &&
                               pick >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO &&
                               pick <= open_delta_hi;

    // OPEN (median 3-6 eller >=4 stærk OPEN) før SHORT (median<=1, max delta<=2, stærk OPEN=0)
    if (!for_calibrate && force_typical_3m_open) {
        TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                tries, open_delta_hi, median_open_dist_m);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
        return out;
    }

    if (!for_calibrate && open_cable_wins) {
        TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                tries, open_delta_hi, median_open_dist_m);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
        return out;
    }

    if (allow_short && short_consensus &&
        (dist_allows_short || tdr_short_cal_active(for_calibrate, cal_type))) {
        TdrResult out = tdr_return_short_consensus(results, strong_open_sig, tries);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
        return out;
    }

    if (!for_calibrate && open_stable_votes >= TDR_STABLE_MIN_AGREE &&
        !short_blocks_open) {
        TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                tries, open_delta_hi, median_open_dist_m);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
        return out;
    }

    if (tdr_short_cal_active(for_calibrate, cal_type) &&
        gp3_short_cal_consensus) {
        TdrResult out = tdr_return_short_consensus(results, strong_open_sig, tries);
        tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
        g_stable_dbg.rule = "SHORT";
#endif
        return out;
    }

    if (!for_calibrate && no_cable_votes >= TDR_STABLE_NO_CABLE_MIN_AGREE &&
        open_stable_votes < TDR_STABLE_MIN_AGREE && !short_blocks_open) {
        if (pulse_cable_shape && any_post_edges) {
            TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                      tries, open_delta_hi, median_open_dist_m);
            tdr_normalize_open_distance(out);
            if (out.fault_found) {
                tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
                g_stable_dbg.rule = "OPEN";
#endif
                return out;
            }
            tdr_set_no_signal(best, true);
        } else {
            tdr_set_no_signal(best, false);
        }
        best.consensus_strong = true;
        tdr_set_vote_debug(best, dbg_md, dbg_so, dbg_sh);
        return best;
    }

    if (!for_calibrate && strong_open_votes >= TDR_STABLE_MIN_AGREE &&
        median_open_band && !short_blocks_open) {
        for (int i = 0; i < nfound; i++) {
            int slot = found_slots[i];
            TdrResult &c = results[slot];
            int cdelta = c.reflect_index - c.launch_index;
            if (cdelta < 0)
                cdelta = 0;
            if (strong_open_sig[slot] &&
                cdelta >= TDR_STRONG_OPEN_DELTA_LO && cdelta <= open_delta_hi) {
                c.is_short           = false;
                c.unstable           = false;
                c.consensus_strong   = true;
                return c;
            }
        }
    }

    bool open_band = !best.is_short &&
                     best_delta >= TDR_OPEN_DELTA_LO &&
                     best_delta <= open_delta_hi;
    bool weak_reflection = max_q < TDR_OPEN_MIN_QUALITY;

    // Pulsform: kabel = bred/stabil GP3-puls under sende; tom stik = kort/ustabil
    bool pulse_short = width_median < TDR_CABLE_MIN_PULSE_WIDTH;
    bool width_shape_unstable = width_spread >= TDR_NO_CABLE_WIDTH_SPREAD;
    bool amp_shape_unstable   = amp_spread > TDR_OPEN_AMP_MAX_SPREAD;
    bool pulse_shape_ok = !pulse_short &&
                          width_spread <= TDR_PULSE_WIDTH_STABLE_SPREAD &&
                          amp_spread <= TDR_OPEN_AMP_MAX_SPREAD;

    // Delta/popcount-stabilitet (eksisterende)
    bool delta_stable = spread <= TDR_STABLE_MAX_SPREAD &&
                        n_agree >= TDR_STABLE_MIN_AGREE &&
                        pop_spread <= TDR_OPEN_POP_MAX_SPREAD;
    bool delta_unstable = spread >= TDR_NO_CABLE_SPREAD ||
                          n_agree < 3 ||
                          pop_spread > TDR_OPEN_POP_MAX_SPREAD ||
                          nfound < 3;

    bool pulse_stable = delta_stable && pulse_shape_ok && nfound >= 3;
    bool pulse_bad = pulse_short || width_shape_unstable || amp_shape_unstable ||
                     delta_unstable;

    if (!for_calibrate &&
        (best.reflect_index < 0 || !best.fault_found) &&
        (max_open_zone_dist >= TDR_DIST_FORCE_OPEN_M ||
         strong_open_votes >= TDR_STABLE_MIN_AGREE ||
         open_stable_votes >= TDR_STABLE_MIN_AGREE ||
         force_typical_3m_open || open_cable_wins)) {
        TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                  tries, open_delta_hi, median_open_dist_m);
        if (out.fault_found) {
            tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
            g_stable_dbg.rule = "OPEN";
#endif
            return out;
        }
    }

    if (open_band && weak_reflection) {
        if (tdr_short_cal_active(for_calibrate, cal_type) &&
            (gp3_short_cal_consensus || tdr_hw_connector_shorted())) {
            TdrResult out = tdr_return_short_consensus(results, strong_open_sig, tries);
            tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
            return out;
        }
        if (!for_calibrate &&
            (max_open_zone_dist >= TDR_DIST_FORCE_OPEN_M ||
             strong_open_votes >= TDR_STABLE_MIN_AGREE))
            return tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                             tries, open_delta_hi, median_open_dist_m);
        tdr_set_no_signal(best, true);
        return best;
    }

    // Aldrig OPEN+afstand ved kort/ustabil puls eller ustabil delta
    if (open_band && pulse_bad) {
        if (tdr_short_cal_active(for_calibrate, cal_type) &&
            (gp3_short_cal_consensus || tdr_hw_connector_shorted())) {
            TdrResult out = tdr_return_short_consensus(results, strong_open_sig, tries);
            tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
            return out;
        }
        if (pulse_cable_shape && any_post_edges) {
            TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                      tries, open_delta_hi, median_open_dist_m);
            tdr_normalize_open_distance(out);
            if (out.fault_found) {
                tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
                g_stable_dbg.rule = "OPEN";
#endif
                return out;
            }
            tdr_set_no_signal(best, true);
            tdr_set_vote_debug(best, dbg_md, dbg_so, dbg_sh);
            return best;
        }
        tdr_set_no_signal(best, false);
        tdr_set_vote_debug(best, dbg_md, dbg_so, dbg_sh);
        return best;
    }

    // Stabil puls (bredde + delta) + tydelig refleksion → vis OPEN-afstand
    if (open_band && pulse_stable && !weak_reflection) {
        best.unstable = false;
        if (median_blocks_short)
            best.is_short = false;
        return best;
    }

    if (open_band) {
        if (tdr_short_cal_active(for_calibrate, cal_type) &&
            (gp3_short_cal_consensus || tdr_hw_connector_shorted())) {
            TdrResult out = tdr_return_short_consensus(results, strong_open_sig, tries);
            tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
            return out;
        }
        if (pulse_cable_shape && any_post_edges) {
            TdrResult out = tdr_pick_open_stable_shot(results, open_cable_sig, shot_q,
                                                      tries, open_delta_hi, median_open_dist_m);
            tdr_normalize_open_distance(out);
            if (out.fault_found) {
                tdr_set_vote_debug(out, dbg_md, dbg_so, dbg_sh);
#ifdef TDR_DEBUG
                g_stable_dbg.rule = "OPEN";
#endif
                return out;
            }
            tdr_set_no_signal(best, true);
        } else {
            tdr_set_no_signal(best, false);
        }
        tdr_set_vote_debug(best, dbg_md, dbg_so, dbg_sh);
        return best;
    }

    if (!for_calibrate &&
        (tdr_distance_forces_open(best.distance_m) || dist_forces_open))
        best.is_short = false;

    if (!for_calibrate && best.is_short) {
        if (!dist_allows_short ||
            median_delta >= TDR_MEDIAN_OPEN_BLOCK_SHORT_LO || force_typical_3m_open ||
            !short_allowed)
            best.is_short = false;
        else if (!short_blocks_open &&
            (median_blocks_short || open_stable_votes >= TDR_STABLE_MIN_AGREE ||
             strong_open_votes >= TDR_STABLE_MIN_AGREE || median_open_band))
            best.is_short = false;
        else if (short_strict_votes < TDR_STABLE_SHORT_MIN_AGREE ||
                 short_shot_votes < TDR_STABLE_SHORT_MIN_AGREE)
            best.is_short = false;
        if (!best.is_short && best_delta <= TDR_SHORT_DELTA_MAX &&
            !median_open_band) {
            tdr_set_no_signal(best, weak_reflection || pulse_bad);
            return best;
        }
    }

    if (median_blocks_short || dist_forces_open)
        best.is_short = false;

    if (!for_calibrate && !best.is_short && best.fault_found) {
        int launch_i = best.launch_index >= 0 ? best.launch_index : TDR_PULSE_ON;
        int fn = 128;
        const uint8_t *f = tdr_get_filtered(fn);
        int rd = best.reflect_index - launch_i;
        if (rd <= TDR_SHORT_DELTA_MAX || rd < TDR_DIST_MIN_DELTA ||
            best.distance_m < TDR_DIST_FORCE_OPEN_M ||
            best.distance_m > TDR_OPEN_DISPLAY_DIST_HI) {
            int zone_g = 0;
            int zone_i = tdr_find_preferred_open_reflect(f, fn, launch_i, zone_g);
            if (zone_i < 0)
                zone_i = tdr_find_late_open_peak(f, fn, launch_i, zone_g);
            if (zone_i >= 0) {
                best.reflect_index = zone_i;
                best.launch_index  = launch_i;
                tdr_fill_distance(best, launch_i, zone_i);
            } else if (rd <= TDR_SHORT_DELTA_MAX) {
                tdr_apply_late_open_reflect(best, f, fn);
            }
        }
        tdr_finalize_stable_open(best, results, tries, median_open_dist_m);
    } else if (for_calibrate && best.fault_found && !best.is_short) {
        int launch_i = best.launch_index >= 0 ? best.launch_index : TDR_PULSE_ON;
        int fn = 128;
        const uint8_t *f = tdr_get_filtered(fn);
        int rd = best.reflect_index - launch_i;
        if (rd < 0)
            rd = 0;
        if (rd <= TDR_SHORT_DELTA_MAX || rd < TDR_DIST_MIN_DELTA ||
            best.distance_m < TDR_DIST_FORCE_OPEN_M) {
            int zone_g = 0;
            int zone_i = tdr_find_preferred_open_reflect(f, fn, launch_i, zone_g);
            if (zone_i < 0)
                zone_i = tdr_find_late_open_peak(f, fn, launch_i, zone_g);
            if (zone_i >= 0) {
                best.reflect_index = zone_i;
                best.launch_index  = launch_i;
                best.is_short      = false;
                tdr_fill_distance(best, launch_i, zone_i);
            } else if (rd <= TDR_SHORT_DELTA_MAX) {
                tdr_apply_late_open_reflect(best, f, fn);
            }
        }
        tdr_fill_calibrate_meta(best);
    }

    if (for_calibrate) {
        g_calib_unst.spread               = spread;
        g_calib_unst.nfound               = nfound;
        g_calib_unst.n_agree              = n_agree;
        g_calib_unst.pulse_shape_ok       = pulse_shape_ok;
        g_calib_unst.width_shape_unstable = width_shape_unstable;
        g_calib_unst.amp_shape_unstable   = amp_shape_unstable;
        g_calib_unst.delta_unstable       = delta_unstable;
        best.unstable = false;
    } else {
        if (nfound < 3 || spread > 8)
            best.unstable = true;
        else if (spread <= TDR_STABLE_MAX_SPREAD && pulse_shape_ok)
            best.unstable = false;
    }

    best.consensus_strong =
        open_stable_votes >= TDR_STABLE_MIN_AGREE ||
        no_cable_votes >= TDR_STABLE_NO_CABLE_MIN_AGREE ||
        short_strict_votes >= TDR_STABLE_SHORT_MIN_AGREE;

    tdr_set_vote_debug(best, dbg_md, dbg_so, dbg_sh);
    return best;
}

TdrResult tdr_measure_stable() {
    TdrResult r = tdr_measure_stable_impl(TDR_OPEN_DELTA_HI, false);
    tdr_fixup_stable_result(r);
#ifdef TDR_DEBUG
    tdr_dbg_print_stable(r);
#endif
    return r;
}

#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
static void tdr_dbg_print_stable(const TdrResult &r) {
    int li = r.launch_index >= 0 ? r.launch_index : TDR_PULSE_ON;
    int ri_use = r.reflect_index;
    int d_use = 0;
    if (r.fault_found && li >= 0 && ri_use >= 0) {
        d_use = ri_use - li;
        if (d_use < 0)
            d_use = 0;
    }

    int fn = 128;
    const uint8_t *f = tdr_get_filtered(fn);
    int raw_g = 0;
    int ri_raw = tdr_find_open_zone_edge(f, fn, li, raw_g);
    int d_raw = (ri_raw >= 0) ? (ri_raw - li) : -1;

    int pref_g = 0;
    int ri_pref = tdr_find_preferred_open_reflect(f, fn, li, pref_g);
    int d_pref = (ri_pref >= 0) ? (ri_pref - li) : -1;

    const char *rule = g_stable_dbg.rule ? g_stable_dbg.rule : "-";
    const char *sig = r.no_cable ? "NC" :
                      (r.weak_signal ? "WK" :
                       (r.fault_found ? (r.is_short ? "SH" : "OP") : "--"));

    TDR_DBG("TDR li=%d ri_raw=%d ri_pref=%d ri_use=%d "
            "d_raw=%d d_pref=%d d_use=%d dist=%.2f sig=%s "
            "md=%u so=%u sh=%u hw=%d pw=%d zmax=%.2f rule=%s fix=%d\n",
            li, ri_raw, ri_pref, ri_use,
            d_raw, d_pref, d_use, (double)r.distance_m, sig,
            (unsigned)r.vote_median_delta, (unsigned)r.vote_strong_open,
            (unsigned)r.vote_short,
            g_stable_dbg.hw_short ? 1 : 0,
            g_stable_dbg.pulse_width_median,
            (double)g_stable_dbg.max_open_zone_dist_m,
            rule, g_stable_dbg.fix_applied ? 1 : 0);
}

static bool tdr_calib_truly_no_signal(const TdrResult &r) {
    int pw = r.vote_pulse_width;
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
    if (pw == 0 && g_stable_dbg.pulse_width_median > 0)
        pw = g_stable_dbg.pulse_width_median;
#endif
    return pw < 4 && r.distance_m < 0.5f && g_calib_zmax_m < 0.5f;
}

static const char *tdr_calib_fail_reason(const TdrResult &r, bool calib_ok) {
    if (calib_ok)
        return "ok";
    if (r.no_cable)
        return "no_cable";
    if (r.weak_signal)
        return "weak_reflection";
    if (r.unstable)
        return "unstable";
    if (tdr_calib_truly_no_signal(r))
        return "no_reflection";
    if (!r.fault_found || r.reflect_index <= 0) {
        if (g_calib_zmax_m >= 0.5f || r.vote_pulse_width >= 4)
            return "ref_mismatch";
        return "no_reflection";
    }
    if (r.is_short)
        return "short";
    if (r.diag == TDR_DIAG_GP23_OPEN)
        return "pin_open";
    if (r.diag == TDR_DIAG_GP3_STUCK)
        return "gp3_stuck";
    return "ref_mismatch";
}

void tdr_dbg_print_calibrate(const TdrResult &r, bool calib_ok, float ref_m) {
    int li = r.launch_index >= 0 ? r.launch_index : TDR_PULSE_ON;
    int ri_use = r.reflect_index;
    int d_use = 0;
    if (r.fault_found && li >= 0 && ri_use >= 0) {
        d_use = ri_use - li;
        if (d_use < 0)
            d_use = 0;
    }

    int fn = 128;
    const uint8_t *f = tdr_get_filtered(fn);
    int raw_g = 0;
    int ri_raw = tdr_find_open_zone_edge(f, fn, li, raw_g);
    int d_raw = (ri_raw >= 0) ? (ri_raw - li) : -1;

    int pref_g = 0;
    int ri_pref = tdr_find_preferred_open_reflect(f, fn, li, pref_g);
    int d_pref = (ri_pref >= 0) ? (ri_pref - li) : -1;

    const char *rule = g_stable_dbg.rule ? g_stable_dbg.rule : "-";
    const char *sig = r.no_cable ? "NC" :
                      (r.weak_signal ? "WK" :
                       (r.fault_found ? (r.is_short ? "SH" : "OP") : "--"));
    const char *cal_reason = tdr_calib_fail_reason(r, calib_ok);
    const bool meas_ok = tdr_calibrate_measurement_ok(r);

    const char *unst_reason =
        g_calib_unst_reason ? g_calib_unst_reason : "-";

    printf("CALIB cal_ok=%d meas_ok=%d cal_reason=%s ref_m=%.1f "
           "li=%d ri_raw=%d ri_pref=%d ri_use=%d "
           "d_raw=%d d_pref=%d d_use=%d dist=%.2f sig=%s "
           "md=%u so=%u sh=%u hw=%d pw=%d zmax=%.2f rule=%s fix=%d "
           "unstable=%d unst_reason=%s spread=%d nfound=%d "
           "vf_calc=%.3f vf_new=%.3f edges=%u\n",
           calib_ok ? 1 : 0, meas_ok ? 1 : 0, cal_reason, (double)ref_m,
           li, ri_raw, ri_pref, ri_use,
           d_raw, d_pref, d_use, (double)r.distance_m, sig,
           (unsigned)r.vote_median_delta, (unsigned)r.vote_strong_open,
           (unsigned)r.vote_short,
           g_stable_dbg.hw_short ? 1 : 0,
           g_stable_dbg.pulse_width_median,
           (double)g_stable_dbg.max_open_zone_dist_m,
           rule, g_stable_dbg.fix_applied ? 1 : 0,
           r.unstable ? 1 : 0, unst_reason, g_calib_unst.spread,
           g_calib_unst.nfound,
           (double)g_stable_dbg.vf_calc, (double)g_stable_dbg.vf_new,
           (unsigned)r.cal_edges);
}
#endif

bool tdr_calibrate_measurement_ok(const TdrResult &r) {
    if (!r.fault_found || r.reflect_index <= 0 || r.launch_index < 0)
        return false;
    if (r.no_cable || r.weak_signal || r.unstable || r.is_short)
        return false;
    int delta = r.reflect_index - r.launch_index;
    if (delta < TDR_OPEN_DELTA_LO || delta > TDR_CALIBRATE_OPEN_DELTA_HI)
        return false;
    return true;
}

bool tdr_calibrate_vf_allowed(const TdrResult &r) {
    if (tdr_calibrate_measurement_ok(r))
        return true;
    if (r.no_cable || r.weak_signal || r.unstable || r.is_short)
        return false;
    if (!r.fault_found || r.reflect_index <= 0 || r.launch_index < 0)
        return false;
    const int pw = r.vote_pulse_width;
    if (g_calib_zmax_m >= TDR_CALIB_DIST_LO_M)
        return true;
    if (tdr_calib_dist_in_band(r.distance_m) && pw >= 4)
        return true;
    return false;
}

bool tdr_apply_calibrate_vf(float L_ref_m, TdrResult &r) {
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
    g_stable_dbg.vf_calc = 0.0f;
    g_stable_dbg.vf_new  = 0.0f;
#endif
    if (!r.fault_found || r.reflect_index <= 0)
        return false;

    int li = r.launch_index >= 0 ? r.launch_index : TDR_PULSE_ON;
    int fn = 128;
    const uint8_t *f = tdr_get_filtered(fn);
    int delta = r.reflect_index - li;
    if (delta < TDR_DIST_MIN_DELTA) {
        int zone_i = tdr_calib_find_reflect_fixup(f, fn, li);
        if (zone_i >= 0) {
            tdr_calib_apply_reflect_fixup(r, li, zone_i);
            delta = zone_i - li;
        }
    }

    float meas_dist = r.distance_m;
    if (g_calib_zmax_m >= TDR_CALIB_DIST_LO_M)
        meas_dist = g_calib_zmax_m;
    else if (tdr_calib_dist_in_band(r.distance_m))
        meas_dist = r.distance_m;

    const float C_LIGHT = 299792458.0f;
    const float vf_cur  = g_velocity_factor;
    float vf_calc       = vf_cur;
    float vf_new        = vf_cur;

    if (delta >= TDR_DIST_MIN_DELTA) {
        float Ts_ns   = tdr_get_sample_period_ns();
        float t_ref_s = (float)delta * (Ts_ns * 1e-9f);
        if (t_ref_s > 0.0f)
            vf_calc = (2.0f * L_ref_m) / (C_LIGHT * t_ref_s);
        vf_new = vf_calc;
    }

    auto vf_in_range = [](float v) {
        return std::isfinite(v) && v > 0.4f && v < 0.9f;
    };

    if (!vf_in_range(vf_new) && L_ref_m >= TDR_CALIB_DIST_LO_M &&
        L_ref_m <= TDR_CALIB_DIST_HI_M && tdr_calib_dist_in_band(meas_dist))
        vf_new = vf_cur * meas_dist / L_ref_m;

    if (!vf_in_range(vf_new) && L_ref_m >= TDR_CALIB_DIST_LO_M &&
        L_ref_m <= TDR_CALIB_DIST_HI_M && tdr_calib_dist_in_band(g_calib_zmax_m))
        vf_new = vf_cur * g_calib_zmax_m / L_ref_m;

#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
    g_stable_dbg.vf_calc = vf_calc;
    g_stable_dbg.vf_new  = vf_new;
#endif

    if (!vf_in_range(vf_new))
        return false;

    tdr_set_velocity_factor(vf_new);

    const bool meas_ok = tdr_calibrate_measurement_ok(r);
    const bool dist_ok = tdr_calib_dist_in_band(r.distance_m);
    const bool zmax_ok = tdr_calib_dist_in_band(g_calib_zmax_m);
    return meas_ok && (dist_ok || zmax_ok);
}

bool tdr_calibrate_short_ok(const TdrResult &r) {
    if (r.no_cable || r.weak_signal || r.unstable)
        return false;
    if (!r.fault_found || r.launch_index < 0)
        return false;
    if (!r.is_short)
        return false;
    int delta = r.reflect_index - r.launch_index;
    if (delta < 0)
        delta = 0;
    return delta <= 3;
}

bool tdr_apply_calibrate_short(const TdrResult &r, int *out_delta) {
    if (!tdr_calibrate_short_ok(r))
        return false;

    int delta = 0;
    if (r.reflect_index >= r.launch_index)
        delta = r.reflect_index - r.launch_index;

    g_cal_short_zero  = (int8_t)delta;
    g_cal_short_valid = true;
    if (out_delta)
        *out_delta = delta;
    return true;
}

bool tdr_calibrate_load100_ok(const TdrResult &r) {
    if (r.no_cable || r.weak_signal || r.unstable || r.is_short)
        return false;
    if (!r.fault_found || r.reflect_index <= 0 || r.launch_index < 0)
        return false;
    int delta = r.reflect_index - r.launch_index;
    if (delta < TDR_OPEN_DELTA_LO || delta > TDR_CALIBRATE_OPEN_DELTA_HI)
        return false;
    return true;
}

bool tdr_apply_calibrate_load100(const TdrResult &r, int *out_delta) {
    if (!tdr_calibrate_load100_ok(r))
        return false;

    int delta = r.reflect_index - r.launch_index;
    g_cal_load_delta = (int8_t)delta;
    g_cal_load_valid = true;
    if (out_delta)
        *out_delta = delta;
    return true;
}

TdrResult tdr_measure_for_calibrate(TdrCalibType type) {
    sleep_ms(TDR_CALIBRATE_SETTLE_MS);
    g_calib_zmax_m = -1.0f;
    std::memset(&g_calib_unst, 0, sizeof(g_calib_unst));
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
    g_calib_unst_reason = nullptr;
#endif
    // Calibrate-path: bredere OPEN-bånd; SHORT-cal genbruger stik-SHORT-detektion
    TdrResult r = tdr_measure_stable_impl(TDR_CALIBRATE_OPEN_DELTA_HI, true, type);
    tdr_fixup_calibrate_result(r);
    tdr_calibrate_finalize_unstable(r);
    return r;
}

// ------------------------------------------------------------
// Autogain
// ------------------------------------------------------------
TdrResult tdr_measure_autogain() {
    TdrResult best{};
    bool have_best = false;

    for (int i = 0; i < 8; i++) {
        TdrResult r = tdr_measure_filtered();
        if (r.fault_found) {
            int delta = r.reflect_index - r.launch_index;
            if (delta < 0)
                delta = 0;
            int best_delta = have_best ? (best.reflect_index - best.launch_index) : -1;
            if (!have_best || delta < best_delta) {
                best = r;
                have_best = true;
            }
        }
        sleep_us(200);
    }

    if (!have_best)
        return TdrResult{};

    return best;
}

// ------------------------------------------------------------
// Calibration (færdig, robust, aldrig låser)
// ------------------------------------------------------------
TdrCalibration tdr_calibrate()
{
    TdrCalibration calib{};
    calib.offset_idx = -1;

    const int tries = 8;
    TdrResult best{};
    bool have_best = false;

    for (int i = 0; i < tries; i++) {
        TdrResult r = tdr_measure_filtered();

        if (!r.fault_found) {
            calib.no_cable = true;
            return calib;
        }

        int delta = r.reflect_index - r.launch_index;
        if (delta < 0)
            delta = 0;
        int best_delta = have_best ? (best.reflect_index - best.launch_index) : -1;
        if (!have_best || delta < best_delta) {
            best = r;
            have_best = true;
        }

        sleep_us(200);
    }

    if (!have_best) {
        calib.no_cable = true;
        return calib;
    }

    int offset = best.reflect_index - best.launch_index;
    if (offset < 0)
        offset = 0;

    if (best.is_short) {
        calib.short_fault = true;
        calib.offset_idx  = offset;
        return calib;
    }

    calib.ok         = true;
    calib.offset_idx = offset;
    return calib;
}

// ------------------------------------------------------------
// UI samples
// ------------------------------------------------------------
const uint8_t* tdr_get_samples(int &n) {
    n = 128;
    return g_samples;
}

const uint8_t* tdr_get_filtered(int &n) {
    n = 128;
    return g_filtered;
}

#ifdef TDR_VOTE_SELFTEST
// Syntetiske waveforms (launch=TDR_PULSE_ON) — kør med -DTDR_VOTE_SELFTEST
static void tdr_selftest_fill_short(uint8_t *s, int n, int launch) {
    memset(s, 0, (size_t)n);
    int pulse_end = g_capture_pulse_off;
    if (pulse_end >= n)
        pulse_end = n - 1;
    for (int i = launch; i <= pulse_end; i++)
        s[i] = 1;
}

static void tdr_selftest_fill_3m_open(uint8_t *s, int n, int launch) {
    memset(s, 0, (size_t)n);
    s[launch] = 1;
    int q1 = launch + TDR_QUIET_REFL_LO;
    int q2 = launch + TDR_QUIET_REFL_HI;
    int e  = launch + TDR_QUIET_EDGE_LO;
    if (e < n)
        s[e] = 1;
    if (e + 1 < n)
        s[e + 1] = 1;
    (void)q1;
    (void)q2;
}

bool tdr_vote_logic_selftest() {
    const int launch = TDR_PULSE_ON;
    uint8_t wav[128];
    bool ok = true;

    tdr_selftest_fill_3m_open(wav, 128, launch);
    ok = ok && tdr_shot_reflection_after_quiet(wav, 128, launch);
    ok = ok && tdr_strong_open_cable_sig(wav, 128, launch);
    ok = ok && !tdr_shot_gp3_immediate_follow(wav, 128, launch);
    {
        TdrResult fake{};
        fake.fault_found = true;
        fake.launch_index = launch;
        fake.reflect_index = launch + 4;
        fake.is_short = true;
        int d = tdr_shot_reflection_delta(fake, wav, 128, launch);
        ok = ok && d >= TDR_TYPICAL_3M_MEDIAN_LO;
        ok = ok && !tdr_short_consensus_allowed(d, d, 1);
    }

    tdr_selftest_fill_short(wav, 128, launch);
    ok = ok && tdr_shot_gp3_immediate_follow(wav, 128, launch);
    ok = ok && tdr_strong_open_cable_sig(wav, 128, launch) == false;
  {
        TdrResult fake{};
        fake.fault_found = true;
        fake.launch_index = launch;
        fake.reflect_index = launch;
        fake.is_short = true;
        int d = tdr_shot_reflection_delta(fake, wav, 128, launch);
        ok = ok && d <= TDR_SHORT_DELTA_MAX;
        ok = ok && tdr_short_consensus_allowed(d, d, 0);
    }

    tdr_selftest_fill_short(wav, 128, launch);
    wav[launch + 1] = 0;
    wav[launch + 2] = 0;
    wav[launch + 3] = 1;
    ok = ok && !tdr_shot_gp3_immediate_follow(wav, 128, launch);
    ok = ok && tdr_shot_reflection_after_quiet(wav, 128, launch);
    ok = ok && tdr_shot_cable_end_reflection(wav, 128, launch);
    ok = ok && !tdr_cable_end_short_evidence(-1.0f, TDR_TYPICAL_3M_MEDIAN_LO, false);
    ok = ok && tdr_cable_end_short_evidence(3.0f, 0, true);

    // GP3 følger + stigende kant ~3 m (kortslutning ved kabelende)
    tdr_selftest_fill_short(wav, 128, launch);
    wav[launch + 2] = 0;
    if (launch + 3 < 128)
        wav[launch + 3] = 1;
    ok = ok && tdr_shot_late_refl_distance_m(wav, 128, launch) >=
                 TDR_SHORT_BLOCK_ZONE_DIST_M;
    ok = ok && tdr_shot_strict_late_refl_distance_m(wav, 128, launch) >=
                 TDR_SHORT_BLOCK_ZONE_DIST_M;
    ok = ok && tdr_cable_end_short_evidence(TDR_SHORT_BLOCK_ZONE_DIST_M, 0, true);

    return ok;
}
#endif
