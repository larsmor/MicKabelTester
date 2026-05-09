#include "pico/stdlib.h"
#include "tdr.hpp"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <cmath>
#include <cstring>

// ------------------------------------------------------------
// Hardware-konfiguration
// ------------------------------------------------------------
static const uint TDR_OUT_PIN = 2;   // Puls ud (100 - 200 ohm i serie)
static const uint TDR_IN_PIN  = 3;   // Refleksion ind (via comparator)
static const uint TDR_GROUND_SWITCH_PIN = 14; // 4066 → XLR-1 til stel

static PIO  g_pio = pio0;
static uint g_sm  = 0;
static uint g_offset;

static float g_velocity_factor = 0.66f;
static float g_clkdiv          = 1.0f;

static uint8_t g_samples[128];
static uint8_t g_filtered[128];

// ------------------------------------------------------------
// PIO-program: send puls + sample input
// ------------------------------------------------------------
// Program:
// 0: set pins, 1   ; puls high
// 1: set pins, 0   ; puls low
// 2: in pins, 1    ; sample 1 bit
// 3: jmp 2         ; loop på sampling
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
    g_velocity_factor = cfg.velocity_factor;
    g_clkdiv          = cfg.clkdiv;

    // 4066 ON → XLR-1 til stel
    gpio_init(TDR_GROUND_SWITCH_PIN);
    gpio_set_dir(TDR_GROUND_SWITCH_PIN, GPIO_OUT);
    gpio_put(TDR_GROUND_SWITCH_PIN, 1);

    // PIO pins
    pio_gpio_init(g_pio, TDR_OUT_PIN);
    pio_gpio_init(g_pio, TDR_IN_PIN);

    gpio_pull_down(TDR_IN_PIN);

    // Claim state machine
    g_sm = pio_claim_unused_sm(g_pio, true);

    // OUT-pin som output, IN-pin som input (fra PIO's synspunkt)
    pio_sm_set_pindirs_with_mask(
        g_pio,
        g_sm,
        (1u << TDR_OUT_PIN),          // out-pin som output
        (1u << TDR_OUT_PIN)           // maske
    );
    pio_sm_set_pindirs_with_mask(
        g_pio,
        g_sm,
        0,                            // in-pin som input
        (1u << TDR_IN_PIN)
    );

    // Tilknyt program
    g_offset = pio_add_program(g_pio, &tdr_program);

    pio_sm_config c = pio_get_default_sm_config();

    // SET styrer TDR_OUT_PIN
    sm_config_set_set_pins(&c, TDR_OUT_PIN, 1);
    // IN læser TDR_IN_PIN
    sm_config_set_in_pins(&c, TDR_IN_PIN);

    // Shift 1 bit ind pr. IN-instruktion, auto-push
    sm_config_set_in_shift(&c, true, true, 1);

    // Clock divider
    sm_config_set_clkdiv(&c, g_clkdiv);

    // Init SM
    pio_sm_init(g_pio, g_sm, g_offset, &c);
    pio_sm_set_enabled(g_pio, g_sm, false);
}

// ------------------------------------------------------------
// Deinit
// ------------------------------------------------------------
void tdr_deinit() {
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_sm_unclaim(g_pio, g_sm);

    gpio_put(TDR_GROUND_SWITCH_PIN, 0);

    gpio_init(TDR_OUT_PIN);
    gpio_init(TDR_IN_PIN);

    gpio_set_dir(TDR_OUT_PIN, GPIO_IN);
    gpio_set_dir(TDR_IN_PIN,  GPIO_IN);

    gpio_pull_down(TDR_OUT_PIN);
    gpio_pull_down(TDR_IN_PIN);
}

// ------------------------------------------------------------
// Sample-periode
// ------------------------------------------------------------
float tdr_get_sample_period_ns() {
    float sysclk = (float)clock_get_hz(clk_sys);
    float freq   = sysclk / g_clkdiv;
    return 1e9f / freq;
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

// ------------------------------------------------------------
// Capture samples via PIO
// ------------------------------------------------------------
static void tdr_capture() {
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_sm_clear_fifos(g_pio, g_sm);
    pio_sm_restart(g_pio, g_sm);

    // Hop til programstart
    pio_sm_exec(g_pio, g_sm, pio_encode_jmp(g_offset));

    pio_sm_set_enabled(g_pio, g_sm, true);

    for (int i = 0; i < 128; i++) {
        uint32_t v = pio_sm_get_blocking(g_pio, g_sm);
        g_samples[i] = (v & 1) ? 1 : 0;
    }

    pio_sm_set_enabled(g_pio, g_sm, false);
}

// ------------------------------------------------------------
// Gradient-baseret refleksionsdetektor (0.5–200 m)
// ------------------------------------------------------------
static TdrResult tdr_detect_reflection(const uint8_t *samples, int n)
{
    TdrResult r{};
    r.fault_found   = false;
    r.is_short      = false;
    r.reflect_index = -1;
    r.distance_m    = 0.0f;

    if (n < 4)
        return r;

    // 1) Glidende middelværdi (3-sample smoothing)
    uint8_t smooth[128];
    smooth[0]     = samples[0];
    smooth[n - 1] = samples[n - 1];

    for (int i = 1; i < n - 1; i++) {
        int sum = samples[i - 1] + samples[i] + samples[i + 1];
        smooth[i] = (sum >= 2) ? 1 : 0;
    }

    // 2) Gradient
    int gradient[128];
    gradient[0] = 0;
    for (int i = 1; i < n; i++) {
        gradient[i] = (int)smooth[i] - (int)smooth[i - 1];
    }

    // 3) Find største absolutte gradient
    int best_i = -1;
    int best_g = 0;

    for (int i = 1; i < n; i++) {
        int g = gradient[i];
        if (std::abs(g) > std::abs(best_g)) {
            best_g = g;
            best_i = i;
        }
    }

    if (best_i < 1)
        return r; // ingen tydelig refleksion

    // 4) Refleksion fundet
    r.fault_found   = true;
    r.reflect_index = best_i;

    // 5) Afstand
    float Ts_ns = tdr_get_sample_period_ns();
    float Ts_s  = Ts_ns * 1e-9f;
    float t     = best_i * Ts_s;
    const float C = 299792458.0f;

    r.distance_m = (C * g_velocity_factor * t) / 2.0f;

    // 6) Fault-type
    r.is_short = (best_g < 0);

    return r;
}

// ------------------------------------------------------------
// Raw TDR
// ------------------------------------------------------------
TdrResult tdr_measure() {
    tdr_capture();

    bool all_same = true;
    for (int i = 1; i < 128; i++) {
        if (g_samples[i] != g_samples[0]) {
            all_same = false;
            break;
        }
    }

    if (all_same)
        return TdrResult{}; // fault_found = false → ingen refleksion

    return tdr_detect_reflection(g_samples, 128);
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
TdrResult tdr_measure_filtered() {
    tdr_capture();
    tdr_filter_majority();
    return tdr_detect_reflection(g_filtered, 128);
}

// ------------------------------------------------------------
// Autogain (flere målinger, vælg bedste)
// ------------------------------------------------------------
TdrResult tdr_measure_autogain() {
    TdrResult best{};
    bool have_best = false;

    for (int i = 0; i < 8; i++) {
        TdrResult r = tdr_measure_filtered();
        if (r.fault_found) {
            if (!have_best || r.reflect_index > best.reflect_index) {
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
