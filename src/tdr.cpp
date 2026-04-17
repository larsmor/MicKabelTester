#include "tdr.hpp"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <cmath>
#include <cstring>

// ------------------------------------------------------------
// Hardware-konfiguration
// ------------------------------------------------------------
static const uint TDR_OUT_PIN = 2;   // Puls ud
static const uint TDR_IN_PIN  = 3;   // Refleksion ind (via comparator)

static PIO  g_pio = pio0;
static uint g_sm  = 0;
static uint g_offset;

static float g_velocity_factor = 0.66f;
static float g_clkdiv          = 1.0f;

static uint8_t g_samples[128];

// ------------------------------------------------------------
// PIO-program: send puls + sample input
// ------------------------------------------------------------
// 0: set pins, 1   (puls HIGH)
// 1: set pins, 0   (puls LOW)
// 2: in pins, 1    (sample input-bit)
// 3: jmp 2         (loop samples)
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

    // Giv PIO ejerskab af pins
    pio_gpio_init(g_pio, TDR_OUT_PIN);
    pio_gpio_init(g_pio, TDR_IN_PIN);

    // Sæt pin-retninger for state machine
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, TDR_OUT_PIN, 1, true);   // OUT = output
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, TDR_IN_PIN,  1, false);  // IN  = input

    // Load PIO-program
    g_offset = pio_add_program(g_pio, &tdr_program);

    pio_sm_config c = pio_get_default_sm_config();

    // set pins-instruktionen bruger dette
    sm_config_set_set_pins(&c, TDR_OUT_PIN, 1);

    // in pins-instruktionen bruger dette
    sm_config_set_in_pins(&c, TDR_IN_PIN);

    // Shift-konfiguration:
    //  - shift right
    //  - autopush enable
    //  - push efter 1 bit → ét sample pr. ord
    sm_config_set_in_shift(&c,
                           true,   // shift right
                           true,   // autopush enable
                           1);     // push ved 1 bit

    sm_config_set_clkdiv(&c, g_clkdiv);

    pio_sm_init(g_pio, g_sm, g_offset, &c);
    pio_sm_set_enabled(g_pio, g_sm, false);
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
// Mål TDR (blocking, men deterministisk og uden deadlocks)
// ------------------------------------------------------------
TdrResult tdr_measure() {
    // Ryd og restart SM
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_sm_clear_fifos(g_pio, g_sm);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_exec(g_pio, g_sm, (uint32_t)(0x0000 | g_offset)); // jmp offset

    // Start state machine
    pio_sm_set_enabled(g_pio, g_sm, true);

    // Læs 128 samples (ét bit pr. ord)
    for (int i = 0; i < 128; i++) {
        uint32_t v = pio_sm_get_blocking(g_pio, g_sm);
        g_samples[i] = (v & 1) ? 1 : 0;
    }

    // Stop SM
    pio_sm_set_enabled(g_pio, g_sm, false);

    // Find refleksion
    TdrResult r{};
    r.fault_found   = false;
    r.is_short      = false;
    r.reflect_index = -1;
    r.distance_m    = 0.0f;

    // Ignorer de første få samples (puls/settling)
    for (int i = 5; i < 128; i++) {
        if (g_samples[i] != g_samples[i - 1]) {
            r.fault_found   = true;
            r.reflect_index = i;
            r.is_short      = g_samples[i] == 1;
            break;
        }
    }

    if (r.fault_found) {
        float Ts_s = tdr_get_sample_period_ns() * 1e-9f;
        float t    = r.reflect_index * Ts_s;
        float C    = 299792458.0f;

        r.distance_m = (C * g_velocity_factor * t) / 2.0f;
    }

    return r;
}

// ------------------------------------------------------------
// Returnér samples til UI
// ------------------------------------------------------------
const uint8_t* tdr_get_samples(int &n) {
    n = 128;
    return g_samples;
}
