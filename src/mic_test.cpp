#include "mic_test.hpp"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include <cstdio>

// ------------------------------------------------------------
// Hardware-konfiguration
// ------------------------------------------------------------
static const uint PIN1 = 10;   // XLR Pin 1
static const uint PIN2 = 11;   // XLR Pin 2
static const uint PIN3 = 12;   // XLR Pin 3

static const uint PIN_TEST_SRC = 13;  // 4.7k til XLR2
static const uint ADC_MIC_PIN  = 26;  // ADC0 (100k til XLR2)

// ------------------------------------------------------------
// Init
// ------------------------------------------------------------
void mic_init() {
    gpio_init(PIN1);
    gpio_init(PIN2);
    gpio_init(PIN3);
    gpio_init(PIN_TEST_SRC);

    gpio_set_dir(PIN1, GPIO_IN);
    gpio_set_dir(PIN2, GPIO_IN);
    gpio_set_dir(PIN3, GPIO_IN);

    gpio_pull_down(PIN1);
    gpio_pull_down(PIN2);
    gpio_pull_down(PIN3);

    gpio_set_dir(PIN_TEST_SRC, GPIO_OUT);
    gpio_put(PIN_TEST_SRC, 0);

    adc_init();
    adc_gpio_init(ADC_MIC_PIN);
}

// ------------------------------------------------------------
// Deinit
// ------------------------------------------------------------
void mic_deinit() {
    gpio_init(PIN1);
    gpio_init(PIN2);
    gpio_init(PIN3);
    gpio_init(PIN_TEST_SRC);

    gpio_set_dir(PIN1, GPIO_IN);
    gpio_set_dir(PIN2, GPIO_IN);
    gpio_set_dir(PIN3, GPIO_IN);
    gpio_set_dir(PIN_TEST_SRC, GPIO_IN);

    gpio_disable_pulls(PIN1);
    gpio_disable_pulls(PIN2);
    gpio_disable_pulls(PIN3);
    gpio_disable_pulls(PIN_TEST_SRC);
}

// ------------------------------------------------------------
// Hjælpefunktion: test om en pin har forbindelse gennem kablet
// ------------------------------------------------------------
static bool test_pin_ok(uint drive_pin, uint sense1, uint sense2)
{
    gpio_set_dir(drive_pin, GPIO_OUT);
    gpio_put(drive_pin, 1);
    sleep_us(80);

    bool ok = gpio_get(sense1) || gpio_get(sense2);

    gpio_put(drive_pin, 0);
    gpio_set_dir(drive_pin, GPIO_IN);
    sleep_us(20);

    return ok;
}

// ------------------------------------------------------------
// Hjælpefunktion: test kortslutning mellem to pins
// ------------------------------------------------------------
static bool test_short_pair(uint a, uint b)
{
    // Nulstil begge pins
    gpio_set_dir(a, GPIO_IN);
    gpio_set_dir(b, GPIO_IN);
    gpio_pull_down(a);
    gpio_pull_down(b);
    sleep_us(20);

    // Drive A, read B
    gpio_set_dir(a, GPIO_OUT);
    gpio_put(a, 1);
    sleep_us(80);
    bool ab = gpio_get(b);

    gpio_put(a, 0);
    gpio_set_dir(a, GPIO_IN);
    sleep_us(20);

    // Drive B, read A
    gpio_set_dir(b, GPIO_OUT);
    gpio_put(b, 1);
    sleep_us(80);
    bool ba = gpio_get(a);

    gpio_put(b, 0);
    gpio_set_dir(b, GPIO_IN);

    return ab || ba;
}

// ------------------------------------------------------------
// AUTO-DETECT: dynamisk + kondensator + kabeltest
// ------------------------------------------------------------
MicResult mic_measure_auto() {
    MicResult r{};
    r.pin_ok[0] = r.pin_ok[1] = r.pin_ok[2] = false;
    r.short_detected = false;
    r.mic_present    = false;

    // --------------------------------------------------------
    // 1) ADC-måling (kondensator-mic)
    // --------------------------------------------------------
    gpio_put(PIN_TEST_SRC, 0);
    sleep_us(200);

    adc_select_input(0);
    uint16_t raw = adc_read();
    float voltage = (raw / 4095.0f) * 3.3f;
    r.adc_voltage = voltage;

    bool looks_like_cond_mic = (voltage > 1.2f);

    // --------------------------------------------------------
    // 2) Continuity-test (KUN gennem kablet)
    // --------------------------------------------------------
    r.pin_ok[0] = test_pin_ok(PIN1, PIN2, PIN3);
    r.pin_ok[1] = test_pin_ok(PIN2, PIN1, PIN3);
    r.pin_ok[2] = test_pin_ok(PIN3, PIN1, PIN2);

    // --------------------------------------------------------
    // 3) Kortslutningstest
    // --------------------------------------------------------
    bool short12 = test_short_pair(PIN1, PIN2);
    bool short13 = test_short_pair(PIN1, PIN3);
    bool short23 = test_short_pair(PIN2, PIN3);

    r.short_detected = short12 || short13 || short23;

    // --------------------------------------------------------
    // 4) Dynamisk mic-detektion
    // --------------------------------------------------------
    bool looks_like_dyn_mic =
        (voltage < 0.3f) &&
        r.pin_ok[0] && r.pin_ok[1] && r.pin_ok[2] &&
        short12 && short13 && short23;

    // --------------------------------------------------------
    // 5) MIC-DETEKTION
    // --------------------------------------------------------
    if (looks_like_cond_mic || looks_like_dyn_mic) {
        r.mic_present    = true;
        r.short_detected = false;
        return r;
    }

    // --------------------------------------------------------
    // 6) Debug (valgfrit)
    // --------------------------------------------------------
    printf("P1=%d P2=%d P3=%d  S12=%d S13=%d S23=%d  ADC=%.3f\n",
           r.pin_ok[0], r.pin_ok[1], r.pin_ok[2],
           short12, short13, short23,
           r.adc_voltage);

    return r;
}
