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

    gpio_set_dir(PIN_TEST_SRC, GPIO_OUT);
    gpio_put(PIN_TEST_SRC, 0);

    adc_init();
    adc_gpio_init(ADC_MIC_PIN);
}

// ------------------------------------------------------------
// Deinit (sluk MIC-pins)
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
// AUTO-DETECT: dynamisk + kondensator
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
    // 2) Kabeltest (pin_ok)
    // --------------------------------------------------------
    gpio_put(PIN_TEST_SRC, 1);
    sleep_us(50);

    r.pin_ok[0] = gpio_get(PIN1);
    r.pin_ok[1] = gpio_get(PIN2);
    r.pin_ok[2] = gpio_get(PIN3);

    gpio_put(PIN_TEST_SRC, 0);

    // --------------------------------------------------------
    // 3) Kortslutningstest
    // --------------------------------------------------------
    bool short12 = false;
    bool short13 = false;
    bool short23 = false;

    auto test_output = [&](uint out_pin, uint in1, uint in2, bool &s1, bool &s2) {
        gpio_set_dir(out_pin, GPIO_OUT);
        gpio_put(out_pin, 1);
        sleep_us(50);

        if (gpio_get(in1)) s1 = true;
        if (gpio_get(in2)) s2 = true;

        gpio_put(out_pin, 0);
        gpio_set_dir(out_pin, GPIO_IN);
    };

    test_output(PIN1, PIN2, PIN3, short12, short13);
    test_output(PIN2, PIN1, PIN3, short12, short23);
    test_output(PIN3, PIN1, PIN2, short13, short23);

    // --------------------------------------------------------
    // 4) Dynamisk mic-detektion (din mikrofon)
    // --------------------------------------------------------
    bool looks_like_dyn_mic =
        (voltage < 0.3f) &&       // ingen DC-load
        r.pin_ok[0] &&            // alle pins reagerer
        r.pin_ok[1] &&
        r.pin_ok[2] &&
        short12 && short13 && short23;  // alle tre shorts

    // --------------------------------------------------------
    // 5) MIC-DETEKTION
    // --------------------------------------------------------
    if (looks_like_cond_mic || looks_like_dyn_mic) {
        r.mic_present    = true;
        r.short_detected = false;
        return r;
    }

    // Debug
    printf("short12=%d short13=%d short23=%d pin1=%d pin2=%d pin3=%d ADC=%.3f\n",
           short12, short13, short23,
           r.pin_ok[0], r.pin_ok[1], r.pin_ok[2],
           r.adc_voltage);

    // --------------------------------------------------------
    // 6) KABEL / FEJL
    // --------------------------------------------------------
    r.short_detected = short12 || short13 || short23;
    return r;
}
