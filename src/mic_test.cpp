#include "mic_test.hpp"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include <cmath>
#include "pico/stdlib.h"

// ------------------------------------------------------------
// Hardware-konfiguration
// ------------------------------------------------------------
static const uint PIN1 = 10;   // XLR Pin 1
static const uint PIN2 = 11;   // XLR Pin 2
static const uint PIN3 = 12;   // XLR Pin 3

static const uint PIN_TEST_SRC = 13;  // Pull-up test source
static const uint ADC_MIC_PIN  = 26;  // ADC0 → mikrofon load test

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
    adc_select_input(0);
}

// ------------------------------------------------------------
// Måling
// ------------------------------------------------------------
MicResult mic_measure() {
    MicResult r{};
    r.pin_ok[0] = false;
    r.pin_ok[1] = false;
    r.pin_ok[2] = false;
    r.short_detected = false;
    r.mic_present    = false;

    // --------------------------------------------------------
    // 1) Test for gennemgang mellem test-source og hver pin
    // --------------------------------------------------------
    gpio_put(PIN_TEST_SRC, 1);
    sleep_us(50);

    bool p1 = gpio_get(PIN1);
    bool p2 = gpio_get(PIN2);
    bool p3 = gpio_get(PIN3);

    gpio_put(PIN_TEST_SRC, 0);

    r.pin_ok[0] = p1;
    r.pin_ok[1] = p2;
    r.pin_ok[2] = p3;

    // --------------------------------------------------------
    // 2) Test for kortslutning mellem lederne
    // --------------------------------------------------------
    int sum = (p1 ? 1 : 0) + (p2 ? 1 : 0) + (p3 ? 1 : 0);

    // Hvis alle tre er høje → sandsynlig kortslutning
    if (sum >= 2) {
        r.short_detected = true;
    }

    // --------------------------------------------------------
    // 3) Test for mikrofon-tilstedeværelse (DC-load)
    // --------------------------------------------------------
    uint16_t raw = adc_read();      // 0–4095
    float voltage = (raw / 4095.0f) * 3.3f;

    // Typisk mikrofon DC-load: 1.5–2.5V
    if (voltage > 1.0f && voltage < 3.0f) {
        r.mic_present = true;
    }

    return r;
}
