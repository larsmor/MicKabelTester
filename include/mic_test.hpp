#pragma once
#include <cstdint>

struct MicResult {
    bool pin_ok[3];        // Pin 1, 2, 3 OK?
    bool short_detected;   // Kortslutning mellem lederne?
    bool mic_present;      // Er der en mikrofon (DC-load)?
};

void mic_init();
MicResult mic_measure();
