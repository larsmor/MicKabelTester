#pragma once

#include <cstdint>

// Antal kabelprofiler (matcher main.cpp)
constexpr int SETTINGS_PROFILE_COUNT = 4;

struct SettingsCalibration {
    int8_t  short_zero_delta; // kalibreret stik-SHORT delta (0-2), -1 = default
    int8_t  load100_delta;    // kalibreret 100Ω refleksions-delta, -1 = default
    uint8_t flags;            // bit0 short valid, bit1 load100 valid
};

constexpr uint8_t SETTINGS_CAL_SHORT_VALID  = 0x01u;
constexpr uint8_t SETTINGS_CAL_LOAD_VALID   = 0x02u;

// Indlæs fra flash. Returnerer false ved første boot / ugyldig data (brug defaults).
bool settings_load(int *profile_index, float vf[SETTINGS_PROFILE_COUNT],
                   SettingsCalibration *cal = nullptr);

// Gem profil-index, VF og kalibrering til flash.
bool settings_save(int profile_index, const float vf[SETTINGS_PROFILE_COUNT],
                   const SettingsCalibration *cal = nullptr);

enum class SettingsFlashStatus {
    Ok,
    Empty,
    BadMagic,
    BadCrc,
    BadData,
};

// Verificer flash-sektor (magic, CRC, data).
SettingsFlashStatus settings_verify_flash();

// Slet gemte indstillinger (flash-sektor slettes).
bool settings_factory_reset();
