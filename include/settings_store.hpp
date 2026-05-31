#pragma once

#include <cstdint>

// Antal kabelprofiler (matcher main.cpp)
constexpr int SETTINGS_PROFILE_COUNT = 4;

// Indlæs fra flash. Returnerer false ved første boot / ugyldig data (brug defaults).
bool settings_load(int *profile_index, float vf[SETTINGS_PROFILE_COUNT]);

// Gem profil-index og alle VF-værdier til flash.
bool settings_save(int profile_index, const float vf[SETTINGS_PROFILE_COUNT]);

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
