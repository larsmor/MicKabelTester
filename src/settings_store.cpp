#include "settings_store.hpp"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include <cstring>

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

// Sidste flash-sektor (2 MB Pico) — uden for programmet
#define SETTINGS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

#define SETTINGS_MAGIC   0x4B544553u  // "KTES"
#define SETTINGS_VERSION 1u

struct SettingsBlob {
    uint32_t magic;
    uint32_t version;
    int32_t  profile_index;
    float    vf[SETTINGS_PROFILE_COUNT];
    uint32_t crc32;
};

static uint32_t settings_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xedb88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return ~crc;
}

static bool settings_blob_valid(const SettingsBlob &blob) {
    if (blob.magic != SETTINGS_MAGIC || blob.version != SETTINGS_VERSION)
        return false;

    SettingsBlob tmp = blob;
    tmp.crc32        = 0;
    uint32_t expect  = settings_crc32(reinterpret_cast<const uint8_t *>(&tmp),
                                      sizeof(tmp));
    return blob.crc32 == expect;
}

static bool settings_vf_sane(float vf) {
    return vf > 0.4f && vf < 0.9f;
}

bool settings_load(int *profile_index, float vf[SETTINGS_PROFILE_COUNT]) {
    const auto *flash_blob =
        reinterpret_cast<const SettingsBlob *>(XIP_BASE + SETTINGS_FLASH_OFFSET);

    if (!settings_blob_valid(*flash_blob))
        return false;

    if (flash_blob->profile_index < 0 ||
        flash_blob->profile_index >= SETTINGS_PROFILE_COUNT)
        return false;

    for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++) {
        if (!settings_vf_sane(flash_blob->vf[i]))
            return false;
    }

    *profile_index = flash_blob->profile_index;
    for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++)
        vf[i] = flash_blob->vf[i];

    return true;
}

bool settings_save(int profile_index, const float vf[SETTINGS_PROFILE_COUNT]) {
    if (profile_index < 0 || profile_index >= SETTINGS_PROFILE_COUNT)
        return false;

    for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++) {
        if (!settings_vf_sane(vf[i]))
            return false;
    }

    SettingsBlob blob{};
    blob.magic         = SETTINGS_MAGIC;
    blob.version       = SETTINGS_VERSION;
    blob.profile_index = profile_index;
    for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++)
        blob.vf[i] = vf[i];
    blob.crc32 = 0;
    blob.crc32 = settings_crc32(reinterpret_cast<const uint8_t *>(&blob),
                                sizeof(blob));

    alignas(4) static uint8_t sector_buf[FLASH_SECTOR_SIZE];
    std::memset(sector_buf, 0xff, sizeof(sector_buf));
    std::memcpy(sector_buf, &blob, sizeof(blob));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(SETTINGS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_FLASH_OFFSET, sector_buf, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    return true;
}
