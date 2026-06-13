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
#define SETTINGS_VERSION 2u

struct SettingsBlobV1 {
    uint32_t magic;
    uint32_t version;
    int32_t  profile_index;
    float    vf[SETTINGS_PROFILE_COUNT];
    uint32_t crc32;
};

struct SettingsBlob {
    uint32_t magic;
    uint32_t version;
    int32_t  profile_index;
    float    vf[SETTINGS_PROFILE_COUNT];
    int8_t   short_zero_delta;
    int8_t   load100_delta;
    uint8_t  cal_flags;
    uint8_t  reserved;
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
    if (blob.magic != SETTINGS_MAGIC)
        return false;
    if (blob.version != SETTINGS_VERSION)
        return false;

    SettingsBlob tmp = blob;
    tmp.crc32        = 0;
    uint32_t expect  = settings_crc32(reinterpret_cast<const uint8_t *>(&tmp),
                                      sizeof(tmp));
    return blob.crc32 == expect;
}

static bool settings_blob_v1_valid(const SettingsBlobV1 &blob) {
    if (blob.magic != SETTINGS_MAGIC || blob.version != 1u)
        return false;

    SettingsBlobV1 tmp = blob;
    tmp.crc32          = 0;
    uint32_t expect    = settings_crc32(reinterpret_cast<const uint8_t *>(&tmp),
                                        sizeof(tmp));
    return blob.crc32 == expect;
}

static bool settings_vf_sane(float vf) {
    return vf > 0.4f && vf < 0.9f;
}

static bool settings_cal_sane(const SettingsBlob &blob) {
    if (blob.short_zero_delta < -1 || blob.short_zero_delta > 3)
        return false;
    if (blob.load100_delta < -1 || blob.load100_delta > 12)
        return false;
    if ((blob.cal_flags & SETTINGS_CAL_SHORT_VALID) &&
        blob.short_zero_delta < 0)
        return false;
    if ((blob.cal_flags & SETTINGS_CAL_LOAD_VALID) &&
        blob.load100_delta < 0)
        return false;
    return true;
}

static void settings_cal_defaults(SettingsCalibration &cal) {
    cal.short_zero_delta = -1;
    cal.load100_delta    = -1;
    cal.flags            = 0;
}

static void settings_cal_from_blob(const SettingsBlob &blob,
                                   SettingsCalibration &cal) {
    settings_cal_defaults(cal);
    cal.short_zero_delta = blob.short_zero_delta;
    cal.load100_delta    = blob.load100_delta;
    cal.flags            = blob.cal_flags;
}

static bool settings_read_blob(SettingsBlob &out) {
    const auto *flash_blob =
        reinterpret_cast<const SettingsBlob *>(XIP_BASE + SETTINGS_FLASH_OFFSET);

    if (settings_blob_valid(*flash_blob)) {
        out = *flash_blob;
        return true;
    }

    const auto *flash_v1 =
        reinterpret_cast<const SettingsBlobV1 *>(XIP_BASE + SETTINGS_FLASH_OFFSET);
    if (!settings_blob_v1_valid(*flash_v1))
        return false;

    out = SettingsBlob{};
    out.magic         = flash_v1->magic;
    out.version       = SETTINGS_VERSION;
    out.profile_index = flash_v1->profile_index;
    for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++)
        out.vf[i] = flash_v1->vf[i];
    out.short_zero_delta = -1;
    out.load100_delta    = -1;
    out.cal_flags        = 0;
    out.reserved         = 0;
    out.crc32            = 0;
    return true;
}

bool settings_load(int *profile_index, float vf[SETTINGS_PROFILE_COUNT],
                   SettingsCalibration *cal) {
    SettingsBlob blob{};
    if (!settings_read_blob(blob))
        return false;

    if (blob.profile_index < 0 ||
        blob.profile_index >= SETTINGS_PROFILE_COUNT)
        return false;

    for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++) {
        if (!settings_vf_sane(blob.vf[i]))
            return false;
    }

    if (!settings_cal_sane(blob))
        return false;

    *profile_index = blob.profile_index;
    for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++)
        vf[i] = blob.vf[i];

    if (cal)
        settings_cal_from_blob(blob, *cal);

    return true;
}

bool settings_save(int profile_index, const float vf[SETTINGS_PROFILE_COUNT],
                   const SettingsCalibration *cal) {
    if (profile_index < 0 || profile_index >= SETTINGS_PROFILE_COUNT)
        return false;

    for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++) {
        if (!settings_vf_sane(vf[i]))
            return false;
    }

    SettingsCalibration cal_tmp{};
    settings_cal_defaults(cal_tmp);
    if (cal)
        cal_tmp = *cal;

    if (cal_tmp.short_zero_delta < -1 || cal_tmp.short_zero_delta > 3)
        return false;
    if (cal_tmp.load100_delta < -1 || cal_tmp.load100_delta > 12)
        return false;

    SettingsBlob blob{};
    blob.magic         = SETTINGS_MAGIC;
    blob.version       = SETTINGS_VERSION;
    blob.profile_index = profile_index;
    for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++)
        blob.vf[i] = vf[i];
    blob.short_zero_delta = cal_tmp.short_zero_delta;
    blob.load100_delta    = cal_tmp.load100_delta;
    blob.cal_flags        = cal_tmp.flags;
    blob.reserved         = 0;
    blob.crc32            = 0;
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

static bool settings_sector_empty(const SettingsBlob &blob) {
    const auto *raw = reinterpret_cast<const uint8_t *>(&blob);
    for (size_t i = 0; i < sizeof(SettingsBlob); i++) {
        if (raw[i] != 0xffu)
            return false;
    }
    return true;
}

SettingsFlashStatus settings_verify_flash() {
    const auto *flash_blob =
        reinterpret_cast<const SettingsBlob *>(XIP_BASE + SETTINGS_FLASH_OFFSET);

    if (settings_sector_empty(*flash_blob))
        return SettingsFlashStatus::Empty;

    if (flash_blob->magic != SETTINGS_MAGIC)
        return SettingsFlashStatus::BadMagic;

    if (flash_blob->version == 1u) {
        const auto *flash_v1 =
            reinterpret_cast<const SettingsBlobV1 *>(XIP_BASE + SETTINGS_FLASH_OFFSET);
        if (!settings_blob_v1_valid(*flash_v1))
            return SettingsFlashStatus::BadCrc;
        if (flash_v1->profile_index < 0 ||
            flash_v1->profile_index >= SETTINGS_PROFILE_COUNT)
            return SettingsFlashStatus::BadData;
        for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++) {
            if (!settings_vf_sane(flash_v1->vf[i]))
                return SettingsFlashStatus::BadData;
        }
        return SettingsFlashStatus::Ok;
    }

    if (flash_blob->version != SETTINGS_VERSION)
        return SettingsFlashStatus::BadData;

    if (!settings_blob_valid(*flash_blob))
        return SettingsFlashStatus::BadCrc;

    if (flash_blob->profile_index < 0 ||
        flash_blob->profile_index >= SETTINGS_PROFILE_COUNT)
        return SettingsFlashStatus::BadData;

    for (int i = 0; i < SETTINGS_PROFILE_COUNT; i++) {
        if (!settings_vf_sane(flash_blob->vf[i]))
            return SettingsFlashStatus::BadData;
    }

    if (!settings_cal_sane(*flash_blob))
        return SettingsFlashStatus::BadData;

    return SettingsFlashStatus::Ok;
}

bool settings_factory_reset() {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(SETTINGS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    return true;
}
