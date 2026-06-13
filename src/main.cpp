#include "pico/stdlib.h"
#include <cstdio>
#include <cmath>
#include "hardware/i2c.h"
#include "hardware/adc.h"

#include "oled.hpp"
#include "TwistController.hpp"
#include "tdr.hpp"
#include "mic_test.hpp"
#include "input.hpp"
#include "ui.hpp"
#include "settings_store.hpp"

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

OLED            display(i2c0, 0x3C);
TwistController twist(i2c0, 0x3F);

struct CableProfile {
    const char *name;
    float vf;
};

static CableProfile profiles[] = {
    {"XLR Std", 0.66f},
    {"Install", 0.70f},
    {"Foam PE", 0.78f},
    {"Custom", 0.72f},
};

static int g_profile_index = 0;
static const int PROFILE_COUNT = 4;

static SettingsCalibration g_cal{};

static void cal_apply_to_tdr() {
    TdrCalibState ts{};
    ts.short_zero_delta = g_cal.short_zero_delta;
    ts.load100_delta    = g_cal.load100_delta;
    ts.short_valid      = (g_cal.flags & SETTINGS_CAL_SHORT_VALID) != 0;
    ts.load_valid       = (g_cal.flags & SETTINGS_CAL_LOAD_VALID) != 0;
    tdr_set_calibration(ts);
}

static void cal_collect_from_tdr() {
    TdrCalibState ts{};
    tdr_get_calibration(ts);
    g_cal.short_zero_delta = ts.short_zero_delta;
    g_cal.load100_delta    = ts.load100_delta;
    g_cal.flags            = 0;
    if (ts.short_valid)
        g_cal.flags |= SETTINGS_CAL_SHORT_VALID;
    if (ts.load_valid)
        g_cal.flags |= SETTINGS_CAL_LOAD_VALID;
}

static void profiles_collect_vf(float vf[SETTINGS_PROFILE_COUNT]) {
    for (int i = 0; i < PROFILE_COUNT; i++)
        vf[i] = profiles[i].vf;
}

static void profiles_apply_vf(const float vf[SETTINGS_PROFILE_COUNT]) {
    for (int i = 0; i < PROFILE_COUNT; i++)
        profiles[i].vf = vf[i];
}

static void profiles_save_to_flash() {
    float vf[SETTINGS_PROFILE_COUNT];
    profiles_collect_vf(vf);
    cal_collect_from_tdr();
    if (settings_save(g_profile_index, vf, &g_cal))
        printf("Settings saved to flash\n");
}

static void profiles_restore_defaults() {
    profiles[0].vf = 0.66f;
    profiles[1].vf = 0.70f;
    profiles[2].vf = 0.78f;
    profiles[3].vf = 0.72f;
    g_profile_index = 0;
    g_cal.short_zero_delta = -1;
    g_cal.load100_delta    = -1;
    g_cal.flags            = 0;
    cal_apply_to_tdr();
}

static void flash_status_message(char *line1, size_t n1,
                                 char *line2, size_t n2,
                                 SettingsFlashStatus st) {
    line2[0] = '\0';

    switch (st) {
    case SettingsFlashStatus::Ok:
        std::snprintf(line1, n1, "Flash OK");
        std::snprintf(line2, n2, "Data valid");
        break;
    case SettingsFlashStatus::Empty:
        std::snprintf(line1, n1, "Flash empty");
        std::snprintf(line2, n2, "Defaults in use");
        break;
    case SettingsFlashStatus::BadMagic:
        std::snprintf(line1, n1, "Invalid magic");
        break;
    case SettingsFlashStatus::BadCrc:
        std::snprintf(line1, n1, "CRC error");
        break;
    case SettingsFlashStatus::BadData:
        std::snprintf(line1, n1, "Bad data");
        break;
    }
}

enum class AppState {
    StartupMenu,
    SettingsMenu,
    TdrView,
    MicView,
    MenuProfiles,
    CalibMenu,
    Calibrate,
    FactoryReset,
    VerifyFlash
};

static void tdr_start_for_profile(const CableProfile &prof) {
    TdrConfig cfg{};
    cfg.clkdiv          = 1.0f;
    cfg.velocity_factor = prof.vf;
    tdr_init(cfg);
}

static constexpr int TDR_UI_HOLD_CYCLES = 4;
static constexpr int TDR_UI_OPEN_HOLD_STREAK = 4;
static constexpr int TDR_UI_OPEN_HOLD_CYCLES = 40;
static constexpr int TDR_UI_STRONG_OPEN_HOLD = 60;
static constexpr int TDR_UI_CABLE_PULSE_MIN = 13;
static constexpr float TDR_UI_OPEN_DIST_LO = 1.5f;
static constexpr float TDR_UI_OPEN_DIST_HI = 5.0f;
static constexpr float TDR_UI_OPEN_DIST_MIN = 0.5f;

static float s_last_open_dist = 0.0f;

static bool tdr_ui_good_open(const TdrResult &r) {
    return r.fault_found && !r.is_short && !r.no_cable &&
           r.distance_m >= TDR_UI_OPEN_DIST_LO &&
           r.distance_m <= TDR_UI_OPEN_DIST_HI;
}

static bool tdr_ui_bogus_open(const TdrResult &r) {
    return r.fault_found && !r.is_short && !r.no_cable &&
           r.distance_m < TDR_UI_OPEN_DIST_MIN;
}

static bool tdr_ui_strong_open(const TdrResult &r) {
    return r.consensus_strong && tdr_ui_good_open(r);
}

static bool tdr_ui_meas_open(const TdrResult &r) {
    return tdr_ui_good_open(r) &&
           (r.vote_median_delta >= 2 || r.vote_strong_open >= 3);
}

static uint8_t tdr_ui_kind(const TdrResult &r) {
    if (r.no_cable || (!r.fault_found && !r.weak_signal && !r.unstable))
        return 0;
    if (r.fault_found && r.is_short)
        return 1;
    if (r.fault_found && !r.is_short)
        return 2;
    return 3;
}

static void calib_status_message(char *line1, size_t n1,
                                 char *line2, size_t n2,
                                 TdrCalibType type,
                                 const TdrResult &r, bool cal_ok) {
    line2[0] = '\0';

    if (cal_ok) {
        switch (type) {
        case TdrCalibType::Open:
            std::snprintf(line1, n1, "Open cal OK");
            break;
        case TdrCalibType::Short:
            std::snprintf(line1, n1, "Short cal OK");
            std::snprintf(line2, n2, "Zero saved");
            break;
        case TdrCalibType::Load100:
            std::snprintf(line1, n1, "Load cal OK");
            std::snprintf(line2, n2, "100 ohm saved");
            break;
        }
        return;
    }
    if (type == TdrCalibType::Short) {
        if (r.no_cable || r.weak_signal) {
            std::snprintf(line1, n1, "No short seen");
            std::snprintf(line2, n2, "Short pins 5-6");
            return;
        }
        if (!r.is_short) {
            std::snprintf(line1, n1, "Not short");
            std::snprintf(line2, n2, "Short pins 5-6");
            return;
        }
        std::snprintf(line1, n1, "Short cal fail");
        return;
    }
    if (type == TdrCalibType::Load100) {
        if (r.no_cable) {
            std::snprintf(line1, n1, "No load seen");
            std::snprintf(line2, n2, "Connect 100 ohm");
            return;
        }
        if (r.is_short) {
            std::snprintf(line1, n1, "Short not load");
            std::snprintf(line2, n2, "Use 100 ohm");
            return;
        }
        if (r.weak_signal) {
            std::snprintf(line1, n1, "Weak reflection");
            std::snprintf(line2, n2, "Check 100 ohm");
            return;
        }
        std::snprintf(line1, n1, "Load cal fail");
        return;
    }
    if (r.no_cable) {
        std::snprintf(line1, n1, "No cable");
        std::snprintf(line2, n2, "Mount ref cable");
        return;
    }
    if (r.weak_signal) {
        std::snprintf(line1, n1, "Weak reflection");
        std::snprintf(line2, n2, "Check cable/plug");
        return;
    }
    if (r.unstable) {
        std::snprintf(line1, n1, "Unstable");
        std::snprintf(line2, n2, "Hold still, retry");
        return;
    }
    if (r.vote_pulse_width < 4 && r.distance_m < 0.5f && !r.fault_found) {
        std::snprintf(line1, n1, "No reflection");
        std::snprintf(line2, n2, "Check cable/plug");
        return;
    }
    if (r.is_short) {
        std::snprintf(line1, n1, "Short detected");
        std::snprintf(line2, n2, "Pins 5-6 shorted");
        return;
    }
    if (r.diag == TDR_DIAG_GP23_OPEN) {
        std::snprintf(line1, n1, "Pin 4-6 open?");
        return;
    }
    if (r.diag == TDR_DIAG_GP3_STUCK) {
        std::snprintf(line1, n1, "GP3 stuck?");
        return;
    }
    std::snprintf(line1, n1, "Ref mismatch");
    std::snprintf(line2, n2, "Set Ref to cable");
}

int main() {
    stdio_init_all();
    sleep_ms(200);

    printf("Hello from Pico!\n");

    i2c_init(i2c0, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // ADC init (global)
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);

    display.begin();

    input_init(twist);
    ui_init(display);

    {
        float loaded_vf[SETTINGS_PROFILE_COUNT];
        SettingsCalibration loaded_cal{};
        if (settings_load(&g_profile_index, loaded_vf, &loaded_cal)) {
            profiles_apply_vf(loaded_vf);
            g_cal = loaded_cal;
            printf("Settings loaded from flash (profile %d)\n", g_profile_index);
        } else {
            printf("Settings: flash empty/invalid, using defaults\n");
        }
        cal_apply_to_tdr();
    }

    AppState state      = AppState::StartupMenu;
    float    calib_ref_m  = 3.0f;
    bool     calib_done   = false;
    bool     calib_ok     = false;
    static int menu_sel      = 0;
    static int settings_sel  = 0;
    static int calib_menu_sel = 0;
    static int confirm_sel   = 0;

    static TdrCalibType calib_type = TdrCalibType::Open;

    static TdrResult last_r{};
    static char      calib_line1[22];
    static char      calib_line2[22];
    static char      msg_line1[22];
    static char      msg_line2[22];

    enum class FactoryResetPhase { Confirm, Result };
    static FactoryResetPhase factory_reset_phase = FactoryResetPhase::Confirm;
    static bool              factory_reset_ok      = false;
    static SettingsFlashStatus flash_verify_status = SettingsFlashStatus::Empty;

    while (true) {
        input_update();
        int16_t delta = input_get_delta();
        bool    press = input_pressed_edge();

        CableProfile &prof = profiles[g_profile_index];
        tdr_set_velocity_factor(prof.vf);

        switch (state) {

        // ----------------------------------------------------
        // TDR VIEW
        // ----------------------------------------------------
        case AppState::TdrView: {
            static TdrResult ui_tdr{};
            static int       ui_hold = 0;
            static int       ui_open_streak = 0;
            static int       ui_open_zero_hold = 0;
            static int       ui_strong_open_hold = 0;

            TdrResult r = tdr_measure_stable();

            if (tdr_ui_good_open(r))
                s_last_open_dist = r.distance_m;
            if (tdr_ui_strong_open(r))
                ui_strong_open_hold = TDR_UI_STRONG_OPEN_HOLD;

            if (tdr_ui_meas_open(r))
                ui_open_streak++;
            else
                ui_open_streak = 0;

            const bool hw_short = r.fault_found && r.is_short;
            const bool hw_no_cable =
                r.no_cable && r.vote_pulse_width < TDR_UI_CABLE_PULSE_MIN;

            if (hw_short || hw_no_cable) {
                ui_open_streak        = 0;
                ui_open_zero_hold     = 0;
                ui_strong_open_hold   = 0;
            } else if (r.fault_found && r.is_short) {
                ui_open_streak      = 0;
                ui_open_zero_hold   = 0;
            }

            const bool pin_open_display =
                ui_open_streak >= TDR_UI_OPEN_HOLD_STREAK &&
                tdr_ui_meas_open(ui_tdr);

            const bool open_overrides_short =
                r.fault_found && !r.is_short &&
                ui_tdr.fault_found && ui_tdr.is_short;

            const bool bogus_open = tdr_ui_bogus_open(r);
            const bool cable_present = r.vote_pulse_width >= TDR_UI_CABLE_PULSE_MIN;
            const bool have_last_open =
                s_last_open_dist >= TDR_UI_OPEN_DIST_LO &&
                s_last_open_dist <= TDR_UI_OPEN_DIST_HI;
            const bool reject_no_cable_update =
                r.no_cable && cable_present && have_last_open &&
                !r.is_short && !hw_short;
            const bool r_no_signal =
                r.no_cable || (!r.fault_found && !r.weak_signal && !r.unstable);
            const bool force_open_hold =
                cable_present && have_last_open && r_no_signal &&
                !ui_tdr.is_short && !r.is_short;
            const bool hold_zero_open =
                !ui_tdr.is_short && !r.is_short &&
                ((bogus_open && cable_present && tdr_ui_good_open(ui_tdr)) ||
                 force_open_hold);

            if (hold_zero_open) {
                if (ui_open_zero_hold <= 0)
                    ui_open_zero_hold = TDR_UI_OPEN_HOLD_CYCLES;
            } else if (tdr_ui_good_open(r)) {
                ui_open_zero_hold = 0;
            }

            const bool weak_open_cycle =
                bogus_open || r_no_signal || !tdr_ui_good_open(r) ||
                !r.consensus_strong;
            const bool hold_strong_open =
                ui_strong_open_hold > 0 &&
                tdr_ui_strong_open(ui_tdr) &&
                !hw_short && !hw_no_cable &&
                cable_present && weak_open_cycle;

            if (hold_strong_open)
                ui_strong_open_hold--;
            else if (tdr_ui_strong_open(r))
                ui_strong_open_hold = TDR_UI_STRONG_OPEN_HOLD;

            const bool freeze_open_display =
                (ui_open_zero_hold > 0 &&
                 (hold_zero_open || tdr_ui_good_open(ui_tdr))) ||
                hold_strong_open;

            if (freeze_open_display) {
                if (ui_open_zero_hold > 0)
                    ui_open_zero_hold--;
            } else if (pin_open_display) {
                if (tdr_ui_meas_open(r))
                    ui_tdr = r;
            } else if (open_overrides_short ||
                       (r.consensus_strong && !reject_no_cable_update) ||
                       ui_hold <= 0 ||
                       tdr_ui_kind(r) == tdr_ui_kind(ui_tdr)) {
                if (!bogus_open || !tdr_ui_good_open(ui_tdr))
                    ui_tdr = r;
                ui_hold = TDR_UI_HOLD_CYCLES;
            } else if (ui_hold > 0) {
                ui_hold--;
            } else {
                if (!bogus_open || !tdr_ui_good_open(ui_tdr))
                    ui_tdr = r;
                ui_hold = TDR_UI_HOLD_CYCLES;
            }

            TdrResult show = ui_tdr;
            if ((hold_strong_open || freeze_open_display) &&
                tdr_ui_good_open(ui_tdr) && !show.is_short && !r.is_short) {
                show = ui_tdr;
            } else if (cable_present && have_last_open && !show.is_short && !r.is_short &&
                (show.no_cable ||
                 (!show.fault_found && !show.is_short && !show.weak_signal))) {
                show.fault_found   = true;
                show.is_short      = false;
                show.no_cable      = false;
                show.weak_signal   = false;
                show.distance_m    = s_last_open_dist;
                show.unstable      = r_no_signal && !tdr_ui_good_open(r);
            } else if (cable_present && r_no_signal && !have_last_open) {
                show.no_cable    = false;
                show.unstable    = true;
                show.fault_found = false;
            }

            ui_draw_tdr(show, prof.name, prof.vf);

            if (show.unstable)
                input_set_rgb(255, 255, 0);
            else if (show.fault_found)
                input_set_rgb(255, 0, 0);
            else
                input_set_rgb(0, 255, 0);

            if (press) {
                tdr_deinit();
                state = AppState::StartupMenu;
            }
            break;
        }


        // ----------------------------------------------------
        // MIC VIEW
        // ----------------------------------------------------
        case AppState::MicView: {
            MicResult m = mic_measure_auto();
            ui_draw_mic(m);

            if (m.mic_present) {
                input_set_rgb(0, 255, 0);   // grøn = mic OK
            }
            else if (m.short_detected) {
                input_set_rgb(255, 0, 0);   // rød = kortslutning (kun kabel-mode)
            }
            else if (!m.pin_ok[1]) {
                input_set_rgb(255, 0, 0);   // rød = Pin 2 ikke forbundet
            }
            else {
                input_set_rgb(255, 255, 0); // gul = ingen mic
            }

            if (press) {
                state = AppState::StartupMenu;
            }
            break;
        }

        // ----------------------------------------------------
        // PROFILE MENU
        // ----------------------------------------------------
        case AppState::MenuProfiles: {
            if (delta != 0) {
                if (delta > 0)
                    g_profile_index = (g_profile_index + 1) % PROFILE_COUNT;
                else
                    g_profile_index = (g_profile_index - 1 + PROFILE_COUNT) % PROFILE_COUNT;
            }
            CableProfile &p = profiles[g_profile_index];
            ui_draw_menu(p.name, p.vf);
            input_set_rgb(0, 0, 255);

            if (press) {
                profiles_save_to_flash();
                state = AppState::SettingsMenu;
            }
            break;
        }

        // ----------------------------------------------------
        // CALIBRATION TYPE MENU
        // ----------------------------------------------------
        case AppState::CalibMenu: {
            if (delta != 0) {
                if (delta > 0) calib_menu_sel++;
                else           calib_menu_sel--;

                if (calib_menu_sel < 0) calib_menu_sel = 3;
                if (calib_menu_sel > 3) calib_menu_sel = 0;
            }

            ui_draw_calib_menu(calib_menu_sel);
            input_set_rgb(0, 50, 200);

            if (press) {
                switch (calib_menu_sel) {
                case 0:
                case 1:
                case 2:
                    mic_deinit();
                    tdr_start_for_profile(profiles[g_profile_index]);
                    calib_type = static_cast<TdrCalibType>(calib_menu_sel);
                    calib_ref_m  = 3.0f;
                    calib_done   = false;
                    calib_ok     = false;
                    calib_line1[0] = '\0';
                    calib_line2[0] = '\0';
                    state = AppState::Calibrate;
                    break;
                default:
                    state = AppState::SettingsMenu;
                    break;
                }
            }
            break;
        }

        // ----------------------------------------------------
        // CALIBRATION
        // ----------------------------------------------------
        case AppState::Calibrate: {

            if (!calib_done && calib_type == TdrCalibType::Open && delta != 0) {
                calib_ref_m += (delta > 0) ? 1.0f : -1.0f;
                if (calib_ref_m < 1.0f) calib_ref_m = 1.0f;
            }

            if (!calib_done && press) {
                ui_show_progress("Measuring...", 20);
                TdrResult r = tdr_measure_for_calibrate(calib_type);

                switch (calib_type) {
                case TdrCalibType::Open:
                    if (!tdr_calibrate_vf_allowed(r)) {
                        calib_ok   = false;
                        calib_done = true;
                    } else {
                        calib_ok   = tdr_apply_calibrate_vf(calib_ref_m, r);
                        calib_done = true;

                        if (g_profile_index == 3 && calib_ok) {
                            profiles[3].vf = tdr_get_velocity_factor();
                        }
                    }
                    break;
                case TdrCalibType::Short:
                    calib_ok   = tdr_apply_calibrate_short(r, nullptr);
                    calib_done = true;
                    break;
                case TdrCalibType::Load100:
                    calib_ok   = tdr_apply_calibrate_load100(r, nullptr);
                    calib_done = true;
                    break;
                }

                if (calib_done && calib_ok) {
                    cal_collect_from_tdr();
                    profiles_save_to_flash();
                }
                last_r = r;
#if defined(TDR_DEBUG) || defined(CALIB_DEBUG)
                if (calib_type == TdrCalibType::Open)
                    tdr_dbg_print_calibrate(r, calib_ok, calib_ref_m);
#endif
            }
            else if (calib_done && press) {
                calib_ref_m  = 3.0f;
                calib_done   = false;
                calib_ok     = false;
                tdr_deinit();
                state = AppState::CalibMenu;
            }

            // LED feedback
            if (calib_done) {
                if (!last_r.fault_found && calib_type == TdrCalibType::Open)
                    input_set_rgb(255, 255, 0);   // yellow = no cable
                else if (!calib_ok)
                    input_set_rgb(255, 0, 0);     // red = measurement error
                else
                    input_set_rgb(0, 255, 0);     // green = OK
            } else {
                input_set_rgb(0, 0, 255);
            }

            if (calib_done) {
                calib_status_message(calib_line1, sizeof(calib_line1),
                                     calib_line2, sizeof(calib_line2),
                                     calib_type, last_r, calib_ok);
            } else {
                calib_line1[0] = '\0';
                calib_line2[0] = '\0';
            }

            ui_draw_calib(static_cast<int>(calib_type),
                          calib_ref_m, calib_done, calib_ok,
                          calib_line1, calib_line2);
            break;
        }

        // ----------------------------------------------------
        // SETTINGS MENU
        // ----------------------------------------------------
        case AppState::SettingsMenu: {
            if (delta != 0) {
                if (delta > 0) settings_sel++;
                else           settings_sel--;

                if (settings_sel < 0) settings_sel = 4;
                if (settings_sel > 4) settings_sel = 0;
            }

            ui_draw_settingsmenu(settings_sel);
            input_set_rgb(0, 50, 200);

            if (press) {
                switch (settings_sel) {
                case 0:
                    state = AppState::MenuProfiles;
                    break;
                case 1:
                    calib_menu_sel = 0;
                    state = AppState::CalibMenu;
                    break;
                case 2:
                    confirm_sel = 0;
                    factory_reset_phase = FactoryResetPhase::Confirm;
                    state = AppState::FactoryReset;
                    break;
                case 3:
                    flash_verify_status = settings_verify_flash();
                    flash_status_message(msg_line1, sizeof(msg_line1),
                                         msg_line2, sizeof(msg_line2),
                                         flash_verify_status);
                    state = AppState::VerifyFlash;
                    break;
                case 4:
                    menu_sel = 2;
                    state = AppState::StartupMenu;
                    break;
                }
            }
            break;
        }

        case AppState::FactoryReset: {
            if (factory_reset_phase == FactoryResetPhase::Confirm) {
                if (delta != 0) {
                    if (delta > 0) confirm_sel++;
                    else           confirm_sel--;

                    if (confirm_sel < 0) confirm_sel = 1;
                    if (confirm_sel > 1) confirm_sel = 0;
                }

                ui_draw_confirm(confirm_sel, "Factory reset",
                                "Reset all settings?");
                input_set_rgb(255, 128, 0);

                if (press) {
                    if (confirm_sel == 0) {
                        factory_reset_ok = settings_factory_reset();
                        if (factory_reset_ok)
                            profiles_restore_defaults();
                        std::snprintf(msg_line1, sizeof(msg_line1),
                                      factory_reset_ok ? "Reset OK" : "Reset failed");
                        msg_line2[0] = '\0';
                        factory_reset_phase = FactoryResetPhase::Result;
                    } else {
                        state = AppState::SettingsMenu;
                    }
                }
            } else {
                ui_draw_message("Factory reset", msg_line1, msg_line2);
                input_set_rgb(factory_reset_ok ? 0 : 255,
                              factory_reset_ok ? 255 : 0,
                              0);

                if (press) {
                    factory_reset_phase = FactoryResetPhase::Confirm;
                    state = AppState::SettingsMenu;
                }
            }
            break;
        }

        case AppState::VerifyFlash: {
            ui_draw_message("Verify flash", msg_line1, msg_line2);
            if (flash_verify_status == SettingsFlashStatus::Ok)
                input_set_rgb(0, 255, 0);
            else if (flash_verify_status == SettingsFlashStatus::Empty)
                input_set_rgb(255, 255, 0);
            else
                input_set_rgb(255, 0, 0);

            if (press)
                state = AppState::SettingsMenu;
            break;
        }

        // ----------------------------------------------------
        // START MENU
        // ----------------------------------------------------
        case AppState::StartupMenu: {
            if (delta != 0) {
                if (delta > 0) menu_sel++;
                else           menu_sel--;

                if (menu_sel < 0) menu_sel = 2;
                if (menu_sel > 2) menu_sel = 0;
            }

            ui_draw_startmenu(menu_sel);
            input_set_rgb(0, 50, 200);

            if (press) {
                switch (menu_sel) {
                case 0: { // TDR VIEW
                    ui_show_progress("Starting...", 20);
                    mic_deinit();   // sluk MIC
                    tdr_start_for_profile(profiles[g_profile_index]);
                    state = AppState::TdrView;
                    break;
                }
                case 1: { // MIC VIEW
                    ui_show_progress("Starting...", 20);
                    tdr_deinit();   // sluk TDR
                    mic_init();     // init MIC
                    state = AppState::MicView;
                    break;
                }
                case 2:
                    settings_sel = 0;
                    state = AppState::SettingsMenu;
                    break;
                }
            }
            break;
        }
        }

        sleep_ms(30);
    }
}
