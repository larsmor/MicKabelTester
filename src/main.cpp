#include <cstdio>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "oled.hpp"
#include "TwistController.hpp"
#include "tdr.hpp"
#include "mic_test.hpp"
#include "input.hpp"
#include "ui.hpp"

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

enum class AppState {
    StartupMenu,
    TdrView,
    MicView,
    MenuProfiles,
    Calibrate
};

static bool tdr_calibrate_vf(float L_ref_m,
                             const TdrResult &r) {
    if (!r.fault_found || r.reflect_index <= 0) return false;

    const float C_LIGHT = 299792458.0f;
    float Ts_ns = tdr_get_sample_period_ns();
    float t_ref_s = (float)r.reflect_index * (Ts_ns * 1e-9f);
    float vf_new  = (2.0f * L_ref_m) / (C_LIGHT * t_ref_s);

    if (!std::isfinite(vf_new) || vf_new <= 0.4f || vf_new >= 0.9f)
        return false;

    tdr_set_velocity_factor(vf_new);
    return true;
}

int main() {
    stdio_init_all();
    sleep_ms(200);

    i2c_init(i2c0, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    display.begin();

    input_init(twist);
    mic_init();

    TdrConfig cfg{};
    cfg.clkdiv          = 1.0f;
    cfg.velocity_factor = profiles[0].vf;
    tdr_init(cfg);

    ui_init(display);

    AppState state      = AppState::StartupMenu;
    float    calib_ref_m  = 10.0f;
    bool     calib_done   = false;
    bool     calib_ok     = false;
    static int menu_sel   = 0;

    while (true) {
        input_update();
        int16_t delta = input_get_delta();
        bool    press = input_pressed_edge();

        CableProfile &prof = profiles[g_profile_index];
        tdr_set_velocity_factor(prof.vf);

        switch (state) {
        case AppState::TdrView: {
            TdrResult r = tdr_measure();
			ui_draw_tdr(r, prof.name, prof.vf);

            if (r.fault_found)
                input_set_rgb(255, 0, 0);
            else
                input_set_rgb(0, 255, 0);

            // Tryk = tilbage til menu
            if (press) {
				state = AppState::StartupMenu;
            }
            break;
        }
        case AppState::MicView: {
            MicResult m = mic_measure();
            ui_draw_mic(m);

            if (m.short_detected || !m.pin_ok[0] || !m.pin_ok[1] || !m.pin_ok[2])
                input_set_rgb(255, 0, 0);
            else if (m.mic_present)
                input_set_rgb(0, 255, 0);
            else
                input_set_rgb(255, 255, 0);

            // Tryk = tilbage til menu
            if (press) {
                state = AppState::StartupMenu;
            }
            break;
        }
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
                // Tryk = ind i kalibrering
                state        = AppState::Calibrate;
                calib_ref_m  = 10.0f;
                calib_done   = false;
                calib_ok     = false;
            }
            break;
        }
        case AppState::Calibrate: {
            if (!calib_done && delta != 0) {
                calib_ref_m += (delta > 0) ? 1.0f : -1.0f;
                if (calib_ref_m < 1.0f) calib_ref_m = 1.0f;
            }

            if (!calib_done && press) {
                TdrResult r = tdr_measure();
				ui_show_progress("Starter...", 20);
                calib_ok   = tdr_calibrate_vf(calib_ref_m, r);
                calib_done = true;

                if (g_profile_index == 3 && calib_ok) {
                    profiles[3].vf = tdr_get_velocity_factor();
                }
            } else if (calib_done && press) {
                // Efter kalibrering: tilbage til menu
                state = AppState::StartupMenu;
            }

            ui_draw_calib(calib_ref_m, calib_done, calib_ok);
            input_set_rgb(calib_done ? (calib_ok ? 0 : 255) : 0,
                          calib_done ? (calib_ok ? 255 : 0) : 0,
                          255);
            break;
        }
        case AppState::StartupMenu: {
            if (delta != 0) {
                if (delta > 0) menu_sel++;
                else           menu_sel--;

                if (menu_sel < 0) menu_sel = 0;
                if (menu_sel > 4) menu_sel = 4;
            }

            ui_draw_startmenu(menu_sel);
            input_set_rgb(0, 50, 200);
			
            if (press) {
				ui_show_progress("Starter...", 20);
                switch (menu_sel) {
                case 0: state = AppState::TdrView;       break;
                case 1: state = AppState::MicView;       break;
                case 2: state = AppState::MenuProfiles;  break;
                case 3: state = AppState::Calibrate;
                        calib_ref_m  = 10.0f;
                        calib_done   = false;
                        calib_ok     = false;
                        break;
                case 4:
                    ui_show_diag(display, display.controller_name());
                    sleep_ms(1500);
                    state = AppState::StartupMenu;
                    break;
                }
            }
            break;
        }
        }

        sleep_ms(30);
    }
}
