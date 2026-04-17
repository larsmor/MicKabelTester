#include "ui.hpp"
#include "gfx.hpp"
#include "tdr.hpp"        // ← tilføj denne
#include "mic_test.hpp"   // ← og denne
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <pico/stdlib.h>

static GFX *g_disp = nullptr;

// Smooth scroll state for startmenu
static int scroll_pos    = 0;
static int scroll_target = 0;

void ui_init(GFX &disp) {
    g_disp = &disp;
}

// Sort tekst ved at cleare pixels
void drawStringInverted(GFX &d, int x, int y, const char *s)
{
    while (*s) {
        d.drawChar(x, y, *s, 0);
        x += 6;
        s++;
    }
}

void ui_show_diag(GFX &d, const char *ctrl_name)
{
    d.clear();

    d.drawString(0, 0, "OLED DIAG", 1);
    d.drawString(0, 10, "Controller:", 1);
    d.drawString(0, 20, ctrl_name, 1);

    int16_t w = d.width();
    int16_t h = d.height();

    d.drawLine(0, 0, w - 1, 0, 1);
    d.drawLine(0, h - 1, w - 1, h - 1, 1);
    d.drawLine(0, 0, 0, h - 1, 1);
    d.drawLine(w - 1, 0, w - 1, h - 1, 1);

    d.drawLine(0, 0, w - 1, h - 1, 1);
    d.drawLine(0, h - 1, w - 1, 0, 1);

    d.show();
}

void ui_draw_tdr(const TdrResult &r,
                 const char *profile_name,
                 float vf)
{
    auto &d = *g_disp;
    d.clear();

    char buf[32];
    std::snprintf(buf, sizeof(buf), "TDR %s", profile_name);
    d.drawString(0, 0, buf, 1);

    int n;
    const uint8_t *samples = tdr_get_samples(n);

    const int max_x = (n < 128) ? n : 128;
    const int y_mid = 32;
    const int amp   = 12;

    int prev_y = y_mid - (samples[0] ? amp : -amp);

    for (int x = 1; x < max_x; x++) {
        int y = y_mid - (samples[x] ? amp : -amp);
        d.drawLine(x - 1, prev_y, x, y, 1);
        prev_y = y;
    }

    if (r.fault_found && r.reflect_index < max_x) {
        int xr = r.reflect_index;
        d.drawLine(xr, y_mid - amp - 2, xr, y_mid + amp + 2, 1);
    }

    if (r.fault_found) {
        std::snprintf(buf, sizeof(buf), "%s %.1fm",
                      r.is_short ? "Kortslut" : "Afbrydelse",
                      r.distance_m);
    } else {
        std::snprintf(buf, sizeof(buf), "Ingen tydelig fejl");
    }
    d.drawString(0, 52, buf, 1);

    d.show();
}

void ui_draw_mic(const MicResult &m)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, 0, "MIC KABELTEST", 1);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "Pin1: %s", m.pin_ok[0] ? "OK" : "AFBRUDT");
    d.drawString(0, 16, buf, 1);

    std::snprintf(buf, sizeof(buf), "Pin2: %s", m.pin_ok[1] ? "OK" : "AFBRUDT");
    d.drawString(0, 26, buf, 1);

    std::snprintf(buf, sizeof(buf), "Pin3: %s", m.pin_ok[2] ? "OK" : "AFBRUDT");
    d.drawString(0, 36, buf, 1);

    std::snprintf(buf, sizeof(buf), "Kortslut: %s", m.short_detected ? "JA" : "NEJ");
    d.drawString(0, 46, buf, 1);

    std::snprintf(buf, sizeof(buf), "Mic: %s", m.mic_present ? "Tilstede" : "Ingen");
    d.drawString(0, 56, buf, 1);

    d.show();
}

void ui_draw_menu(const char *profile_name,
                  float vf)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, 0, "Profil:", 1);
    d.drawString(0, 12, profile_name, 1);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "vf=%.3f", vf);
    d.drawString(0, 24, buf, 1);

    d.drawString(0, 40, "Drej: naeste", 1);
    d.drawString(0, 52, "Tryk: kalibrer", 1);

    d.show();
}

void ui_draw_calib(float ref_len_m,
                   bool done,
                   bool ok)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, 0, "Kalibrering", 1);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "Ref: %.1fm", ref_len_m);
    d.drawString(0, 16, buf, 1);

    if (!done) {
        d.drawString(0, 32, "Tryk: maaling", 1);
        d.drawString(0, 44, "Drej: +/-1m", 1);
    } else {
        d.drawString(0, 32, ok ? "OK gemt" : "Fejl", 1);
        d.drawString(0, 44, "Tryk: tilbage", 1);
    }

    d.show();
}

void ui_draw_startmenu(int selection)
{
    auto &d = *g_disp;
    d.clear();

    // Fast header
    d.drawString(0, 0,  "KABEL TESTER",     1);
    d.drawString(0, 10, "Select function:", 1);

    const char *items[] = {
        "TDR measurement",
        "MIC test",
        "Profiles",
        "Calibration",
        "Diagnostics"
    };

    // Scroll-target ud fra selection
    if (selection >= 3)
        scroll_target = (selection - 2) * 10;
    else
        scroll_target = 0;

    // Smooth scroll
    if (scroll_pos < scroll_target)
        scroll_pos += 2;
    else if (scroll_pos > scroll_target)
        scroll_pos -= 2;

    if (std::abs(scroll_pos - scroll_target) < 2)
        scroll_pos = scroll_target;

    // Menu starter ved y = 24
    for (int i = 0; i < 5; i++) {
        int y = 24 + i * 10 - scroll_pos;

        // Tegn kun under header
        if (y < 24 || y > d.height())
            continue;

        d.fillRect(0, y - 1, d.width(), 10, 0);

        if (i == selection) {
            d.fillRect(0, y - 1, d.width(), 10, 1);
            drawStringInverted(d, 4, y, items[i]);
        } else {
            d.drawString(4, y, items[i], 1);
        }
    }

    d.show();
}

void ui_show_progress(const char *msg, int steps)
{
    auto &d = *g_disp;

    for (int i = 0; i <= steps; i++) {
        d.clear();

        // Tekst
        d.drawString(0, 20, msg, 1);

        // Ramme (manuelt tegnet)
        d.drawLine(10,     40,     118, 40,     1); // top
        d.drawLine(10,     50,     118, 50,     1); // bund
        d.drawLine(10,     40,     10,  50,     1); // venstre
        d.drawLine(118,    40,     118, 50,     1); // højre

        // Fyld (progress)
        int w = (i * 106) / steps;
        d.fillRect(11, 41, w, 8, 1);

        d.show();
        sleep_ms(40);
    }
}
