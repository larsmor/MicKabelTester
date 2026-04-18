#include "ui.hpp"
#include "gfx.hpp"
#include "tdr.hpp"
#include "mic_test.hpp"
#include "battery.hpp"
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

// ------------------------------------------------------------
// Helper: draw rectangle using only drawLine()
// ------------------------------------------------------------
static void gfx_drawRect(GFX &d, int x, int y, int w, int h, int color)
{
    d.drawLine(x, y, x + w, y, color);
    d.drawLine(x, y + h, x + w, y + h, color);
    d.drawLine(x, y, x, y + h, color);
    d.drawLine(x + w, y, x + w, y + h, color);
}

static void gfx_fillRect(GFX &d, int x, int y, int w, int h, int color)
{
    for (int i = 0; i < h; i++)
        d.drawLine(x, y + i, x + w, y + i, color);
}

// ------------------------------------------------------------
// Battery icon (top-right corner)
// ------------------------------------------------------------
void ui_draw_battery(int percent)
{
    auto &d = *g_disp;

    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    int x = d.width() - 22;
    int y = 0;

    // Frame
    gfx_drawRect(d, x, y, 18, 8, 1);

    // Tip
    gfx_fillRect(d, x + 18, y + 2, 2, 4, 1);

    // Fill
    int fill = (percent * 16) / 100;
    if (fill > 0)
        gfx_fillRect(d, x + 1, y + 1, fill, 6, 1);
}

// ------------------------------------------------------------
// Inverted text helper
// ------------------------------------------------------------
void drawStringInverted(GFX &d, int x, int y, const char *s)
{
    while (*s) {
        d.drawChar(x, y, *s, 0);
        x += 6;
        s++;
    }
}

// ------------------------------------------------------------
// DIAGNOSTICS
// ------------------------------------------------------------
void ui_show_diag(GFX &d, const char *ctrl_name)
{
    d.clear();
    d.drawString(0, 0, "OLED DIAG", 1);
    d.drawString(0, 10, "Controller:", 1);
    d.drawString(0, 20, ctrl_name, 1);

    ui_draw_battery(battery_get_percent());
    d.show();
}

// ------------------------------------------------------------
// TDR VIEW
// ------------------------------------------------------------
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
                      r.is_short ? "Short" : "Open",
                      r.distance_m);
    } else {
        std::snprintf(buf, sizeof(buf), "No clear fault");
    }
    d.drawString(0, 52, buf, 1);

    ui_draw_battery(battery_get_percent());
    d.show();
}

// ------------------------------------------------------------
// MIC VIEW
// ------------------------------------------------------------
void ui_draw_mic(const MicResult &m)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, 0, "MIC CABLE TEST", 1);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "Pin1: %s", m.pin_ok[0] ? "OK" : "OPEN");
    d.drawString(0, 16, buf, 1);

    std::snprintf(buf, sizeof(buf), "Pin2: %s", m.pin_ok[1] ? "OK" : "OPEN");
    d.drawString(0, 26, buf, 1);

    std::snprintf(buf, sizeof(buf), "Pin3: %s", m.pin_ok[2] ? "OK" : "OPEN");
    d.drawString(0, 36, buf, 1);

    std::snprintf(buf, sizeof(buf), "Short: %s", m.short_detected ? "YES" : "NO");
    d.drawString(0, 46, buf, 1);

    std::snprintf(buf, sizeof(buf), "Mic: %s", m.mic_present ? "Present" : "None");
    d.drawString(0, 56, buf, 1);

    ui_draw_battery(battery_get_percent());
    d.show();
}

// ------------------------------------------------------------
// PROFILE MENU
// ------------------------------------------------------------
void ui_draw_menu(const char *profile_name,
                  float vf)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, 0, "Profile:", 1);
    d.drawString(0, 12, profile_name, 1);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "vf=%.3f", vf);
    d.drawString(0, 24, buf, 1);

    d.drawString(0, 40, "Turn: next", 1);
    d.drawString(0, 52, "Press: calibrate", 1);

    ui_draw_battery(battery_get_percent());
    d.show();
}

// ------------------------------------------------------------
// CALIBRATION
// ------------------------------------------------------------
void ui_draw_calib(float ref_len_m,
                   bool done,
                   bool ok,
                   const char *status_text)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, 0, "Calibration", 1);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "Ref: %.1fm", ref_len_m);
    d.drawString(0, 16, buf, 1);

    if (!done) {
        d.drawString(0, 32, "Press: measure", 1);
        d.drawString(0, 44, "Turn: +/-1m", 1);
    } else {
        d.drawString(0, 32, status_text, 1);
        d.drawString(0, 44, "Press: back", 1);
    }

    ui_draw_battery(battery_get_percent());
    d.show();
}

// ------------------------------------------------------------
// START MENU (smooth scroll)
// ------------------------------------------------------------
void ui_draw_startmenu(int selection)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, 0,  "CABLE TESTER", 1);
    d.drawString(0, 10, "Select function:", 1);

    const char *items[] = {
        "TDR measurement",
        "MIC test",
        "Profiles",
        "Calibration",
        "Diagnostics"
    };

    // Scroll target logic
    if (selection >= 3)
        scroll_target = (selection - 2) * 10;
    else
        scroll_target = 0;

    // Smooth scroll
    if (scroll_pos < scroll_target)
        scroll_pos += 2;
    else if (scroll_pos > scroll_target)
        scroll_pos -= 2;

    if (abs(scroll_pos - scroll_target) < 2)
        scroll_pos = scroll_target;

    // Draw items
    for (int i = 0; i < 5; i++) {
        int y = 24 + i * 10 - scroll_pos;

        if (y < 24 || y > d.height())
            continue;

        if (i == selection) {
            gfx_fillRect(d, 0, y - 1, d.width(), 10, 1);
            drawStringInverted(d, 4, y, items[i]);
        } else {
            d.drawString(4, y, items[i], 1);
        }
    }

    ui_draw_battery(battery_get_percent());
    d.show();
}

// ------------------------------------------------------------
// PROGRESS BAR
// ------------------------------------------------------------
void ui_show_progress(const char *msg, int steps)
{
    auto &d = *g_disp;

    for (int i = 0; i <= steps; i++) {
        d.clear();
        d.drawString(0, 20, msg, 1);

        d.drawLine(10, 40, 118, 40, 1);
        d.drawLine(10, 50, 118, 50, 1);
        d.drawLine(10, 40, 10, 50, 1);
        d.drawLine(118, 40, 118, 50, 1);

        int w = (i * 106) / steps;
        gfx_fillRect(d, 11, 41, w, 8, 1);

        ui_draw_battery(battery_get_percent());
        d.show();
        sleep_ms(40);
    }
}
