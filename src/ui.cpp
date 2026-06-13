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

// 7px font; hold text within 64px (y 0..56)
static constexpr int UI_LINE = 8;
static int ui_y(int line) { return line * UI_LINE; }

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
// OLED TDR GRAPH
// ------------------------------------------------------------
void ui_draw_tdr_graph(GFX &d,
                       const uint8_t *raw,
                       const uint8_t *flt,
                       int n,
                       const TdrResult &r)
{
    TdrResult show = r;
    if (show.fault_found && !show.is_short && !show.no_cable &&
        show.distance_m < 0.5f) {
        show.unstable    = true;
        show.fault_found = false;
    } else if (show.no_cable) {
        show.fault_found = false;
        show.unstable    = false;
        show.distance_m  = 0.0f;
        show.reflect_index = -1;
    }

    d.clear();

    const int gx = 0;
    const int gy = 16;
    const int gw = d.width();
    const int gh = ui_y(7) - gy;

    gfx_drawRect(d, gx, gy, gw - 1, gh - 1, 1);

    auto mapX = [&](int i) {
        if (n <= 1) return gx;
        return gx + (i * (gw - 1)) / (n - 1);
    };

    auto mapY = [&](int v) {
        return gy + gh - 1 - (v ? (gh - 2) : 0);
    };

    // RAW (tynd, midt i båndet)
    for (int i = 1; i < n; i++) {
        int y0 = mapY(raw[i - 1]) - 2;
        int y1 = mapY(raw[i]) - 2;
        d.drawLine(mapX(i - 1), y0, mapX(i), y1, 1);
    }

    // FILTERED (tyk, fuld højde)
    for (int i = 1; i < n; i++) {
        int y0 = mapY(flt[i - 1]);
        int y1 = mapY(flt[i]);
        d.drawLine(mapX(i - 1), y0, mapX(i), y1, 1);
        if (y0 + 1 < gy + gh - 1 && y1 + 1 < gy + gh - 1)
            d.drawLine(mapX(i - 1), y0 + 1, mapX(i), y1 + 1, 1);
    }

    if (show.fault_found && show.reflect_index > 0 && show.reflect_index < n) {
        int x = mapX(show.reflect_index);
        d.drawLine(x, gy, x, gy + gh - 1, 1);

        char buf[32];
        if (show.unstable)
            snprintf(buf, sizeof(buf), "~%.1fm?", show.distance_m);
        else
            snprintf(buf, sizeof(buf), "%.1fm", show.distance_m);
        d.drawString(0, 0, buf, 1);

        if (show.unstable)
            d.drawString(72, 0, "Unstable", 1);
        else
            d.drawString(72, 0, show.is_short ? "SHORT" : "OPEN", 1);
    } else if (show.no_cable) {
        if (show.diag == TDR_DIAG_GROUND_FAULT) {
            d.drawString(0, 0, "Ground fault", 1);
            d.drawString(0, 8, "Pin 4-5/6", 1);
        } else {
            d.drawString(0, 0, "No cable", 1);
        }
    } else if (show.weak_signal) {
        d.drawString(0, 0, "Weak signal", 1);
        d.drawString(0, 8, "No reflection", 1);
    } else if (show.unstable) {
        d.drawString(0, 0, "Unstable", 1);
        d.drawString(0, 8, "Retry measure", 1);
    } else {
        int ones = 0;
        for (int i = 0; i < n; i++)
            if (flt[i]) ones++;
        int raw_ones = 0;
        for (int i = 0; i < n; i++)
            if (raw[i]) raw_ones++;
        if (raw_ones == 0) {
            if (show.diag == TDR_DIAG_GP23_OK) {
                d.drawString(0, 0, "GP2-GP3 path OK", 1);
                d.drawString(0, 8, "Capture fail", 1);
            } else if (show.diag == TDR_DIAG_GP23_OPEN) {
                d.drawString(0, 0, "No signal LO", 1);
                d.drawString(0, 8, "Pin 5-6 open", 1);
            } else if (show.diag == TDR_DIAG_GP3_STUCK) {
                d.drawString(0, 0, "No TDR path", 1);
                d.drawString(0, 8, "6pin 4-6 open", 1);
            } else {
                bool varies = false;
                for (int i = 1; i < n; i++) {
                    if (raw[i] != raw[0]) {
                        varies = true;
                        break;
                    }
                }
                if (varies) {
                    d.drawString(0, 0, "Weak signal", 1);
                    d.drawString(0, 8, "No reflection", 1);
                } else {
                    d.drawString(0, 0, "No signal LO", 1);
                    d.drawString(0, 8, "Capture fail", 1);
                }
            }
        }
        else if (raw_ones == n) {
            d.drawString(0, 0, "No signal HI", 1);
            d.drawString(0, 8, "GP3 stuck?", 1);
        }
        else
            d.drawString(0, 0, "No cable", 1);
    }

    const bool show_vote_debug =
        show.unstable && show.vote_median_delta > 0;
    if (show_vote_debug) {
        char dbg[22];
        snprintf(dbg, sizeof(dbg), "md=%u so=%u sh=%u",
                 (unsigned)show.vote_median_delta,
                 (unsigned)show.vote_strong_open,
                 (unsigned)show.vote_short);
        d.drawString(0, ui_y(7), dbg, 1);
    }

    ui_draw_battery(battery_get_percent());
    d.show();
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

    int n;
    const uint8_t *raw = tdr_get_samples(n);
    const uint8_t *flt = tdr_get_filtered(n);

    ui_draw_tdr_graph(d, raw, flt, n, r);
}

// ------------------------------------------------------------
// MIC VIEW
// ------------------------------------------------------------
void ui_draw_mic(const MicResult &m)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, ui_y(0), "MIC TEST", 1);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "P1:%s", m.pin_ok[0] ? "OK" : "OPEN");
    d.drawString(0, ui_y(1), buf, 1);

    std::snprintf(buf, sizeof(buf), "P2:%s", m.pin_ok[1] ? "OK" : "OPEN");
    d.drawString(0, ui_y(2), buf, 1);

    std::snprintf(buf, sizeof(buf), "P3:%s", m.pin_ok[2] ? "OK" : "OPEN");
    d.drawString(0, ui_y(3), buf, 1);

    std::snprintf(buf, sizeof(buf), "Sh:%s", m.short_detected ? "YES" : "NO");
    d.drawString(0, ui_y(4), buf, 1);

    std::snprintf(buf, sizeof(buf), "Mic:%s", m.mic_present ? "YES" : "NO");
    d.drawString(0, ui_y(5), buf, 1);

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

    d.drawString(0, ui_y(0), "Profile:", 1);
    d.drawString(0, ui_y(1), profile_name, 1);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "vf=%.3f", vf);
    d.drawString(0, ui_y(2), buf, 1);

    d.drawString(0, ui_y(5), "Turn: next", 1);
    d.drawString(0, ui_y(6), "Press: back", 1);

    ui_draw_battery(battery_get_percent());
    d.show();
}

// ------------------------------------------------------------
// CALIBRATION
// ------------------------------------------------------------
void ui_draw_calib_menu(int selection)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, ui_y(0), "Calibration", 1);
    d.drawString(0, ui_y(1), "Select:", 1);

    const char *items[] = {
        "Open",
        "Short",
        "Load 100 ohm",
        "Back"
    };

    const int item_count = 4;
    const int menu_top = ui_y(2);

    for (int i = 0; i < item_count; i++) {
        int y = menu_top + i * UI_LINE;

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

void ui_draw_calib(int calib_type,
                   float ref_len_m,
                   bool done,
                   bool ok,
                   const char *status_line1,
                   const char *status_line2)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, ui_y(0), "Calibration", 1);

    const char *type_names[] = { "Open", "Short", "Load 100 ohm" };
    if (calib_type >= 0 && calib_type <= 2)
        d.drawString(0, ui_y(1), type_names[calib_type], 1);

    if (!done) {
        if (calib_type == 0) {
            char buf[24];
            std::snprintf(buf, sizeof(buf), "Ref: %.1fm", ref_len_m);
            d.drawString(0, ui_y(2), buf, 1);
            d.drawString(0, ui_y(3), "Match cable len", 1);
            d.drawString(0, ui_y(4), "Turn: +/-1m Ref", 1);
            d.drawString(0, ui_y(5), "Press: measure", 1);
        } else if (calib_type == 1) {
            d.drawString(0, ui_y(2), "No cable", 1);
            d.drawString(0, ui_y(3), "Short pins 5-6", 1);
            d.drawString(0, ui_y(4), "At connector", 1);
            d.drawString(0, ui_y(5), "Press: measure", 1);
        } else {
            d.drawString(0, ui_y(2), "No cable", 1);
            d.drawString(0, ui_y(3), "Connect 100 ohm", 1);
            d.drawString(0, ui_y(4), "At connector", 1);
            d.drawString(0, ui_y(5), "Press: measure", 1);
        }
    } else {
        if (status_line1 && status_line1[0])
            d.drawString(0, ui_y(3), status_line1, 1);
        if (status_line2 && status_line2[0])
            d.drawString(0, ui_y(4), status_line2, 1);
        d.drawString(0, ui_y(6), "Press: back", 1);
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

    d.drawString(0, ui_y(0), "CABLE TESTER", 1);
    d.drawString(0, ui_y(1), "Select:", 1);

    const char *items[] = {
        "TDR measurement",
        "MIC test",
        "Settings"
    };

    const int item_count = 3;
    const int menu_top = ui_y(2);
    const int menu_bottom = ui_y(6);
    const int visible_lines = (menu_bottom - menu_top) / UI_LINE + 1;
    const bool scroll_enabled = item_count > visible_lines;

    int scroll_offset = 0;
    if (scroll_enabled) {
        if (selection >= visible_lines - 1)
            scroll_target = (selection - (visible_lines - 2)) * UI_LINE;
        else
            scroll_target = 0;

        if (scroll_pos < scroll_target)
            scroll_pos += 2;
        else if (scroll_pos > scroll_target)
            scroll_pos -= 2;

        if (abs(scroll_pos - scroll_target) < 2)
            scroll_pos = scroll_target;

        scroll_offset = scroll_pos;
    } else {
        scroll_pos = 0;
        scroll_target = 0;
    }

    for (int i = 0; i < item_count; i++) {
        int y = menu_top + i * UI_LINE - scroll_offset;

        if (y < menu_top || y > ui_y(6))
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
// SETTINGS MENU
// ------------------------------------------------------------
void ui_draw_settingsmenu(int selection)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, ui_y(0), "Settings", 1);
    d.drawString(0, ui_y(1), "Select:", 1);

    const char *items[] = {
        "Profiles",
        "Calibration",
        "Factory reset",
        "Verify flash",
        "Back"
    };

    const int item_count = 5;
    const int menu_top = ui_y(2);

    for (int i = 0; i < item_count; i++) {
        int y = menu_top + i * UI_LINE;

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
// CONFIRM / MESSAGE
// ------------------------------------------------------------
void ui_draw_confirm(int selection,
                     const char *title,
                     const char *msg)
{
    auto &d = *g_disp;
    d.clear();

    d.drawString(0, ui_y(0), title, 1);
    if (msg && msg[0])
        d.drawString(0, ui_y(1), msg, 1);

    const char *items[] = { "Yes", "No" };
    const int menu_top = ui_y(4);

    for (int i = 0; i < 2; i++) {
        int y = menu_top + i * UI_LINE;

        if (i == selection) {
            gfx_fillRect(d, 0, y - 1, d.width(), 10, 1);
            drawStringInverted(d, 4, y, items[i]);
        } else {
            d.drawString(4, y, items[i], 1);
        }
    }

    d.drawString(0, ui_y(6), "Turn: select", 1);

    ui_draw_battery(battery_get_percent());
    d.show();
}

void ui_draw_message(const char *title,
                     const char *line1,
                     const char *line2)
{
    auto &d = *g_disp;
    d.clear();

    if (title && title[0])
        d.drawString(0, ui_y(0), title, 1);
    if (line1 && line1[0])
        d.drawString(0, ui_y(2), line1, 1);
    if (line2 && line2[0])
        d.drawString(0, ui_y(3), line2, 1);

    d.drawString(0, ui_y(6), "Press: back", 1);

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

void ui_draw_tdr_curve(const uint8_t *samples, int n)
{
	auto &d = *g_disp;

    d.clear();

    // Tegn baseline midt på skærmen
    int baseline = d.height() / 2;

    for (int x = 0; x < n && x < d.width(); x++) {
        int y = samples[x] ? baseline - 20 : baseline + 20;
        d.drawPixel(x, y, 1);
    }

    d.drawString(0, 0, "TDR Curve", 1);
    d.show();
}
