#pragma once
#include "gfx.hpp"
#include "tdr.hpp"
#include "mic_test.hpp"

void ui_init(GFX &disp);
void ui_show_diag(GFX &d, const char *ctrl_name);

void ui_draw_tdr(const TdrResult &r, const char *profile_name, float vf);
void ui_draw_mic(const MicResult &m);
void ui_draw_menu(const char *profile_name, float vf);
void ui_draw_calib(float ref_len_m, bool done, bool ok);
void ui_draw_startmenu(int selection);
void drawStringInverted(GFX &d, int x, int y, const char *s);