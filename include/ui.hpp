#pragma once
#include <cstdint>

struct TdrResult;
struct MicResult;

class GFX;

void ui_init(GFX &disp);

void ui_show_diag(GFX &d, const char *ctrl_name);

void ui_draw_tdr(const TdrResult &r,
                 const char *profile_name,
                 float vf);

void ui_draw_mic(const MicResult &m);

void ui_draw_menu(const char *profile_name,
                  float vf);

void ui_draw_calib(float ref_len_m,
                   bool done,
                   bool ok,
                   const char *status_text);

void ui_draw_startmenu(int selection);

void ui_show_progress(const char *msg, int steps);
