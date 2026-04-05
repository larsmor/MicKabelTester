#pragma once
#include <cstdint>

class TwistController;

void     input_init(TwistController &twist);
void     input_update();
int16_t  input_get_delta();
bool     input_pressed_edge();
void     input_set_rgb(uint8_t r, uint8_t g, uint8_t b);
