#include "input.hpp"
#include "TwistController.hpp"

static TwistController *g_twist      = nullptr;
static int16_t          g_delta      = 0;
static bool             g_press      = false;
static bool             g_press_prev = false;
static bool             g_press_edge = false;

void input_init(TwistController &twist) {
    g_twist = &twist;
    g_twist->init();
    g_press_prev = g_twist->isPressed();
}

void input_update() {
    if (!g_twist) return;

    g_twist->update();              // pt. tom, men kald den alligevel
    g_delta = g_twist->getDelta();  // læser encoder og beregner delta

    g_press      = g_twist->isPressed();
    g_press_edge = (!g_press_prev && g_press);  // rising edge
    g_press_prev = g_press;
}

int16_t input_get_delta() {
    return g_delta;
}

bool input_pressed_edge() {
    return g_press_edge;
}

void input_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    if (!g_twist) return;
    g_twist->setRGB(r, g, b);
}
