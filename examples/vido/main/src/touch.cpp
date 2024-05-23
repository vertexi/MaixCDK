#include "maix_touchscreen.hpp"

using namespace maix;

touchscreen::TouchScreen *touchscreen_p;

extern "C" void touch_init();
extern "C" void touch_deinit();
extern "C" void touch_read(int *x, int *y, int *state);

void touch_init()
{
    // touch screen
    touchscreen_p = new touchscreen::TouchScreen();
}

void touch_deinit()
{
    delete touchscreen_p;
}

void touch_read(int *x, int *y, int *state)
{
    auto res = touchscreen_p->read();
    *x = res.at(0);
    *y = res.at(1);
    *state = res.at(2);
}
