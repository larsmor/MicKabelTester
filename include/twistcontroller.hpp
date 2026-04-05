#pragma once
#include "hardware/i2c.h"
#include <cstdint>

class TwistController {
public:
    TwistController(i2c_inst_t* i2c, uint8_t address);

    void    init();
    void    update();          // pt. tom, men beholdes til evt. udvidelse
    int16_t getDelta();
    void    setRGB(uint8_t r, uint8_t g, uint8_t b);
    bool    isPressed();       // læser knapstatus (aktiv lav på Qwiic Twist)
    void    setCount(int16_t value);

private:
    i2c_inst_t* _i2c;
    uint8_t     _addr;

    int16_t     _lastCount = 0;

    int16_t readEncoderCount();
    void    writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};
