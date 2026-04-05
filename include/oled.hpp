#pragma once
#include "gfx.hpp"
#include "ssd1306.hpp"
#include "sh1106.hpp"

enum class OledType {
    Unknown,
    SSD1306,
    SH1106
};

class OLED : public GFX {
public:
    OLED(i2c_inst_t *i2c, uint8_t addr, int16_t w = 128, int16_t h = 64)
        : GFX(w, h),
          _ssd(i2c, addr, w, h),
          _sh(i2c, addr, w, h),
          _impl(nullptr),
          _type(OledType::Unknown)
    {}

    void begin() override {
        if (_ssd.probe()) {
            _impl = &_ssd;
            _type = OledType::SSD1306;
            _impl->begin();
        } else if (_sh.probe()) {
            _impl = &_sh;
            _type = OledType::SH1106;
            _impl->begin();
        } else {
            _impl = nullptr;
            _type = OledType::Unknown;
        }
    }

    void clear() override {
        if (_impl) _impl->clear();
    }

    void drawPixel(int16_t x, int16_t y, uint16_t color) override {
        if (_impl) _impl->drawPixel(x, y, color);
    }

    void show() override {
        if (_impl) _impl->show();
    }

    OledType type() const { return _type; }

    const char* controller_name() const {
        switch (_type) {
        case OledType::SSD1306: return "SSD1306";
        case OledType::SH1106:  return "SH1106";
        default:                return "UNKNOWN";
        }
    }

private:
    SSD1306  _ssd;
    SH1106   _sh;
    GFX     *_impl;
    OledType _type;
};
