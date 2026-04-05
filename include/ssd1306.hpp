#pragma once
#include "gfx.hpp"
#include "hardware/i2c.h"
#include <cstring>

class SSD1306 : public GFX {
public:
    SSD1306(i2c_inst_t *i2c, uint8_t addr, int16_t w = 128, int16_t h = 64)
        : GFX(w, h), _i2c(i2c), _addr(addr)
    {
        _pages    = _height / 8;
        _buf_size = _width * _pages;
    }

    bool probe() {
        uint8_t cmd[2] = {0x00, 0xAE}; // DISPLAY OFF
        int res = i2c_write_blocking(_i2c, _addr, cmd, 2, false);
        return (res == 2);
    }

    void begin() override {
        static const uint8_t init_seq[] = {
            0x00,
            0xAE,
            0x20, 0x00,
            0xB0,
            0xC8,
            0x00,
            0x10,
            0x40,
            0x81, 0x7F,
            0xA1,
            0xA6,
            0xA8, 0x3F,
            0xD3, 0x00,
            0xD5, 0x80,
            0xD9, 0xF1,
            0xDA, 0x12,
            0xDB, 0x40,
            0x8D, 0x14,
            0xAF
        };
        i2c_write_blocking(_i2c, _addr, init_seq, sizeof(init_seq), false);
        clear();
        show();
    }

    void clear() override {
        std::memset(_buf, 0, sizeof(_buf));
    }

    void drawPixel(int16_t x, int16_t y, uint16_t color) override {
        if (x < 0 || x >= _width || y < 0 || y >= _height) return;
        uint16_t idx = x + (y / 8) * _width;
        uint8_t bit = 1u << (y & 7);
        if (color)
            _buf[idx] |= bit;
        else
            _buf[idx] &= ~bit;
    }

    void show() override {
        for (int page = 0; page < _pages; ++page) {
            uint8_t cmd[] = {
                0x00,
                (uint8_t)(0xB0 | page),
                0x00,
                0x10
            };
            i2c_write_blocking(_i2c, _addr, cmd, sizeof(cmd), false);

            uint8_t data[1 + 128];
            data[0] = 0x40;
            std::memcpy(&data[1], &_buf[page * _width], _width);
            i2c_write_blocking(_i2c, _addr, data, _width + 1, false);
        }
    }

private:
    i2c_inst_t *_i2c;
    uint8_t _addr;
    int     _pages;
    int     _buf_size;
    uint8_t _buf[128 * 8]; // 128x64
};
