#include "TwistController.hpp"
#include <cstdio>

#define REG_STATUS      0x00
#define REG_BUTTON      0x01
#define REG_COUNT_L     0x05
#define REG_COUNT_M     0x06
#define REG_RED         0x0D
#define REG_GREEN       0x0E
#define REG_BLUE        0x0F 

const uint8_t statusButtonClickedBit = 2;
const uint8_t statusButtonPressedBit = 1;
const uint8_t statusEncoderMovedBit = 0;

TwistController::TwistController(i2c_inst_t* i2c, uint8_t address)
    : _i2c(i2c), _addr(address) {}

void TwistController::init() {
    _lastCount = readEncoderCount();
}

void TwistController::update() {
    // Kan udvides senere (f.eks. cache af status)
}

int16_t TwistController::getDelta() {
    int16_t now   = readEncoderCount();
    int16_t delta = now - _lastCount;
    _lastCount    = now;
    return delta;
}

void TwistController::setRGB(uint8_t r, uint8_t g, uint8_t b) {
    writeRegister(REG_RED,   r);
    writeRegister(REG_GREEN, g);
    writeRegister(REG_BLUE,  b);
}

bool TwistController::isPressed() {
    uint8_t v = readRegister(REG_BUTTON);
    // 1 = ikke trykket, 0 = trykket (aktiv lav)
    return (v & (1 << statusButtonPressedBit)) == 0;
}

void TwistController::setCount(int16_t value) {
    uint8_t lo = value & 0xFF;
    uint8_t hi = (value >> 8) & 0xFF;

    writeRegister(REG_COUNT_L, lo);
    writeRegister(REG_COUNT_M, hi);

    _lastCount = value;
}

int16_t TwistController::readEncoderCount() {
    uint8_t lo = readRegister(REG_COUNT_L);
    uint8_t hi = readRegister(REG_COUNT_M);
    return (int16_t)((hi << 8) | lo);
}

void TwistController::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    i2c_write_blocking(_i2c, _addr, buf, 2, false);
}

uint8_t TwistController::readRegister(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(_i2c, _addr, &reg, 1, true);
    i2c_read_blocking(_i2c, _addr, buf, 2, false);
    return buf[0];
}
