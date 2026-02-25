#include "drivers/AS5600Encoder.h"

#include <math.h>

namespace {
constexpr uint8_t AS5600_REG_ANGLE_HIGH = 0x0E;
constexpr uint8_t AS5600_REG_ANGLE_LOW = 0x0F;
constexpr float DEG_PER_LSB = 360.0f / 4096.0f;
}

AS5600Encoder::AS5600Encoder(TwoWire& wire, uint8_t i2cAddress)
    : _wire(wire), _addr(i2cAddress), _zeroOffsetDeg(0.0f) {}

bool AS5600Encoder::begin() {
    _wire.beginTransmission(_addr);
    if (_wire.endTransmission() != 0) {
        return false;
    }
    (void)readRaw();
    return true;
}

uint16_t AS5600Encoder::readRaw() {
    return static_cast<uint16_t>(readRegister16(AS5600_REG_ANGLE_HIGH, AS5600_REG_ANGLE_LOW) & 0x0FFFu);
}

float AS5600Encoder::readAngleDeg() {
    return static_cast<float>(readRaw()) * DEG_PER_LSB;
}

float AS5600Encoder::readAngleRad() {
    return readAngleDeg() * (PI / 180.0f);
}

void AS5600Encoder::setZeroOffsetDeg(float offset) {
    _zeroOffsetDeg = offset;
}

float AS5600Encoder::getZeroOffsetDeg() const {
    return _zeroOffsetDeg;
}

float AS5600Encoder::readAngleCalibratedDeg() {
    float adjusted = readAngleDeg() - _zeroOffsetDeg;
    while (adjusted < 0.0f) {
        adjusted += 360.0f;
    }
    while (adjusted >= 360.0f) {
        adjusted -= 360.0f;
    }
    return adjusted;
}

uint16_t AS5600Encoder::readRegister16(uint8_t regHigh, uint8_t regLow) {
    _wire.beginTransmission(_addr);
    _wire.write(regHigh);
    if (_wire.endTransmission(false) != 0) {
        return 0;
    }

    const uint8_t bytesRead = _wire.requestFrom(static_cast<int>(_addr), 2);
    if (bytesRead < 2) {
        return 0;
    }

    const uint8_t highByte = _wire.read();
    const uint8_t lowByte = _wire.read();
    (void)regLow;
    return static_cast<uint16_t>((static_cast<uint16_t>(highByte) << 8u) | lowByte);
}
