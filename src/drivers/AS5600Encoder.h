#pragma once

#include <Arduino.h>
#include <Wire.h>

class AS5600Encoder {
public:
    AS5600Encoder(TwoWire& wire, uint8_t i2cAddress);

    bool begin();
    uint16_t readRaw();
    float readAngleDeg();
    float readAngleRad();

    void setZeroOffsetDeg(float offset);
    float getZeroOffsetDeg() const;
    float readAngleCalibratedDeg();

private:
    TwoWire& _wire;
    uint8_t _addr;
    float _zeroOffsetDeg;

    uint16_t readRegister16(uint8_t regHigh, uint8_t regLow);
};
