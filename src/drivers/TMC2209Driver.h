#pragma once

#include <Arduino.h>

#include <TMCStepper.h>

class TMC2209Driver {
public:
    TMC2209Driver(HardwareSerial& serial, uint8_t address, uint8_t enPin, uint8_t diagPin);

    bool begin(uint32_t uartBaud);
    void enableDriver(bool enable);

    void setCurrent(float runCurrent_mA, float holdCurrent_mA);
    void setMicrostepping(uint8_t microsteps);

    void enableStallGuard(int8_t threshold);
    bool isStall() const;

    void setSpreadCycle(bool enable);
    void setStealthChop(bool enable);

    uint32_t readStatus();

private:
    HardwareSerial& _serial;
    uint8_t _address;
    uint8_t _enPin;
    uint8_t _diagPin;

    TMC2209Stepper* _tmc;
};
