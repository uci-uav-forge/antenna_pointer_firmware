#include "drivers/TMC2209Driver.h"

#include "config/Pins.h"

namespace {
constexpr float TMC_R_SENSE_OHMS = 0.11f; // TODO: Match the exact Rsense resistor on your driver boards.
}

TMC2209Driver::TMC2209Driver(HardwareSerial& serial, uint8_t address, uint8_t enPin, uint8_t diagPin)
    : _serial(serial), _address(address), _enPin(enPin), _diagPin(diagPin), _tmc(nullptr) {}

bool TMC2209Driver::begin(uint32_t uartBaud) {
    pinMode(_enPin, OUTPUT);
    pinMode(_diagPin, INPUT_PULLUP);
    enableDriver(false);

    _serial.begin(uartBaud, SERIAL_8N1, TMC_UART_RX_PIN, TMC_UART_TX_PIN);

    if (_tmc == nullptr) {
        _tmc = new TMC2209Stepper(&_serial, TMC_R_SENSE_OHMS, _address);
    }
    if (_tmc == nullptr) {
        return false;
    }

    _tmc->begin();
    _tmc->pdn_disable(true);
    _tmc->mstep_reg_select(true);
    _tmc->I_scale_analog(false);
    _tmc->toff(4);
    _tmc->blank_time(24);
    _tmc->en_spreadCycle(false);
    _tmc->pwm_autoscale(true);
    // TODO: Tune chopper and PWM settings for your mechanics and voltage.

    enableDriver(true);
    return true;
}

void TMC2209Driver::enableDriver(bool enable) {
    // Typical EN pin is active-low on TMC2209 breakout boards.
    digitalWrite(_enPin, enable ? LOW : HIGH);
}

void TMC2209Driver::setCurrent(float runCurrent_mA, float holdCurrent_mA) {
    if (_tmc == nullptr) {
        return;
    }

    const uint16_t runCurrent = static_cast<uint16_t>(constrain(runCurrent_mA, 100.0f, 2500.0f));
    _tmc->rms_current(runCurrent);

    float holdRatio = 0.5f;
    if (runCurrent_mA > 1.0f) {
        holdRatio = holdCurrent_mA / runCurrent_mA;
    }
    const uint8_t ihold = static_cast<uint8_t>(constrain(holdRatio * 31.0f, 0.0f, 31.0f));
    _tmc->ihold(ihold);
}

void TMC2209Driver::setMicrostepping(uint8_t microsteps) {
    if (_tmc == nullptr) {
        return;
    }
    _tmc->microsteps(microsteps);
}

void TMC2209Driver::enableStallGuard(int8_t threshold) {
    if (_tmc == nullptr) {
        return;
    }

    const int16_t mapped = static_cast<int16_t>(threshold) + 64;
    const uint8_t sgthrs = static_cast<uint8_t>(constrain(mapped, 0, 127));
    _tmc->SGTHRS(sgthrs);
    _tmc->TCOOLTHRS(0xFFFFF);
    _tmc->semin(5);
    _tmc->semax(2);
    // TODO: Verify SGTHRS mapping and tune for your load and speed.
}

bool TMC2209Driver::isStall() const {
    // TODO: Some boards invert DIAG polarity. Flip this condition if needed.
    return digitalRead(_diagPin) == HIGH;
}

void TMC2209Driver::setSpreadCycle(bool enable) {
    if (_tmc == nullptr) {
        return;
    }
    _tmc->en_spreadCycle(enable);
}

void TMC2209Driver::setStealthChop(bool enable) {
    setSpreadCycle(!enable);
}

uint32_t TMC2209Driver::readStatus() {
    if (_tmc == nullptr) {
        return 0;
    }
    return _tmc->DRV_STATUS();
}
