#include "motion/StepperAxis.h"

#include <math.h>

#include "config/MotorConfig.h"
#include "drivers/AS5600Encoder.h"
#include "drivers/MultiTurnAngleTracker.h"
#include "drivers/TMC2209Driver.h"

namespace {
constexpr uint32_t AXIS_TMC_UART_BAUD = 115200;
constexpr int8_t HOMING_STALL_THRESHOLD = 8; // TODO: Tune this stallGuard threshold per axis.
constexpr float MIN_DT_SEC = 0.0001f;
constexpr float MAX_DT_SEC = 0.05f;
}

StepperAxis::StepperAxis(const AxisConfig& cfg,
                         TMC2209Driver& driver,
                         AS5600Encoder& encoder)
    : _cfg(cfg),
      _driver(driver),
      _encoder(encoder),
      _angleTracker(),
      _taskHandle(nullptr),
      _cmdQueue(nullptr),
      _state(AxisState::IDLE),
      _commandedAngleDeg(0.0f),
      _targetAngleDeg(0.0f),
      _currentSpeedDegPerSec(0.0f),
      _lastMeasuredAngleDeg(0.0f),
      _stepAccumulator(0.0f),
      _encoderSampleTimerSec(0.0f),
      _stepDirPositive(true) {}

void StepperAxis::begin() {
    pinMode(_cfg.stepPin, OUTPUT);
    pinMode(_cfg.dirPin, OUTPUT);
    pinMode(_cfg.enablePin, OUTPUT);

    digitalWrite(_cfg.stepPin, LOW);
    digitalWrite(_cfg.dirPin, LOW);

    const bool driverOk = _driver.begin(AXIS_TMC_UART_BAUD);
    if (!driverOk) {
        _state = AxisState::ERROR;
        return;
    }

    _driver.setCurrent(_cfg.runCurrent_mA, _cfg.holdCurrent_mA);
    _driver.setMicrostepping(_cfg.microsteps);
    _driver.enableStallGuard(HOMING_STALL_THRESHOLD);
    _driver.setStealthChop(true);

    if (!_encoder.begin()) {
        _state = AxisState::ERROR;
        return;
    }

    const float initialWrappedDeg = _encoder.readAngleCalibratedDeg();
    _angleTracker.reset(initialWrappedDeg);
    _lastMeasuredAngleDeg = _angleTracker.getAngle();
    _commandedAngleDeg = _lastMeasuredAngleDeg;
    _targetAngleDeg = _commandedAngleDeg;

    if (_cmdQueue == nullptr) {
        _cmdQueue = xQueueCreate(16, sizeof(AxisCommand));
        if (_cmdQueue == nullptr) {
            _state = AxisState::ERROR;
            return;
        }
    }

    _state = AxisState::IDLE;
}

void StepperAxis::startTask(const char* name,
                            uint32_t stackWords,
                            UBaseType_t priority,
                            BaseType_t coreID) {
    if (_taskHandle != nullptr) {
        return;
    }

    xTaskCreatePinnedToCore(
        taskTrampoline,
        name,
        stackWords,
        this,
        priority,
        &_taskHandle,
        coreID);
}

void StepperAxis::homeSensorless() {
    if (_cmdQueue == nullptr) {
        return;
    }
    const AxisCommand cmd{AxisCommandType::HOME, 0.0f};
    (void)xQueueSend(_cmdQueue, &cmd, 0);
}

void StepperAxis::moveToAngle(float targetDeg) {
    if (_cmdQueue == nullptr) {
        return;
    }
    const AxisCommand cmd{AxisCommandType::MOVE_TO, targetDeg};
    (void)xQueueSend(_cmdQueue, &cmd, 0);
}

void StepperAxis::moveByAngle(float deltaDeg) {
    if (_cmdQueue == nullptr) {
        return;
    }
    const AxisCommand cmd{AxisCommandType::MOVE_BY, deltaDeg};
    (void)xQueueSend(_cmdQueue, &cmd, 0);
}

void StepperAxis::stop() {
    if (_cmdQueue == nullptr) {
        return;
    }
    const AxisCommand cmd{AxisCommandType::STOP, 0.0f};
    (void)xQueueSend(_cmdQueue, &cmd, 0);
}

void StepperAxis::setMotorEnabled(bool enable) {
    _driver.enableDriver(enable);
}

bool StepperAxis::isMoving() const {
    return _state == AxisState::MOVING ||
           _state == AxisState::HOMING_START ||
           _state == AxisState::HOMING_RUNNING;
}

float StepperAxis::getMeasuredAngleDeg() const {
    return _lastMeasuredAngleDeg;
}

float StepperAxis::getCommandedAngleDeg() const {
    return _commandedAngleDeg;
}

uint16_t StepperAxis::getEncoderRawCounts() const {
    return const_cast<AS5600Encoder&>(_encoder).readRaw();
}

void StepperAxis::setCurrentAngleAsZero() {
    const float currentRawDeg = _encoder.readAngleDeg();
    _encoder.setZeroOffsetDeg(currentRawDeg);

    const float wrapped = _encoder.readAngleCalibratedDeg();
    _angleTracker.reset(wrapped);
    _lastMeasuredAngleDeg = _angleTracker.getAngle();
    _commandedAngleDeg = 0.0f;
    _targetAngleDeg = 0.0f;
}

AxisState StepperAxis::getState() const {
    return _state;
}

void StepperAxis::taskTrampoline(void* arg) {
    auto* axis = static_cast<StepperAxis*>(arg);
    axis->taskLoop();
}

void StepperAxis::taskLoop() {
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t lastMicros = micros();

    while (true) {
        const uint32_t nowMicros = micros();
        float dtSec = static_cast<float>(nowMicros - lastMicros) * 1.0e-6f;
        lastMicros = nowMicros;
        dtSec = constrain(dtSec, MIN_DT_SEC, MAX_DT_SEC);

        processCommands();
        handleStateMachineTick(dtSec);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
    }
}

void StepperAxis::processCommands() {
    if (_cmdQueue == nullptr) {
        return;
    }

    AxisCommand cmd{};
    while (xQueueReceive(_cmdQueue, &cmd, 0) == pdTRUE) {
        switch (cmd.type) {
            case AxisCommandType::HOME:
                _state = AxisState::HOMING_START;
                _currentSpeedDegPerSec = 0.0f;
                _stepAccumulator = 0.0f;
                break;

            case AxisCommandType::MOVE_TO:
                _targetAngleDeg = cmd.valueDeg;
                _driver.enableDriver(true);
                _state = AxisState::MOVING;
                break;

            case AxisCommandType::MOVE_BY:
                _targetAngleDeg = _commandedAngleDeg + cmd.valueDeg;
                _driver.enableDriver(true);
                _state = AxisState::MOVING;
                break;

            case AxisCommandType::STOP:
                _currentSpeedDegPerSec = 0.0f;
                _stepAccumulator = 0.0f;
                _targetAngleDeg = _commandedAngleDeg;
                _state = AxisState::IDLE;
                _driver.enableDriver(false);
                break;
        }
    }
}

void StepperAxis::handleStateMachineTick(float dtSec) {
    switch (_state) {
        case AxisState::IDLE:
            break;

        case AxisState::HOMING_START:
        case AxisState::HOMING_RUNNING:
            runHomingStateMachine(dtSec);
            break;

        case AxisState::MOVING:
            runMovingState(dtSec);
            break;

        case AxisState::ERROR:
            _currentSpeedDegPerSec = 0.0f;
            _stepAccumulator = 0.0f;
            _driver.enableDriver(false);
            break;
    }
}

void StepperAxis::runHomingStateMachine(float dtSec) {
    if (_state == AxisState::HOMING_START) {
        _driver.enableDriver(true);
        _driver.enableStallGuard(HOMING_STALL_THRESHOLD);

        const bool positiveDir = !_cfg.homeDirectionNegative;
        _currentSpeedDegPerSec = positiveDir ? _cfg.homingSpeedDegPerSec : -_cfg.homingSpeedDegPerSec;

        _stepAccumulator = 0.0f;
        _state = AxisState::HOMING_RUNNING;
        return;
    }

    if (_state != AxisState::HOMING_RUNNING) {
        return;
    }

    stepGeneratorUpdate(dtSec);

    if (_driver.isStall()) {
        _currentSpeedDegPerSec = 0.0f;
        _stepAccumulator = 0.0f;

        const float wrapped = _encoder.readAngleCalibratedDeg();
        _angleTracker.reset(wrapped);
        _lastMeasuredAngleDeg = _angleTracker.getAngle();

        _commandedAngleDeg = _cfg.homeAngleDeg;
        _targetAngleDeg = _cfg.homeAngleDeg;
        _state = AxisState::IDLE;

        // TODO: For best repeatability, use a backoff + second slow homing pass.
    }
}

void StepperAxis::runMovingState(float dtSec) {
    const float distance = _targetAngleDeg - _commandedAngleDeg;
    const float absDistance = fabsf(distance);
    const float direction = (distance >= 0.0f) ? 1.0f : -1.0f;

    const float speedAbs = fabsf(_currentSpeedDegPerSec);
    const float stoppingDistance = (speedAbs * speedAbs) / (2.0f * _cfg.maxAccelDegPerSec2 + 1e-6f);

    if (absDistance <= TARGET_REACHED_TOLERANCE_DEG && speedAbs < 1.0f) {
        _commandedAngleDeg = _targetAngleDeg;
        _currentSpeedDegPerSec = 0.0f;
        _stepAccumulator = 0.0f;
        _state = AxisState::IDLE;
        return;
    }

    if (absDistance <= stoppingDistance) {
        if (_currentSpeedDegPerSec > 0.0f) {
            _currentSpeedDegPerSec -= _cfg.maxAccelDegPerSec2 * dtSec;
            if (_currentSpeedDegPerSec < 0.0f) {
                _currentSpeedDegPerSec = 0.0f;
            }
        } else {
            _currentSpeedDegPerSec += _cfg.maxAccelDegPerSec2 * dtSec;
            if (_currentSpeedDegPerSec > 0.0f) {
                _currentSpeedDegPerSec = 0.0f;
            }
        }
    } else {
        _currentSpeedDegPerSec += direction * _cfg.maxAccelDegPerSec2 * dtSec;
        _currentSpeedDegPerSec = constrain(_currentSpeedDegPerSec,
                                           -_cfg.maxSpeedDegPerSec,
                                           _cfg.maxSpeedDegPerSec);
    }

    stepGeneratorUpdate(dtSec);

    _encoderSampleTimerSec += dtSec;
    if (_encoderSampleTimerSec >= ENCODER_SAMPLE_PERIOD_SEC) {
        _encoderSampleTimerSec = 0.0f;

        const float measured = _angleTracker.update(_encoder.readAngleCalibratedDeg());
        _lastMeasuredAngleDeg = measured;

        const float err = measured - _commandedAngleDeg;
        if (fabsf(err) > ENCODER_LARGE_ERROR_DEG) {
            _state = AxisState::ERROR;
            _currentSpeedDegPerSec = 0.0f;
            _stepAccumulator = 0.0f;
            return;
        }

        _commandedAngleDeg += ENCODER_SMALL_K_CORRECTION * err;
        // TODO: Consider closed-loop observer/filtering to avoid jitter from encoder noise.
    }
}

void StepperAxis::stepGeneratorUpdate(float dtSec) {
    const float stepsPerDeg = (_cfg.stepsPerRev * static_cast<float>(_cfg.microsteps) * _cfg.gearRatio) / 360.0f;
    if (stepsPerDeg <= 0.0f) {
        _state = AxisState::ERROR;
        return;
    }

    const float speedDegPerSec = _currentSpeedDegPerSec;
    if (fabsf(speedDegPerSec) < 1.0e-6f) {
        return;
    }

    const bool positive = speedDegPerSec >= 0.0f;
    if (_stepDirPositive != positive) {
        setDir(positive);
        _stepDirPositive = positive;
    }

    const float stepsThisTick = fabsf(speedDegPerSec) * stepsPerDeg * dtSec;
    _stepAccumulator += stepsThisTick;

    const uint32_t wholeSteps = static_cast<uint32_t>(_stepAccumulator);
    if (wholeSteps == 0) {
        return;
    }

    _stepAccumulator -= static_cast<float>(wholeSteps);
    const float degPerStep = 1.0f / stepsPerDeg;
    const float signedDegPerStep = positive ? degPerStep : -degPerStep;

    for (uint32_t i = 0; i < wholeSteps; ++i) {
        stepOnce();
        _commandedAngleDeg += signedDegPerStep;
    }
}

void StepperAxis::setDir(bool positive) {
    const bool pinLevel = positive ^ _cfg.invertDir;
    digitalWrite(_cfg.dirPin, pinLevel ? HIGH : LOW);
}

void StepperAxis::stepOnce() {
    digitalWrite(_cfg.stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(_cfg.stepPin, LOW);
}
