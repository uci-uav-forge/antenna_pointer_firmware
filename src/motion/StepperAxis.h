#pragma once

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "drivers/MultiTurnAngleTracker.h"

struct AxisConfig;
class TMC2209Driver;
class AS5600Encoder;

enum class AxisCommandType { HOME, MOVE_TO, MOVE_BY, STOP };

struct AxisCommand {
    AxisCommandType type;
    float valueDeg;
};

enum class AxisState {
    IDLE,
    HOMING_START,
    HOMING_RUNNING,
    MOVING,
    ERROR
};

class StepperAxis {
public:
    StepperAxis(const AxisConfig& cfg,
                TMC2209Driver& driver,
                AS5600Encoder& encoder);

    void begin();

    void startTask(const char* name,
                   uint32_t stackWords,
                   UBaseType_t priority,
                   BaseType_t coreID);

    void homeSensorless();
    void moveToAngle(float targetDeg);
    void moveByAngle(float deltaDeg);
    void stop();
    void setMotorEnabled(bool enable);

    bool isMoving() const;

    float getMeasuredAngleDeg() const;
    float getCommandedAngleDeg() const;
    uint16_t getEncoderRawCounts() const;

    void setCurrentAngleAsZero();

    AxisState getState() const;

private:
    const AxisConfig& _cfg;
    TMC2209Driver& _driver;
    AS5600Encoder& _encoder;
    MultiTurnAngleTracker _angleTracker;

    TaskHandle_t _taskHandle;
    QueueHandle_t _cmdQueue;

    AxisState _state;

    float _commandedAngleDeg;
    float _targetAngleDeg;
    float _currentSpeedDegPerSec;

    float _lastMeasuredAngleDeg;

    float _stepAccumulator;
    float _encoderSampleTimerSec;
    bool _stepDirPositive;

    static void taskTrampoline(void* arg);
    void taskLoop();

    void processCommands();
    void handleStateMachineTick(float dtSec);
    void runHomingStateMachine(float dtSec);
    void runMovingState(float dtSec);

    void stepGeneratorUpdate(float dtSec);

    void setDir(bool positive);
    void stepOnce();
};
