#pragma once

#include <Arduino.h>

class MultiTurnAngleTracker {
public:
    MultiTurnAngleTracker();

    void reset(float initialDeg);
    float update(float newDeg);
    float getAngle() const;

private:
    float _lastDeg;
    int32_t _turns;
    bool _initialized;
};
