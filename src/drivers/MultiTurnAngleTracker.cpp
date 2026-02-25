#include "drivers/MultiTurnAngleTracker.h"

MultiTurnAngleTracker::MultiTurnAngleTracker() : _lastDeg(0.0f), _turns(0), _initialized(false) {}

void MultiTurnAngleTracker::reset(float initialDeg) {
    _lastDeg = initialDeg;
    _turns = 0;
    _initialized = true;
}

float MultiTurnAngleTracker::update(float newDeg) {
    if (!_initialized) {
        reset(newDeg);
        return getAngle();
    }

    const float delta = newDeg - _lastDeg;
    if (delta > 180.0f) {
        --_turns;
    } else if (delta < -180.0f) {
        ++_turns;
    }

    _lastDeg = newDeg;
    return getAngle();
}

float MultiTurnAngleTracker::getAngle() const {
    return static_cast<float>(_turns) * 360.0f + _lastDeg;
}
