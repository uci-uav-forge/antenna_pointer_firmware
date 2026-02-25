#pragma once

#include <Arduino.h>

#include "config/Pins.h"

struct AxisConfig {
    const char* name;
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
    uint8_t diagPin;

    // TMC configuration
    uint8_t tmcAddress;
    float runCurrent_mA;
    float holdCurrent_mA;
    uint8_t microsteps;

    // Kinematics
    float stepsPerRev;
    float gearRatio;
    bool invertDir;

    // Motion limits
    float maxSpeedDegPerSec;
    float maxAccelDegPerSec2;

    // Homing
    float homingSpeedDegPerSec;
    bool homeDirectionNegative;
    float homeAngleDeg;
};

// Encoder / fusion tuning
constexpr float ENCODER_LARGE_ERROR_DEG = 2.0f;
constexpr float ENCODER_SMALL_K_CORRECTION = 0.10f;
constexpr float TARGET_REACHED_TOLERANCE_DEG = 0.15f;
constexpr float ENCODER_SAMPLE_PERIOD_SEC = 0.01f;

// TODO: Tune current, speed, accel, homing direction, and mechanical ratio for the real hardware.
inline constexpr AxisConfig AZIMUTH_AXIS_CONFIG{
    "Azimuth",
    AZ_STEP_PIN,
    AZ_DIR_PIN,
    AZ_EN_PIN,
    AZ_DIAG_PIN,
    0,
    900.0f,
    350.0f,
    16,
    200.0f,
    5.0f,
    false,
    60.0f,
    180.0f,
    8.0f,
    true,
    0.0f,
};

inline constexpr AxisConfig ELEVATION_AXIS_CONFIG{
    "Elevation",
    EL_STEP_PIN,
    EL_DIR_PIN,
    EL_EN_PIN,
    EL_DIAG_PIN,
    1,
    800.0f,
    300.0f,
    16,
    200.0f,
    5.0f,
    false,
    45.0f,
    150.0f,
    6.0f,
    true,
    0.0f,
};
