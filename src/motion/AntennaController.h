#pragma once

#include <Arduino.h>

#include "drivers/AS5600Encoder.h"
#include "drivers/TMC2209Driver.h"
#include "motion/StepperAxis.h"

class AntennaController {
public:
    AntennaController();

    void begin();
    void startTasks();

    void homeAll();
    void homeAzimuth();
    void homeElevation();

    void pointTo(float azDeg, float elDeg);
    void pointAzimuth(float azDeg);
    void pointElevation(float elDeg);

    void jogAzimuth(float deltaDeg);
    void jogElevation(float deltaDeg);

    void stopAll();
    void setMotorEnableAzimuth(bool enable);
    void setMotorEnableElevation(bool enable);
    void setMotorEnableBoth(bool enable);

    void setAzimuthZeroNow();
    void setElevationZeroNow();

    float getAzimuthMeasured() const;
    float getElevationMeasured() const;
    float getAzimuthCommanded() const;
    float getElevationCommanded() const;

    uint16_t getAzimuthEncoderCounts() const;
    uint16_t getElevationEncoderCounts() const;

    bool isAnyAxisMoving() const;
    AxisState getAzimuthState() const;
    AxisState getElevationState() const;

private:
    TMC2209Driver _azDriver;
    TMC2209Driver _elDriver;

    AS5600Encoder _azEncoder;
    AS5600Encoder _elEncoder;

    StepperAxis _azAxis;
    StepperAxis _elAxis;
};
