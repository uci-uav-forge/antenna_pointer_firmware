# Antenna Pointer Quick Guide

`AntennaController` is the main API. Calls are non-blocking: commands are queued and executed by axis tasks in the background.

## Startup

```cpp
#include <Arduino.h>
#include "motion/AntennaController.h"

AntennaController antenna;

void setup() {
    Serial.begin(115200);
    delay(1000);

    antenna.begin();
    antenna.startTasks();
    antenna.homeAll();
}
```

## Main commands

```cpp
antenna.pointTo(45.0f, 10.0f);
antenna.pointAzimuth(-30.0f);
antenna.pointElevation(15.0f);

antenna.homeAll();
antenna.homeAzimuth();
antenna.homeElevation();

float az = antenna.getAzimuthMeasured();
float el = antenna.getElevationMeasured();
```

## What to configure

- `src/config/Pins.h`: pins, UART, I2C wiring
- `src/config/MotorConfig.h`: motor, limits, homing, encoder thresholds


