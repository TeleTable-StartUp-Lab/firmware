#pragma once

#include <Arduino.h>

#include "drivers/bh1750_sensor.h"
#include "drivers/obstacle_sensor.h"

class SensorSuite
{
public:
    SensorSuite();

    void beginIr();
    bool beginLux();

    void update(uint32_t nowMs);

    void printIrOnce();
    void printLuxOnce();

    void setIrWatch(bool on);
    void setIrPeriodic(bool on);
    void setLuxPeriodic(bool on);

    bool frontObstacleNow() const;
    bool isIrLeftObstacle() const;
    bool isIrMidObstacle() const;
    bool isIrRightObstacle() const;

    bool hasLux() const;
    float lux() const;

private:
    ObstacleSensor irLeft;
    ObstacleSensor irMid;
    ObstacleSensor irRight;

    Bh1750Sensor lightSensor;

    bool irWatch;
    bool irPeriodic;
    bool luxPeriodic;

    uint32_t lastIrPrintMs;
    uint32_t lastLuxPrintMs;
};
