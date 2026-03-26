#pragma once

#include <Arduino.h>

#include "drivers/bh1750_sensor.h"
#include "drivers/mpu6050_sensor.h"
#include "drivers/obstacle_sensor.h"

class SensorSuite
{
public:
    SensorSuite();

    void beginIr();
    bool beginLux();
    bool beginImu();

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

    bool hasImu() const;
    uint8_t imuAddress() const;
    const Mpu6050Sensor::Reading &imu() const;

private:
    ObstacleSensor irLeft;
    ObstacleSensor irMid;
    ObstacleSensor irRight;

    Bh1750Sensor lightSensor;
    Mpu6050Sensor imuSensor;

    bool irWatch;
    bool irPeriodic;
    bool luxPeriodic;

    uint32_t lastIrPrintMs;
    uint32_t lastLuxPrintMs;
};
