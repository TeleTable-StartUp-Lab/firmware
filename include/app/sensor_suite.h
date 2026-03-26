#pragma once

#include <Arduino.h>

#include "drivers/bh1750_sensor.h"
#include "drivers/mpu6050_sensor.h"
#include "drivers/obstacle_sensor.h"
#include "drivers/rfid_rc522_sensor.h"

class SensorSuite
{
public:
    SensorSuite();

    void beginIr();
    bool beginLux();
    bool beginImu();
    bool beginRfid();

    void update(uint32_t nowMs);

    void printIrOnce();
    void printLuxOnce();
    void printRfidOnce() const;

    void setIrWatch(bool on);
    void setIrPeriodic(bool on);
    void setLuxPeriodic(bool on);
    void setRfidWatch(bool on);

    bool frontObstacleNow() const;
    bool isIrLeftObstacle() const;
    bool isIrMidObstacle() const;
    bool isIrRightObstacle() const;

    bool hasLux() const;
    float lux() const;

    bool hasImu() const;
    uint8_t imuAddress() const;
    const Mpu6050Sensor::Reading &imu() const;

    bool hasRfid() const;
    uint8_t rfidVersion() const;
    const RfidRc522Sensor::Reading &rfid() const;

private:
    ObstacleSensor irLeft;
    ObstacleSensor irMid;
    ObstacleSensor irRight;

    Bh1750Sensor lightSensor;
    Mpu6050Sensor imuSensor;
    RfidRc522Sensor rfidSensor;

    bool irWatch;
    bool irPeriodic;
    bool luxPeriodic;
    bool rfidWatch;

    uint32_t lastIrPrintMs;
    uint32_t lastLuxPrintMs;
};
