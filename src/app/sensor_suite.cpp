#include "app/sensor_suite.h"

#include "board_pins.h"

#include <Wire.h>

SensorSuite::SensorSuite()
    : irLeft({.pin = BoardPins::IR_LEFT, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false}),
      irMid({.pin = BoardPins::IR_MIDDLE, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false}),
      irRight({.pin = BoardPins::IR_RIGHT, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false}),
      lightSensor({.wire = &Wire, .address = 0x23, .sample_period_ms = 250}),
      irWatch(true),
      irPeriodic(false),
      luxPeriodic(true),
      lastIrPrintMs(0),
      lastLuxPrintMs(0)
{
}

void SensorSuite::beginIr()
{
    irLeft.begin();
    irMid.begin();
    irRight.begin();
}

bool SensorSuite::beginLux()
{
    return lightSensor.begin();
}

void SensorSuite::update(uint32_t nowMs)
{
    irLeft.update(nowMs);
    irMid.update(nowMs);
    irRight.update(nowMs);

    if (irWatch)
    {
        if (irLeft.roseObstacle())
            Serial.println("[ir] left obstacle");
        if (irLeft.fellObstacle())
            Serial.println("[ir] left clear");

        if (irMid.roseObstacle())
            Serial.println("[ir] middle obstacle");
        if (irMid.fellObstacle())
            Serial.println("[ir] middle clear");

        if (irRight.roseObstacle())
            Serial.println("[ir] right obstacle");
        if (irRight.fellObstacle())
            Serial.println("[ir] right clear");
    }

    if (irPeriodic && (nowMs - lastIrPrintMs >= 500))
    {
        lastIrPrintMs = nowMs;
        printIrOnce();
    }

    lightSensor.update(nowMs);

    if (luxPeriodic && (nowMs - lastLuxPrintMs >= 500))
    {
        lastLuxPrintMs = nowMs;
        printLuxOnce();
    }
}

void SensorSuite::printIrOnce()
{
    Serial.printf("[ir] L=%d M=%d R=%d (1=obstacle)\n",
                  irLeft.isObstacle() ? 1 : 0,
                  irMid.isObstacle() ? 1 : 0,
                  irRight.isObstacle() ? 1 : 0);
}

void SensorSuite::printLuxOnce()
{
    if (!lightSensor.hasReading())
    {
        Serial.println("[lux] no reading");
        return;
    }
    Serial.printf("[lux] %.1f lx\n", static_cast<double>(lightSensor.lux()));
}

void SensorSuite::setIrWatch(bool on)
{
    irWatch = on;
}

void SensorSuite::setIrPeriodic(bool on)
{
    irPeriodic = on;
}

void SensorSuite::setLuxPeriodic(bool on)
{
    luxPeriodic = on;
}

bool SensorSuite::frontObstacleNow() const
{
    return irLeft.isObstacle() || irMid.isObstacle() || irRight.isObstacle();
}

bool SensorSuite::isIrLeftObstacle() const
{
    return irLeft.isObstacle();
}

bool SensorSuite::isIrMidObstacle() const
{
    return irMid.isObstacle();
}

bool SensorSuite::isIrRightObstacle() const
{
    return irRight.isObstacle();
}

bool SensorSuite::hasLux() const
{
    return lightSensor.hasReading();
}

float SensorSuite::lux() const
{
    return lightSensor.lux();
}
