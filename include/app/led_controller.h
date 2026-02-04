#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "app/sensor_suite.h"

class LedController
{
public:
    explicit LedController(SensorSuite &sensors);

    void begin();

    void autoTask();

    void setAutoEnabled(bool enabled);
    void setEnabled(bool enabled);
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setBrightness(uint8_t v);

    bool isEnabled() const;
    bool isAutoEnabled() const;
    uint8_t brightness() const;

    void apply();

private:
    static constexpr uint16_t LED_COUNT = 144;

    SensorSuite &sensors;
    Adafruit_NeoPixel ledStrip;

    bool ledEnabled;
    bool ledAutoEnabled;

    uint8_t ledBrightness;
    uint8_t ledR;
    uint8_t ledG;
    uint8_t ledB;
};
