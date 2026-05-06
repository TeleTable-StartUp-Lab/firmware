#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "app/sensor_suite.h"

class LedController
{
public:
    enum class LedMode : uint8_t
    {
        Static = 0,
        Breathing = 1,
        Loop = 2,
        Rainbow = 3,
        ColorWipe = 4,
        TheaterChase = 5,
        Scanner = 6,
        Sparkle = 7,
    };

    explicit LedController(SensorSuite &sensors);

    void begin();

    void autoTask();

    void setAutoEnabled(bool enabled);
    void setAutoLuxThreshold(float luxThreshold);
    void setEnabled(bool enabled);
    void setMode(LedMode mode);
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setBrightness(uint8_t v);
    void startArrivalCelebration();

    bool isEnabled() const;
    bool isAutoEnabled() const;
    float autoLuxThreshold() const;
    uint8_t brightness() const;

    void apply();

private:
    static constexpr uint16_t LED_COUNT = 144;

    struct LedSnapshot
    {
        bool enabled;
        bool autoEnabled;
        bool autoActivated;
        LedMode mode;
        uint8_t brightness;
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    static uint32_t wheel(Adafruit_NeoPixel &strip, uint8_t pos);
    void cancelCelebration(bool restorePreviousState);
    void finishArrivalCelebration();
    void renderArrivalCelebration(uint32_t nowMs);
    void renderStatic();
    void renderBreathing(uint32_t nowMs);
    void renderLoop(uint32_t nowMs);
    void renderRainbow(uint32_t nowMs);
    void renderColorWipe(uint32_t nowMs);
    void renderTheaterChase(uint32_t nowMs);
    void renderScanner(uint32_t nowMs);
    void renderSparkle(uint32_t nowMs);
    void render(uint32_t nowMs);

    SensorSuite &sensors;
    Adafruit_NeoPixel ledStrip;

    bool ledEnabled;
    bool ledAutoEnabled;
    bool ledAutoActivated;
    float ledAutoLuxThreshold;

    LedMode ledMode;
    uint32_t lastAnimMs;
    uint16_t loopIndex;
    uint8_t rainbowOffset;
    bool scannerForward;
    bool celebrationActive;
    uint32_t celebrationStartMs;
    LedSnapshot celebrationRestore;

    uint8_t ledBrightness;
    uint8_t ledR;
    uint8_t ledG;
    uint8_t ledB;
};
