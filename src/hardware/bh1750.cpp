#include "bh1750.h"
#include "pins.h"
#include "utils/logger.h"

bool BH1750Sensor::_isInitialized = false;

void BH1750Sensor::begin()
{
    // Do NOT call Wire.begin() here if it is already called in main.cpp or display.cpp
    // Just try to initialize the sensor on the existing bus

    // Some BH1750 modules need a small delay to wake up
    delay(10);

    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
    {
        Logger::info("LIGHT", "BH1750 initialized and ready");
        _isInitialized = true;
    }
    else
    {
        Logger::error("LIGHT", "BH1750 NOT found! Error 263 usually means wiring or pull-ups.");
        _isInitialized = false;
    }
}

float BH1750Sensor::readLight()
{
    if (!_isInitialized)
        return 0.0f;

    float level = lightMeter.readLightLevel();

    // If the library returns -1 or -2, it's a communication error
    if (level < 0)
    {
        return 0.0f;
    }
    return level;
}