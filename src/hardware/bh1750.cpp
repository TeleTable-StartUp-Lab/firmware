#include "bh1750.h"
#include "pins.h"
#include "utils/logger.h"
void BH1750Sensor::begin()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
    {
        Logger::info("LIGHT", "BH1750 initialized");
    }
    else
    {
        Logger::error("LIGHT", "Error starting the BH1750!");
    }
}

float BH1750Sensor::readLight()
{
    return lightMeter.readLightLevel();
}