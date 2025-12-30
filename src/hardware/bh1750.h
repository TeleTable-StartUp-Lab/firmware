#ifndef BH1750_HANDLER_H
#define BH1750_HANDLER_H

#include <BH1750.h>
#include <Wire.h>

class BH1750Sensor
{
public:
    void begin();
    float readLight();

private:
    BH1750 lightMeter;
    static bool _isInitialized;
};

#endif