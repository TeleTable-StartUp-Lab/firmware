#ifndef LED_HANDLER_H
#define LED_HANDLER_H

#include <FastLED.h>
#include "pins.h"

class LEDController
{
public:
    void begin();
    void setAllColors(CRGB color);
    void clear();

private:
    CRGB leds[NUM_LEDS];
};

#endif