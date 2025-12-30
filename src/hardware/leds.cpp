#include "leds.h"

void LEDController::begin()
{
    // IMPORTANT: Limits the current to 500mA for USB operation
    FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);

    // Strip initialization
    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);

    FastLED.setBrightness(128);

    clear();
}

void LEDController::setAllColors(CRGB color)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = color;
    }
    FastLED.show();
}

void LEDController::clear()
{
    FastLED.clear();
    FastLED.show();
}