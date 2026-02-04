#include "app/led_controller.h"

#include "board_pins.h"

LedController::LedController(SensorSuite &sensorsRef)
    : sensors(sensorsRef),
      ledStrip(LED_COUNT, static_cast<uint8_t>(BoardPins::LED_STRIP_DATA), NEO_GRB + NEO_KHZ800),
      ledEnabled(false),
      ledAutoEnabled(true),
      ledBrightness(40),
      ledR(255),
      ledG(255),
      ledB(255)
{
}

void LedController::begin()
{
    ledStrip.begin();
    ledStrip.clear();
    ledStrip.setBrightness(ledBrightness);
    ledStrip.show();
}

void LedController::autoTask()
{
    if (!ledAutoEnabled)
        return;
    if (!sensors.hasLux())
        return;

    const float lux = sensors.lux();

    if (!ledEnabled && lux < BoardPins::LED_LUX_ON_THRESHOLD)
    {
        setEnabled(true);
        Serial.printf("[led] auto on (lux=%.1f)\n", static_cast<double>(lux));
    }
    else if (ledEnabled && lux > BoardPins::LED_LUX_OFF_THRESHOLD)
    {
        setEnabled(false);
        Serial.printf("[led] auto off (lux=%.1f)\n", static_cast<double>(lux));
    }
}

void LedController::setAutoEnabled(bool enabled)
{
    ledAutoEnabled = enabled;
}

void LedController::setEnabled(bool enabled)
{
    if (ledEnabled == enabled)
        return;

    ledEnabled = enabled;
    apply();
    Serial.printf("[led] %s\n", ledEnabled ? "ON" : "OFF");
}

void LedController::setColor(uint8_t r, uint8_t g, uint8_t b)
{
    ledR = r;
    ledG = g;
    ledB = b;
}

void LedController::setBrightness(uint8_t v)
{
    ledBrightness = v;
}

bool LedController::isEnabled() const
{
    return ledEnabled;
}

bool LedController::isAutoEnabled() const
{
    return ledAutoEnabled;
}

uint8_t LedController::brightness() const
{
    return ledBrightness;
}

void LedController::apply()
{
    if (!ledEnabled)
    {
        ledStrip.clear();
        ledStrip.show();
        return;
    }

    const uint32_t c = ledStrip.Color(ledR, ledG, ledB);
    for (uint16_t i = 0; i < LED_COUNT; ++i)
    {
        ledStrip.setPixelColor(i, c);
    }

    ledStrip.setBrightness(ledBrightness);
    ledStrip.show();
}
