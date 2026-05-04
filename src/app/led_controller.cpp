#include "app/led_controller.h"

#include "board_pins.h"

LedController::LedController(SensorSuite &sensorsRef)
    : sensors(sensorsRef),
      ledStrip(LED_COUNT, static_cast<uint8_t>(BoardPins::LED_STRIP_DATA), NEO_GRB + NEO_KHZ800),
      ledEnabled(false),
      ledAutoEnabled(true),
      ledMode(LedMode::Static),
      lastAnimMs(0),
      loopIndex(0),
      rainbowOffset(0),
      celebrationActive(false),
      celebrationStartMs(0),
      celebrationRestore{false, true, LedMode::Static, 40, 255, 255, 255},
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
    const uint32_t nowMs = millis();

    if (celebrationActive)
    {
        renderArrivalCelebration(nowMs);
        return;
    }

    if (ledEnabled && ledMode != LedMode::Static)
    {
        // Keep animations running regardless of auto mode.
        render(nowMs);
    }

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
    if (celebrationActive)
        cancelCelebration(false);
    ledAutoEnabled = enabled;
}

void LedController::setEnabled(bool enabled)
{
    if (celebrationActive)
        cancelCelebration(false);
    if (ledEnabled == enabled)
        return;

    ledEnabled = enabled;
    apply();
    Serial.printf("[led] %s\n", ledEnabled ? "ON" : "OFF");
}

void LedController::setMode(LedMode mode)
{
    if (celebrationActive)
        cancelCelebration(false);
    if (ledMode == mode)
        return;

    ledMode = mode;
    lastAnimMs = 0;
    loopIndex = 0;
    rainbowOffset = 0;
    apply();
}

void LedController::setColor(uint8_t r, uint8_t g, uint8_t b)
{
    if (celebrationActive)
        cancelCelebration(false);
    ledR = r;
    ledG = g;
    ledB = b;
}

void LedController::setBrightness(uint8_t v)
{
    if (celebrationActive)
        cancelCelebration(false);
    ledBrightness = v;
}

void LedController::startArrivalCelebration()
{
    celebrationRestore = {ledEnabled, ledAutoEnabled, ledMode, ledBrightness, ledR,
                          ledG, ledB};
    celebrationActive = true;
    celebrationStartMs = millis();
    lastAnimMs = 0;

    ledEnabled = true;
    ledAutoEnabled = false;
    ledMode = LedMode::Static;
    renderArrivalCelebration(celebrationStartMs);
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
    render(millis());
}

void LedController::cancelCelebration(bool restorePreviousState)
{
    if (!celebrationActive)
        return;

    celebrationActive = false;
    if (!restorePreviousState)
        return;

    ledEnabled = celebrationRestore.enabled;
    ledAutoEnabled = celebrationRestore.autoEnabled;
    ledMode = celebrationRestore.mode;
    ledBrightness = celebrationRestore.brightness;
    ledR = celebrationRestore.r;
    ledG = celebrationRestore.g;
    ledB = celebrationRestore.b;
    lastAnimMs = 0;
    loopIndex = 0;
    rainbowOffset = 0;
    apply();
}

void LedController::finishArrivalCelebration()
{
    cancelCelebration(true);
}

void LedController::renderArrivalCelebration(uint32_t nowMs)
{
    constexpr uint32_t DURATION_MS = 1600;
    constexpr uint32_t FRAME_MS = 160;
    constexpr uint8_t CELEBRATION_BRIGHTNESS = 96;

    if ((nowMs - celebrationStartMs) >= DURATION_MS)
    {
        finishArrivalCelebration();
        return;
    }

    if (lastAnimMs != 0 && (nowMs - lastAnimMs) < FRAME_MS)
        return;
    lastAnimMs = nowMs;

    const bool bluePhase = (((nowMs - celebrationStartMs) / FRAME_MS) % 2U) == 0U;
    const uint32_t color = bluePhase ? ledStrip.Color(0, 80, 255)
                                     : ledStrip.Color(0, 255, 90);

    for (uint16_t i = 0; i < LED_COUNT; ++i)
        ledStrip.setPixelColor(i, color);

    ledStrip.setBrightness(CELEBRATION_BRIGHTNESS);
    ledStrip.show();
}

uint32_t LedController::wheel(Adafruit_NeoPixel &strip, uint8_t pos)
{
    // Classic RGB color wheel (0..255)
    pos = 255 - pos;
    if (pos < 85)
        return strip.Color(255 - pos * 3, 0, pos * 3);
    if (pos < 170)
    {
        pos -= 85;
        return strip.Color(0, pos * 3, 255 - pos * 3);
    }
    pos -= 170;
    return strip.Color(pos * 3, 255 - pos * 3, 0);
}

void LedController::renderStatic()
{
    const uint32_t c = ledStrip.Color(ledR, ledG, ledB);
    for (uint16_t i = 0; i < LED_COUNT; ++i)
    {
        ledStrip.setPixelColor(i, c);
    }

    ledStrip.setBrightness(ledBrightness);
    ledStrip.show();
}

void LedController::renderBreathing(uint32_t nowMs)
{
    constexpr uint32_t FRAME_MS = 30;
    if (lastAnimMs != 0 && (nowMs - lastAnimMs) < FRAME_MS)
        return;
    lastAnimMs = nowMs;

    constexpr uint32_t PERIOD_MS = 2000;
    const uint32_t phase = nowMs % PERIOD_MS;
    const uint32_t half = PERIOD_MS / 2;
    const uint32_t ramp = (phase < half) ? phase : (PERIOD_MS - phase);
    const uint8_t scaledBrightness = static_cast<uint8_t>((ramp * static_cast<uint32_t>(ledBrightness)) / half);

    const uint32_t c = ledStrip.Color(ledR, ledG, ledB);
    for (uint16_t i = 0; i < LED_COUNT; ++i)
    {
        ledStrip.setPixelColor(i, c);
    }

    ledStrip.setBrightness(scaledBrightness);
    ledStrip.show();
}

void LedController::renderLoop(uint32_t nowMs)
{
    constexpr uint32_t FRAME_MS = 30;
    if (lastAnimMs != 0 && (nowMs - lastAnimMs) < FRAME_MS)
        return;
    lastAnimMs = nowMs;

    loopIndex = static_cast<uint16_t>((loopIndex + 1) % LED_COUNT);

    ledStrip.clear();

    const uint32_t c = ledStrip.Color(ledR, ledG, ledB);
    ledStrip.setPixelColor(loopIndex, c);

    ledStrip.setBrightness(ledBrightness);
    ledStrip.show();
}

void LedController::renderRainbow(uint32_t nowMs)
{
    constexpr uint32_t FRAME_MS = 30;
    if (lastAnimMs != 0 && (nowMs - lastAnimMs) < FRAME_MS)
        return;
    lastAnimMs = nowMs;

    rainbowOffset = static_cast<uint8_t>(rainbowOffset + 2);

    for (uint16_t i = 0; i < LED_COUNT; ++i)
    {
        const uint8_t pos = static_cast<uint8_t>((static_cast<uint32_t>(i) * 256U / LED_COUNT) + rainbowOffset);
        ledStrip.setPixelColor(i, wheel(ledStrip, pos));
    }

    ledStrip.setBrightness(ledBrightness);
    ledStrip.show();
}

void LedController::render(uint32_t nowMs)
{
    if (!ledEnabled)
    {
        ledStrip.clear();
        ledStrip.show();
        return;
    }

    switch (ledMode)
    {
    case LedMode::Static:
        renderStatic();
        return;
    case LedMode::Breathing:
        renderBreathing(nowMs);
        return;
    case LedMode::Loop:
        renderLoop(nowMs);
        return;
    case LedMode::Rainbow:
        renderRainbow(nowMs);
        return;
    }

    // Fallback
    renderStatic();
}
