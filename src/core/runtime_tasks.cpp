#include "core/runtime_tasks.h"
#include <FastLED.h>
#include "utils/logger.h"

RuntimeTasks::RuntimeTasks(SystemState *state,
                           BH1750Sensor *light,
                           LEDController *leds,
                           BackendClient *backend,
                           WifiManager *wifi)
    : _state(state), _light(light), _leds(leds), _backend(backend), _wifi(wifi) {}

void RuntimeTasks::begin()
{
}

void RuntimeTasks::start()
{
    if (!_sensorTask)
    {
        xTaskCreatePinnedToCore(sensorTaskTrampoline, "SensorTask", 4096, this, 2, &_sensorTask, 1);
    }
    if (!_backendTask)
    {
        xTaskCreatePinnedToCore(backendTaskTrampoline, "BackendTask", 4096, this, 1, &_backendTask, 0);
    }
    if (!_wifiTask)
    {
        xTaskCreatePinnedToCore(wifiTaskTrampoline, "WifiTask", 3072, this, 1, &_wifiTask, 0);
    }
}

void RuntimeTasks::stop()
{
    if (_sensorTask)
    {
        vTaskDelete(_sensorTask);
        _sensorTask = nullptr;
    }
    if (_backendTask)
    {
        vTaskDelete(_backendTask);
        _backendTask = nullptr;
    }
    if (_wifiTask)
    {
        vTaskDelete(_wifiTask);
        _wifiTask = nullptr;
    }
}

void RuntimeTasks::sensorTaskTrampoline(void *arg)
{
    static_cast<RuntimeTasks *>(arg)->sensorTaskLoop();
}

void RuntimeTasks::backendTaskTrampoline(void *arg)
{
    static_cast<RuntimeTasks *>(arg)->backendTaskLoop();
}

void RuntimeTasks::wifiTaskTrampoline(void *arg)
{
    static_cast<RuntimeTasks *>(arg)->wifiTaskLoop();
}

void RuntimeTasks::handleAutomaticLighting(float lux)
{
    const float LUX_ON = 25.0f;
    const float LUX_OFF = 40.0f;

    if (!_lightsActive && lux < LUX_ON && lux >= 0)
    {
        _leds->setAllColors(CRGB(200, 150, 50));
        FastLED.show();
        _lightsActive = true;
    }
    else if (_lightsActive && lux > LUX_OFF)
    {
        _leds->clear();
        FastLED.show();
        _lightsActive = false;
    }
}

void RuntimeTasks::sensorTaskLoop()
{
    for (;;)
    {
        const float lux = _light->readLight();
        const bool obsL = (IR::readLeft() == 0);
        const bool obsR = (IR::readRight() == 0);

        _state->lock();
        _state->lux = lux;
        _state->obstacleLeft = obsL;
        _state->obstacleRight = obsR;
        _state->lastStatusUpdate = millis();
        _state->unlock();

        handleAutomaticLighting(lux);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void RuntimeTasks::backendTaskLoop()
{
    for (;;)
    {
        _backend->loop();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void RuntimeTasks::wifiTaskLoop()
{
    for (;;)
    {
        const bool ok = _wifi->isConnected();

        _state->lock();
        _state->systemHealth = ok ? "OK" : "WiFi Error";
        _state->unlock();

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
