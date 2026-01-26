#pragma once
#include <Arduino.h>
#include "core/system_state.h"
#include "hardware/bh1750.h"
#include "hardware/ir_sensors.h"
#include "hardware/leds.h"
#include "network/backend_client.h"
#include "network/wifi_manager.h"

class RuntimeTasks
{
public:
    RuntimeTasks(SystemState *state,
                 BH1750Sensor *light,
                 LEDController *leds,
                 BackendClient *backend,
                 WifiManager *wifi);

    void begin();
    void start();
    void stop();

private:
    static void sensorTaskTrampoline(void *arg);
    static void backendTaskTrampoline(void *arg);
    static void wifiTaskTrampoline(void *arg);

    void sensorTaskLoop();
    void backendTaskLoop();
    void wifiTaskLoop();

    void handleAutomaticLighting(float lux);

    SystemState *_state;
    BH1750Sensor *_light;
    LEDController *_leds;
    BackendClient *_backend;
    WifiManager *_wifi;

    TaskHandle_t _sensorTask = nullptr;
    TaskHandle_t _backendTask = nullptr;
    TaskHandle_t _wifiTask = nullptr;

    bool _lightsActive = false;
};
