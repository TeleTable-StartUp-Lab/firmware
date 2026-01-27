#pragma once
#include <Arduino.h>
#include <functional>

namespace WsControlClient
{

    struct TankCmd
    {
        float throttle;
        float steer;
    };

    using TankHandler = std::function<void(const TankCmd &)>;
    using StopHandler = std::function<void()>;
    using ModeHandler = std::function<void(const String &)>;
    using LedColorHandler = std::function<void(uint8_t r, uint8_t g, uint8_t b)>;
    using VolumeHandler = std::function<void(uint8_t percent)>;
    using BeepHandler = std::function<void(uint16_t hz, uint16_t ms)>;

    struct Handlers
    {
        TankHandler onTank;
        StopHandler onStop;
        ModeHandler onMode;
        LedColorHandler onLedColor;
        VolumeHandler onVolume;
        BeepHandler onBeep;
    };

    void begin(const char *host, uint16_t port, const char *path, const Handlers &handlers);
    void loop();
    bool isConnected();

}
