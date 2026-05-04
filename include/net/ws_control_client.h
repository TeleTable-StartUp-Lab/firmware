#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <functional>

namespace WsControlClient
{
    struct Handlers
    {
        std::function<void()> onConnected;
        std::function<void()> onDisconnected;

        std::function<void(const String &startNode, const String &destinationNode)> onNavigate;
        std::function<void(float linearVelocity, float angularVelocity)> onDriveCommand;
        std::function<void(int32_t maxSpeedPercent)> onSetManualSpeedCap;
        std::function<void(bool enabled, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness, const String &mode)> onLed;
        std::function<void(uint32_t hz, uint32_t ms)> onAudioBeep;
        std::function<void(float value)> onAudioVolume;
        std::function<void(uint32_t sampleRateHz, uint8_t channels, uint8_t bitsPerSample, bool littleEndian)> onAudioStreamStart;
        std::function<void()> onAudioStreamStop;
        std::function<void(const uint8_t *data, size_t len)> onAudioStreamData;

        std::function<void()> onStop;
        std::function<void(const String &mode)> onSetMode;

        std::function<void(const String &command, const JsonDocument &raw)> onUnknownCommand;
    };

    void begin(const Handlers &handlers);
    void loop();

    bool isConnected();

    bool sendJson(const JsonDocument &doc);
    bool sendText(const String &text);

    void disconnect();
}
