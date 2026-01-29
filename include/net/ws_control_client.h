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
